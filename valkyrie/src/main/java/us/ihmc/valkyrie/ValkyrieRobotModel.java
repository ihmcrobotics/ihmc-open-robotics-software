package us.ihmc.valkyrie;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.Arrays;
import java.util.function.Consumer;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.drcRobot.SimulationLowLevelControllerFactory;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.avatar.reachabilityMap.footstep.StepReachabilityIOHelper;
import us.ihmc.avatar.ros.RobotROSClockCalculator;
import us.ihmc.avatar.ros.WallTimeBasedROSClockCalculator;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityData;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.DefaultSwingPlannerParameters;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.modelFileLoaders.SdfLoader.SDFModelLoader;
import us.ihmc.multicastLogDataProtocol.modelLoaders.DefaultLogModelProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelWrapper;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationToolkit.RobotDefinitionTools;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.diagnostic.ValkyrieDiagnosticParameters;
import us.ihmc.valkyrie.fingers.SimulatedValkyrieFingerControlThread;
import us.ihmc.valkyrie.fingers.ValkyrieHandModel;
import us.ihmc.valkyrie.parameters.*;
import us.ihmc.valkyrie.sensors.ValkyrieSensorSuiteManager;
import us.ihmc.wholeBodyController.FootContactPoints;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.UIParameters;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticParameters;

public class ValkyrieRobotModel implements DRCRobotModel
{
   private static final boolean PRINT_MODEL = false;

   private final String[] resourceDirectories = {"models/", "models/gazebo/", "models/val_description/", "models/val_description/sdf/"};

   private final ValkyrieRobotVersion robotVersion;
   private final RobotTarget target;

   private double controllerDT;
   private double estimatorDT;
   private double simulateDT;

   private double modelSizeScale = 1.0;
   private double modelMassScale = 1.0;
   private MaterialDefinition robotMaterial = null;
   private boolean useOBJGraphics = true;
   private String customModel = null;
   private FootContactPoints<RobotSide> simulationContactPoints = null;
   private RobotDefinition robotDefinition;
   private Consumer<RobotDefinition> robotDefinitionMutator;

   private ValkyriePhysicalProperties robotPhysicalProperties;
   private ValkyrieJointMap jointMap;
   private RobotContactPointParameters<RobotSide> contactPointParameters;
   private ValkyrieSensorInformation sensorInformation;
   private HighLevelControllerParameters highLevelControllerParameters;
   private ValkyrieCalibrationParameters calibrationParameters;

   private CoPTrajectoryParameters copTrajectoryParameters;
   private WalkingControllerParameters walkingControllerParameters;
   private PushRecoveryControllerParameters pushRecoveryControllerParameters;
   private StateEstimatorParameters stateEstimatorParameters;
   private WallTimeBasedROSClockCalculator rosClockCalculator;
   private RobotInitialSetup<HumanoidFloatingRootJointRobot> valkyrieInitialSetup;
   private StepReachabilityData stepReachabilityData;

   private SimulationLowLevelControllerFactory simulationLowLevelControllerFactory;

   private ValkyrieSensorSuiteManager sensorSuiteManager = null;

   public ValkyrieRobotModel(RobotTarget target)
   {
      this(target, ValkyrieRobotVersion.DEFAULT);
   }

   public ValkyrieRobotModel(RobotTarget target, ValkyrieRobotVersion robotVersion)
   {
      this.target = target;
      this.robotVersion = robotVersion;

      controllerDT = target == RobotTarget.SCS ? 0.004 : 0.006;
      estimatorDT = 0.002;
      simulateDT = estimatorDT / 3.0;
   }

   public void setVal2Scale()
   {
      setModelSizeScale(0.925170);
      setModelMassScale(0.925170);
   }

   public ValkyrieRobotVersion getRobotVersion()
   {
      return robotVersion;
   }

   @Override
   public RobotTarget getTarget()
   {
      return target;
   }

   public ValkyriePhysicalProperties getRobotPhysicalProperties()
   {
      if (robotPhysicalProperties == null)
         robotPhysicalProperties = new ValkyriePhysicalProperties(modelSizeScale, modelMassScale);
      return robotPhysicalProperties;
   }

   @Override
   public ValkyrieJointMap getJointMap()
   {
      if (jointMap == null)
         jointMap = new ValkyrieJointMap(getRobotPhysicalProperties(), robotVersion);
      return jointMap;
   }

   /**
    * Sets the period of a controller tick in second.
    *
    * @param controllerDT the period of the controller thread in second.
    */
   public void setControllerDT(double controllerDT)
   {
      this.controllerDT = controllerDT;
   }

   /**
    * Sets the period of a estimator tick in second.
    *
    * @param estimatorDT the period of the estimator thread in second.
    */
   public void setEstimatorDT(double estimatorDT)
   {
      this.estimatorDT = estimatorDT;
   }

   /**
    * Sets the period of a simulation tick in second.
    *
    * @param simulateDT the period of the simulation thread in second.
    */
   public void setSimulateDT(double simulateDT)
   {
      this.simulateDT = simulateDT;
   }

   /**
    * Overrides the transparency of the robot visuals.
    *
    * @param transparency the new transparency.
    */
   public void setTransparency(double transparency)
   {
      setDiffuseColor(ColorDefinitions.LightGreen().derive(0, 1.0, 1.0, 1.0 - transparency));
   }

   /**
    * Overrides the robot visuals with a material that has the given diffuse color.
    *
    * @param diffuseColor the new diffuse color for the robot's meshes.
    */
   public void setDiffuseColor(ColorDefinition diffuseColor)
   {
      setMaterial(new MaterialDefinition(diffuseColor));
   }

   /**
    * Overrides the robot visuals with the given material.
    *
    * @param robotMaterial the new material for the robot's meshes, default value {@link null}.
    */
   public void setMaterial(MaterialDefinition robotMaterial)
   {
      if (robotDefinition != null)
         throw new IllegalArgumentException("Cannot set robotMaterial once robotDefinition has been created.");
      this.robotMaterial = robotMaterial;
   }

   /**
    * Indicates whether to use the OBJ or DAE meshes for the robot visuals.
    *
    * @param useOBJGraphics switch to use OBJ when {@code true}, use DAE otherwise. Default value is
    *                       {@code true}.
    */
   public void setUseOBJGraphics(boolean useOBJGraphics)
   {
      if (robotDefinition != null)
         throw new IllegalArgumentException("Cannot change to use OBJ graphics once robotDefinition has been created.");
      this.useOBJGraphics = useOBJGraphics;
   }

   /**
    * Overrides the default set of contact points to use for the simulation for the feet.
    *
    * @param simulationContactPoints the new set of foot contact points. Default value is {@code null}.
    */
   public void setSimulationContactPoints(FootContactPoints<RobotSide> simulationContactPoints)
   {
      if (contactPointParameters != null)
         throw new IllegalArgumentException("Cannot set simulation contact points once contactPointParameters has been created.");
      this.simulationContactPoints = simulationContactPoints;
   }

   /**
    * Overrides the model to load.
    *
    * @param customModel the file's fullname of the model to load. Default value is {@code null}.
    */
   public void setCustomModel(String customModel)
   {
      if (robotDefinition != null)
         throw new IllegalArgumentException("Cannot set customModel once generalizedRobotModel has been created.");
      this.customModel = customModel;
   }

   /**
    * Scale to apply to the robot mass.
    *
    * @param modelMassScale the new scale value. Default value is {@code 1.0}.
    */
   public void setModelMassScale(double modelMassScale)
   {
      if (robotPhysicalProperties != null)
         throw new IllegalArgumentException("Cannot set modelMassScale once robotPhysicalProperties has been created.");
      this.modelMassScale = modelMassScale;
   }

   /**
    * Scale to apply to the robot size.
    *
    * @param modelSizeScale the new scale value. Default value is {@code 1.0}.
    */
   public void setModelSizeScale(double modelSizeScale)
   {
      if (robotPhysicalProperties != null)
         throw new IllegalArgumentException("Cannot set modelSizeScale once robotPhysicalProperties has been created.");
      this.modelSizeScale = modelSizeScale;
   }

   public void disableOneDoFJointDamping()
   {
      setRobotDefinitionMutator(getRobotDefinitionMutator().andThen(def -> def.forEachOneDoFJointDefinition(joint -> joint.setDamping(0.0))));
   }

   public void setRobotDefinitionMutator(Consumer<RobotDefinition> robotDefinitionMutator)
   {
      if (robotDefinition != null)
         throw new IllegalArgumentException("Cannot set customModel once robotDefinition has been created.");
      this.robotDefinitionMutator = robotDefinitionMutator;
   }

   public void setRobotInitialSetup(RobotInitialSetup<HumanoidFloatingRootJointRobot> valkyrieInitialSetup)
   {
      if (this.valkyrieInitialSetup != null)
         throw new IllegalArgumentException("Cannot set valkyrieInitialSetup once it has been created.");
      this.valkyrieInitialSetup = valkyrieInitialSetup;
   }

   public Consumer<RobotDefinition> getRobotDefinitionMutator()
   {
      if (robotDefinitionMutator == null)
         robotDefinitionMutator = new ValkyrieRobotDefinitionMutator(getJointMap(), useOBJGraphics);
      return robotDefinitionMutator;
   }

   @Override
   public RobotDefinition getRobotDefinition()
   {
      if (robotDefinition == null)
      {
         robotDefinition = RobotDefinitionTools.loadSDFModel(getSDFModelInputStream(),
                                                             Arrays.asList(getResourceDirectories()),
                                                             getClass().getClassLoader(),
                                                             getJointMap().getModelName(),
                                                             getContactPointParameters(),
                                                             getJointMap(),
                                                             true);
         if (robotMaterial != null)
            RobotDefinitionTools.setRobotDefinitionMaterial(robotDefinition, robotMaterial);
         getRobotDefinitionMutator().accept(robotDefinition);
      }

      return robotDefinition;
   }

   private String[] getResourceDirectories()
   {
      return resourceDirectories;
   }

   private InputStream getSDFModelInputStream()
   {
      InputStream inputStream = null;

      if (customModel != null)
      {

         System.out.println("Loading robot model from: '" + customModel + "'");
         inputStream = getClass().getClassLoader().getResourceAsStream(customModel);

         if (inputStream == null)
         {
            try
            {
               inputStream = new FileInputStream(customModel);
            }
            catch (FileNotFoundException e)
            {
               throw new RuntimeException(e);
            }
         }
      }
      else
      {
         String sdfFile = null;

         if (target == RobotTarget.REAL_ROBOT)
            sdfFile = robotVersion.getRealRobotSdfFile();
         else
            sdfFile = robotVersion.getSimSdfFile();

         inputStream = getClass().getClassLoader().getResourceAsStream(sdfFile);
      }

      return inputStream;
   }

   @Override
   public RobotInitialSetup<HumanoidFloatingRootJointRobot> getDefaultRobotInitialSetup()
   {
      if (valkyrieInitialSetup == null)
         valkyrieInitialSetup = new ValkyrieInitialSetup(getRobotDefinition(), getJointMap());
      return valkyrieInitialSetup;
   }

   public ValkyrieHandModel getHandModel()
   {
      return getHandModel(null);
   }

   public ValkyrieHandModel getHandModel(RobotSide side)
   {
      return new ValkyrieHandModel();
   }

   @Override
   public FullHumanoidRobotModel createFullRobotModel()
   {
      return new FullHumanoidRobotModelWrapper(getRobotDefinition(), getJointMap());
   }

   @Override
   public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes, boolean enableJointDamping)
   {
      boolean enableTorqueVelocityLimits = false;

      HumanoidFloatingRootJointRobot robot = new HumanoidFloatingRootJointRobot(getRobotDefinition(),
                                                                                getJointMap(),
                                                                                enableJointDamping,
                                                                                enableTorqueVelocityLimits);

      if (PRINT_MODEL)
      {
         System.out.println("\nValkyrieRobotModel Link Masses:");

         StringBuffer stringBuffer = new StringBuffer();
         robot.printRobotJointsAndMasses(stringBuffer);
         System.out.println(stringBuffer);
         System.out.println();

         System.out.println("ValkyrieRobotModel: \n" + robot);
      }

      return robot;
   }

   @Override
   public double getSimulateDT()
   {
      return simulateDT;
   }

   @Override
   public double getEstimatorDT()
   {
      return estimatorDT;
   }

   @Override
   public double getControllerDT()
   {
      return controllerDT;
   }

   @Override
   public RobotROSClockCalculator getROSClockCalculator()
   {
      if (rosClockCalculator == null)
         rosClockCalculator = new WallTimeBasedROSClockCalculator();
      return rosClockCalculator;
   }

   @Override
   public ValkyrieSensorSuiteManager getSensorSuiteManager()
   {
      return getSensorSuiteManager(null);
   }

   @Override
   public ValkyrieSensorSuiteManager getSensorSuiteManager(ROS2NodeInterface ros2Node)
   {
      if (sensorSuiteManager == null)
      {
         sensorSuiteManager = new ValkyrieSensorSuiteManager(getSimpleRobotName(),
                                                             this,
                                                             getCollisionBoxProvider(),
                                                             getROSClockCalculator(),
                                                             getSensorInformation(),
                                                             getJointMap(),
                                                             target,
                                                             ros2Node);
      }
      return sensorSuiteManager;
   }

   @Override
   public SimulatedValkyrieFingerControlThread createSimulatedHandController(RealtimeROS2Node realtimeROS2Node)
   {
      if (!robotVersion.hasFingers())
         return null;

      return new SimulatedValkyrieFingerControlThread(createFullRobotModel(),
                                                      realtimeROS2Node,
                                                      ROS2Tools.getControllerOutputTopic(getSimpleRobotName()),
                                                      ROS2Tools.getControllerInputTopic(getSimpleRobotName()));
   }

   @Override
   public LogModelProvider getLogModelProvider()
   {
      return new DefaultLogModelProvider<>(SDFModelLoader.class, getJointMap().getModelName(), getSDFModelInputStream(), getResourceDirectories());
   }

   @Override
   public DataServerSettings getLogSettings()
   {
      if (target == RobotTarget.REAL_ROBOT)
      {
         return new DataServerSettings(true, "ValkyrieJSCGUI");
      }
      else
      {
         return new DataServerSettings(false, "SimulationGUI");
      }
   }

   @Override
   public CollisionBoxProvider getCollisionBoxProvider()
   {
      return new ValkyrieCollisionBoxProvider(createFullRobotModel());
   }

   @Override
   public InputStream getWholeBodyControllerParametersFile()
   {
      switch (target)
      {
         case SCS:
            return getClass().getResourceAsStream(getSimulationParameterResourceName());
         case GAZEBO:
         case REAL_ROBOT:
            return getClass().getResourceAsStream(getHardwareParameterResourceName());
         default:
            throw new UnsupportedOperationException("Unsupported target: " + target);
      }
   }

   public static String getSimulationParameterResourceName()
   {
      return "/us/ihmc/valkyrie/parameters/controller_simulation.xml";
   }

   public static String getHardwareParameterResourceName()
   {
      return "/us/ihmc/valkyrie/parameters/controller_hardware.xml";
   }

   @Override
   public FootstepPlannerParametersBasics getFootstepPlannerParameters()
   {
      return new ValkyrieFootstepPlannerParameters();
   }

   @Override
   public FootstepPlannerParametersBasics getFootstepPlannerParameters(String fileNameSuffix)
   {
      return new ValkyrieFootstepPlannerParameters(fileNameSuffix);
   }

   @Override
   public VisibilityGraphsParametersBasics getVisibilityGraphsParameters()
   {
      return new DefaultVisibilityGraphParameters();
   }

   @Override
   public SwingPlannerParametersBasics getSwingPlannerParameters()
   {
      return new DefaultSwingPlannerParameters();
   }

   @Override
   public SwingPlannerParametersBasics getSwingPlannerParameters(String fileNameSuffix)
   {
      return new ValkyrieSwingPlannerParameters(fileNameSuffix);
   }

   @Override
   public String getStepReachabilityResourceName()
   {
      return "us/ihmc/valkyrie/parameters/StepReachabilityMap.json";
   }

   @Override
   public StepReachabilityData getStepReachabilityData()
   {
      if (stepReachabilityData == null)
      {
         stepReachabilityData = new StepReachabilityIOHelper().loadStepReachability(this);
      }

      return stepReachabilityData;
   }

   @Override
   public RobotContactPointParameters<RobotSide> getContactPointParameters()
   {
      if (contactPointParameters == null)
         contactPointParameters = new ValkyrieContactPointParameters(getJointMap(), getRobotPhysicalProperties(), simulationContactPoints);
      return contactPointParameters;
   }

   @Override
   public ValkyrieSensorInformation getSensorInformation()
   {
      if (sensorInformation == null)
         sensorInformation = new ValkyrieSensorInformation(getRobotPhysicalProperties(), target);
      return sensorInformation;
   }

   @Override
   public HighLevelControllerParameters getHighLevelControllerParameters()
   {
      if (highLevelControllerParameters == null)
         highLevelControllerParameters = new ValkyrieHighLevelControllerParameters(target, getJointMap());
      return highLevelControllerParameters;
   }

   public ValkyrieCalibrationParameters getCalibrationParameters()
   {
      if (calibrationParameters == null)
         calibrationParameters = new ValkyrieCalibrationParameters(getJointMap());
      return calibrationParameters;
   }

   @Override
   public CoPTrajectoryParameters getCoPTrajectoryParameters()
   {
      if (copTrajectoryParameters == null)
         copTrajectoryParameters = new ValkyrieCoPTrajectoryParameters();
      return copTrajectoryParameters;
   }

   public void setWalkingControllerParameters(WalkingControllerParameters walkingControllerParameters)
   {
      this.walkingControllerParameters = walkingControllerParameters;
   }

   public void setContactPointParameters(RobotContactPointParameters<RobotSide> contactPointParameters)
   {
      this.contactPointParameters = contactPointParameters;
   }

   public void setHighLevelControllerParameters(HighLevelControllerParameters highLevelControllerParameters)
   {
      this.highLevelControllerParameters = highLevelControllerParameters;
   }

   public void setSimulationLowLevelControllerFactory(SimulationLowLevelControllerFactory simulationLowLevelControllerFactory)
   {
      if (this.simulationLowLevelControllerFactory != null)
         throw new IllegalArgumentException("Cannot set low-level controller factory once simulation has been setup.");
      this.simulationLowLevelControllerFactory = simulationLowLevelControllerFactory;
   }

   @Override
   public WalkingControllerParameters getWalkingControllerParameters()
   {
      if (walkingControllerParameters == null)
         walkingControllerParameters = new ValkyrieWalkingControllerParameters(getJointMap(), getRobotPhysicalProperties(), target);
      return walkingControllerParameters;
   }

   @Override
   public PushRecoveryControllerParameters getPushRecoveryControllerParameters()
   {
      if (pushRecoveryControllerParameters == null)
         pushRecoveryControllerParameters = new ValkyriePushRecoveryControllerParameters(getJointMap());
      return pushRecoveryControllerParameters;
   }

   @Override
   public StateEstimatorParameters getStateEstimatorParameters()
   {
      if (stateEstimatorParameters == null)
         stateEstimatorParameters = new ValkyrieStateEstimatorParameters(target, getEstimatorDT(), getSensorInformation(), getJointMap());
      return stateEstimatorParameters;
   }

   @Override
   public RobotCollisionModel getHumanoidRobotKinematicsCollisionModel()
   {
      return new ValkyrieKinematicsCollisionModel(getJointMap());
   }

   @Override
   public RobotCollisionModel getSimulationRobotCollisionModel(CollidableHelper helper, String robotCollisionMask, String... environmentCollisionMasks)
   {
      if (robotVersion == ValkyrieRobotVersion.ARM_MASS_SIM)
      {
         ValkyrieArmMassSimCollisionModel collisionModel = new ValkyrieArmMassSimCollisionModel(getJointMap(), true);
         collisionModel.setCollidableHelper(helper, robotCollisionMask, environmentCollisionMasks);
         return collisionModel;
      }
      else
      {
         ValkyrieSimulationCollisionModel collisionModel = new ValkyrieSimulationCollisionModel(getJointMap());
         collisionModel.setCollidableHelper(helper, robotCollisionMask, environmentCollisionMasks);
         return collisionModel;
      }
   }

   @Override
   public SimulationLowLevelControllerFactory getSimulationLowLevelControllerFactory()
   {
      if (simulationLowLevelControllerFactory == null)
         simulationLowLevelControllerFactory = DRCRobotModel.super.getSimulationLowLevelControllerFactory();
      return simulationLowLevelControllerFactory;
   }

   @Override
   public DiagnosticParameters getDiagnoticParameters()
   {
      return new ValkyrieDiagnosticParameters(getJointMap(), getSensorInformation(), target == RobotTarget.REAL_ROBOT);
   }

   @Override
   public UIParameters getUIParameters()
   {
      return new ValkyrieUIParameters(robotVersion, getRobotPhysicalProperties(), getJointMap());
   }

   @Override
   public String getSimpleRobotName()
   {
      return "Valkyrie";
   }

   @Override
   public String toString()
   {
      return getSimpleRobotName();
   }

   @Override
   public RobotLowLevelMessenger newRobotLowLevelMessenger(ROS2NodeInterface ros2Node)
   {
      return new ValkyrieDirectRobotInterface(ros2Node, this);
   }
}
