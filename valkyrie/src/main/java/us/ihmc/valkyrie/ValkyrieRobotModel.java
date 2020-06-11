package us.ihmc.valkyrie;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.LinkedHashMap;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.drcRobot.shapeContactSettings.DRCRobotModelShapeCollisionSettings;
import us.ihmc.avatar.factory.SimulatedHandControlTask;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.ros.RobotROSClockCalculator;
import us.ihmc.avatar.ros.WallTimeBasedROSClockCalculator;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.SliderBoardParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.icp.DefaultSplitFractionCalculatorParameters;
import us.ihmc.footstepPlanning.icp.SplitFractionCalculatorParametersBasics;
import us.ihmc.footstepPlanning.swing.DefaultSwingPlannerParameters;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.QuadTreeFootstepPlanningParameters;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.modelFileLoaders.SdfLoader.*;
import us.ihmc.multicastLogDataProtocol.modelLoaders.DefaultLogModelProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFromDescription;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.ContactSensorType;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.fingers.SimulatedValkyrieFingerController;
import us.ihmc.valkyrie.fingers.ValkyrieHandModel;
import us.ihmc.valkyrie.parameters.*;
import us.ihmc.valkyrie.sensors.ValkyrieSensorSuiteManager;
import us.ihmc.wholeBodyController.FootContactPoints;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.UIParameters;

public class ValkyrieRobotModel implements DRCRobotModel
{
   private static final boolean PRINT_MODEL = false;

   private final String[] resourceDirectories = {"models/", "models/gazebo/", "models/val_description/", "models/val_description/sdf/"};

   private final ValkyrieRobotVersion robotVersion;
   private final RobotTarget target;

   private double modelSizeScale = 1.0;
   private double modelMassScale = 1.0;
   private double transparency = Double.NaN;
   private boolean useShapeCollision = false;
   private boolean useOBJGraphics = true;
   private String customModel = null;
   private FootContactPoints<RobotSide> simulationContactPoints = null;
   private GeneralizedSDFRobotModel generalizedRobotModel;
   private RobotDescription robotDescription;
   private SDFDescriptionMutator sdfDescriptionMutator;

   private ValkyriePhysicalProperties robotPhysicalProperties;
   private ValkyrieJointMap jointMap;
   private RobotContactPointParameters<RobotSide> contactPointParameters;
   private ValkyrieSensorInformation sensorInformation;
   private HighLevelControllerParameters highLevelControllerParameters;
   private ValkyrieCalibrationParameters calibrationParameters;

   private ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters;
   private WalkingControllerParameters walkingControllerParameters;
   private StateEstimatorParameters stateEstimatorParameters;
   private WallTimeBasedROSClockCalculator rosClockCalculator;
   private ValkyrieRobotModelShapeCollisionSettings robotModelShapeCollisionSettings;
   private DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> valkyrieInitialSetup;

   private ValkyrieSensorSuiteManager sensorSuiteManager = null;

   public ValkyrieRobotModel(RobotTarget target)
   {
      this(target, ValkyrieRobotVersion.DEFAULT);
   }

   public ValkyrieRobotModel(RobotTarget target, ValkyrieRobotVersion robotVersion)
   {
      this.target = target;
      this.robotVersion = robotVersion;
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
    * Overrides the transparency of the robot visuals.
    *
    * @param transparency the new transparency, default value {@link Double#NaN}.
    */
   public void setTransparency(double transparency)
   {
      if (robotDescription != null)
         throw new IllegalArgumentException("Cannot set transparency once robotDescription has been created.");
      this.transparency = transparency;
   }

   /**
    * Indicates whether to use the OBJ or DAE meshes for the robot visuals.
    *
    * @param useOBJGraphics switch to use OBJ when {@code true}, use DAE otherwise. Default value is
    *                       {@code true}.
    */
   public void setUseOBJGraphics(boolean useOBJGraphics)
   {
      if (generalizedRobotModel != null)
         throw new IllegalArgumentException("Cannot change to use OBJ graphics once generalizedRobotModel has been created.");
      this.useOBJGraphics = useOBJGraphics;
   }

   /**
    * Sets whether the simulation contact engine should use point-to-shape or shape-to-shape model.
    *
    * @param useShapeCollision switch to use shape-to-shape when {@code true}, use point-to-shape
    *                          otherwise. Default value is {@code false}.
    */
   public void setUseShapeCollision(boolean useShapeCollision)
   {
      if (robotModelShapeCollisionSettings != null)
         throw new IllegalArgumentException("Cannot change to use shape collision once robotModelShapeCollisionSettings has been created.");
      if (robotDescription != null)
         throw new IllegalArgumentException("Cannot change to use shape collision once robotDescription has been created.");
      this.useShapeCollision = useShapeCollision;
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
      if (generalizedRobotModel != null)
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

   public void setSDFDescriptionMutator(SDFDescriptionMutator sdfDescriptionMutator)
   {
      if (generalizedRobotModel != null)
         throw new IllegalArgumentException("Cannot set customModel once generalizedRobotModel has been created.");
      this.sdfDescriptionMutator = sdfDescriptionMutator;
   }

   public void setRobotInitialSetup(DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> valkyrieInitialSetup)
   {
      if (this.valkyrieInitialSetup != null)
         throw new IllegalArgumentException("Cannot set valkyrieInitialSetup once it has been created.");
      this.valkyrieInitialSetup = valkyrieInitialSetup;
   }

   public SDFDescriptionMutator getSDFDescriptionMutator()
   {
      if (sdfDescriptionMutator == null)
         sdfDescriptionMutator = new ValkyrieSDFDescriptionMutator(getJointMap(), useOBJGraphics);
      return sdfDescriptionMutator;
   }

   public GeneralizedSDFRobotModel getGeneralizedRobotModel()
   {
      if (generalizedRobotModel == null)
      {
         JaxbSDFLoader loader = DRCRobotSDFLoader.loadDRCRobot(getResourceDirectories(), getSDFModelInputStream(), getSDFDescriptionMutator());

         for (String forceSensorName : ValkyrieSensorInformation.forceSensorNames)
         {
            RigidBodyTransform transform = new RigidBodyTransform(ValkyrieSensorInformation.getForceSensorTransform(forceSensorName));
            loader.addForceSensor(getJointMap(), forceSensorName, forceSensorName, transform);
         }

         for (RobotSide side : RobotSide.values)
         {
            LinkedHashMap<String, LinkedHashMap<String, ContactSensorType>> contactsensors = ValkyrieSensorInformation.contactSensors.get(side);

            for (String parentJointName : contactsensors.keySet())
            {
               for (String sensorName : contactsensors.get(parentJointName).keySet())
               {
                  loader.addContactSensor(getJointMap(), sensorName, parentJointName, contactsensors.get(parentJointName).get(sensorName));
               }
            }
         }

         generalizedRobotModel = loader.getGeneralizedSDFRobotModel(getJointMap().getModelName());
      }

      return generalizedRobotModel;
   }

   @Override
   public RobotDescription getRobotDescription()
   {
      if (robotDescription == null)
      {
         boolean useCollisionMeshes = false;

         GeneralizedSDFRobotModel generalizedSDFRobotModel = getGeneralizedRobotModel();
         RobotDescriptionFromSDFLoader descriptionLoader = new RobotDescriptionFromSDFLoader();

         if (useShapeCollision)
         {
            robotDescription = descriptionLoader.loadRobotDescriptionFromSDF(generalizedSDFRobotModel, getJointMap(), null, false, transparency);
            ValkyrieCollisionMeshDefinitionDataHolder collisionMeshDefinitionDataHolder = new ValkyrieCollisionMeshDefinitionDataHolder(getJointMap(),
                                                                                                                                        getRobotPhysicalProperties());
            collisionMeshDefinitionDataHolder.setVisible(false);

            robotDescription.addCollisionMeshDefinitionData(collisionMeshDefinitionDataHolder);
         }
         else
         {
            robotDescription = descriptionLoader.loadRobotDescriptionFromSDF(generalizedSDFRobotModel,
                                                                             getJointMap(),
                                                                             getContactPointParameters(),
                                                                             useCollisionMeshes,
                                                                             transparency);
         }
      }
      return robotDescription;
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
   public DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw)
   {
      if (valkyrieInitialSetup == null)
         valkyrieInitialSetup = new ValkyrieInitialSetup();
      valkyrieInitialSetup.setInitialGroundHeight(groundHeight);
      valkyrieInitialSetup.setInitialYaw(initialYaw);
      return valkyrieInitialSetup;
   }

   @Override
   public ValkyrieHandModel getHandModel()
   {
      return new ValkyrieHandModel();
   }

   @Override
   public FullHumanoidRobotModel createFullRobotModel()
   {
      return new FullHumanoidRobotModelFromDescription(getRobotDescription(), getJointMap(), getSensorInformation().getSensorFramesToTrack());
   }

   @Override
   public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes, boolean enableJointDamping)
   {
      boolean enableTorqueVelocityLimits = false;

      HumanoidFloatingRootJointRobot sdfRobot = new HumanoidFloatingRootJointRobot(getRobotDescription(),
                                                                                   getJointMap(),
                                                                                   enableJointDamping,
                                                                                   enableTorqueVelocityLimits);

      if (PRINT_MODEL)
      {
         System.out.println("\nValkyrieRobotModel Link Masses:");

         StringBuffer stringBuffer = new StringBuffer();
         sdfRobot.printRobotJointsAndMasses(stringBuffer);
         System.out.println(stringBuffer);
         System.out.println();

         System.out.println("ValkyrieRobotModel: \n" + sdfRobot);
      }

      return sdfRobot;
   }

   @Override
   public double getSimulateDT()
   {
      return getEstimatorDT() / 3.0;
   }

   @Override
   public double getEstimatorDT()
   {
      return 0.002;
   }

   @Override
   public double getControllerDT()
   {
      if (target == RobotTarget.SCS)
         return 0.004;
      else
         return 0.006;
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
      if (sensorSuiteManager == null)
      {
         sensorSuiteManager = new ValkyrieSensorSuiteManager(getSimpleRobotName(),
                                                             this,
                                                             getCollisionBoxProvider(),
                                                             getROSClockCalculator(),
                                                             getSensorInformation(),
                                                             getJointMap(),
                                                             target);
      }
      return sensorSuiteManager;
   }

   @Override
   public SimulatedHandControlTask createSimulatedHandController(FloatingRootJointRobot simulatedRobot, RealtimeRos2Node realtimeRos2Node)
   {
      if (!robotVersion.hasFingers())
         return null;

      boolean hasFingers = true;
      for (RobotSide robotSide : RobotSide.values)
      {
         String handLinkName = getJointMap().getHandName(robotSide);
         hasFingers = hasFingers && !simulatedRobot.getLink(handLinkName).getParentJoint().getChildrenJoints().isEmpty();
      }

      if (hasFingers)
      {
         return new SimulatedValkyrieFingerController(simulatedRobot,
                                                      realtimeRos2Node,
                                                      this, ROS2Tools.getControllerOutputTopic(getSimpleRobotName()),
                                                      ROS2Tools.getControllerInputTopic(getSimpleRobotName()));
      }
      else
      {
         return null;
      }
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
   public DRCRobotModelShapeCollisionSettings getShapeCollisionSettings()
   {
      if (robotModelShapeCollisionSettings == null)
         robotModelShapeCollisionSettings = new ValkyrieRobotModelShapeCollisionSettings(useShapeCollision);
      return robotModelShapeCollisionSettings;
   }

   @Override
   public InputStream getWholeBodyControllerParametersFile()
   {
      switch (target)
      {
         case SCS:
            return getClass().getResourceAsStream("/us/ihmc/valkyrie/parameters/controller_simulation.xml");
         case GAZEBO:
         case REAL_ROBOT:
            return getClass().getResourceAsStream("/us/ihmc/valkyrie/parameters/controller_hardware.xml");
         default:
            throw new UnsupportedOperationException("Unsupported target: " + target);
      }
   }

   @Override
   public SliderBoardParameters getSliderBoardParameters()
   {
      return new ValkyrieSliderBoardParameters();
   }

   @Override
   public FootstepPlannerParametersBasics getFootstepPlannerParameters()
   {
      return new ValkyrieFootstepPlannerParameters();
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
   public SplitFractionCalculatorParametersBasics getSplitFractionCalculatorParameters()
   {
      return new DefaultSplitFractionCalculatorParameters();
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
   public QuadTreeFootstepPlanningParameters getQuadTreeFootstepPlanningParameters()
   {
      return new ValkyrieFootstepPlanningParameters();
   }

   @Override
   public ICPWithTimeFreezingPlannerParameters getCapturePointPlannerParameters()
   {
      if (capturePointPlannerParameters == null)
         capturePointPlannerParameters = new ValkyrieSmoothCMPPlannerParameters(getRobotPhysicalProperties(), target);
      return capturePointPlannerParameters;
   }

   public void setWalkingControllerParameters(WalkingControllerParameters walkingControllerParameters)
   {
      this.walkingControllerParameters = walkingControllerParameters;
   }

   public void setContactPointParameters(RobotContactPointParameters<RobotSide> contactPointParameters)
   {
      this.contactPointParameters = contactPointParameters;
   }

   @Override
   public WalkingControllerParameters getWalkingControllerParameters()
   {
      if (walkingControllerParameters == null)
         walkingControllerParameters = new ValkyrieWalkingControllerParameters(getJointMap(), getRobotPhysicalProperties(), target);
      return walkingControllerParameters;
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
      ValkyrieSimulationCollisionModel collisionModel = new ValkyrieSimulationCollisionModel(getJointMap());
      collisionModel.setCollidableHelper(helper, robotCollisionMask, environmentCollisionMasks);
      return collisionModel;
   }

   @Override
   public UIParameters getUIParameters()
   {
      return new ValkyrieUIParameters(getRobotPhysicalProperties());
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
}
