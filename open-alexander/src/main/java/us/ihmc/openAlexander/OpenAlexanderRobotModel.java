package us.ihmc.openAlexander;

import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import us.ihmc.avatar.AvatarSimulatedHandControlThread;
import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.HumanoidRobotInitialSetup;
import us.ihmc.avatar.kinematicsSimulation.SimulatedHandKinematicController;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation.DefaultSplitFractionCalculatorParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation.SplitFractionCalculatorParametersReadOnly;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.AStarBodyPathPlannerParameters;
import us.ihmc.footstepPlanning.AStarBodyPathPlannerParametersBasics;
import us.ihmc.footstepPlanning.LocomotionParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.handsros2.HandModel;
import us.ihmc.handsros2.abilityHand.AbilityHandModel;
import us.ihmc.handsros2.ezGripper.EZGripperModel;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.openAlexander.parameters.controller.ZuluContactPointParameters;
import us.ihmc.openAlexander.parameters.controller.ZuluHighLevelControllerParameters;
import us.ihmc.openAlexander.parameters.controller.ZuluStateEstimatorParameters;
import us.ihmc.openAlexander.parameters.controller.OpenAlexanderWalkingControllerParameters;
import us.ihmc.openAlexander.parameters.diagnostic.AlexanderDiagnosticParameters;
import us.ihmc.openAlexander.parameters.model.AlexanderKSTKinematicsCollisionModel;
import us.ihmc.openAlexander.parameters.model.AlexanderPhysicalProperties;
import us.ihmc.openAlexander.parameters.model.AlexanderSimulationCollisionModel;
import us.ihmc.openAlexander.parameters.model.OpenAlexanderURDFParameters;
import us.ihmc.openAlexander.parameters.planning.AlexanderFootstepPlannerParameters;
import us.ihmc.openAlexander.parameters.planning.AlexanderLocomotionParameters;
import us.ihmc.openAlexander.parameters.planning.AlexanderSwingPlannerParameters;
import us.ihmc.openAlexander.parameters.planning.AlexanderVisibilityGraphParameters;
import us.ihmc.openAlexander.parameters.simulation.AlexanderInitialSetup;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.perception.depthData.CollisionBoxProvider;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelWrapper;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.simulation.collision.CollidableHelper;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationToolkit.RobotDefinitionTools;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticParameters;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.io.InputStream;
import java.util.List;

public class OpenAlexanderRobotModel implements DRCRobotModel
{
   static final boolean ENFORCE_UNIQUE_REFERENCE_FRAMES = false;

   private static final double DEFAULT_SIMULATE_DT = 0.0002;
   private static final double DEFAULT_ESTIMATE_DT = 0.001;
   private static final double DEFAULT_CONTROL_DT = 0.003;
   private static final double DEFAULT_FEEDBACK_CONTROLLER_DT = 0.002;
   private static final double DEFAULT_PERCEPTION_DT = 0.003;
   private static final double ETHERCAT_DT = 0.001;

   private double simulateDT = DEFAULT_SIMULATE_DT;
   private double estimatorDT = DEFAULT_ESTIMATE_DT;
   private double controllerDT = DEFAULT_CONTROL_DT;
   private double feedbackControllerDT = DEFAULT_FEEDBACK_CONTROLLER_DT;
   private double perceptionDT = DEFAULT_PERCEPTION_DT;
   private double stepGeneratorDT = 10 * controllerDT;

   protected final AlexanderPhysicalProperties physicalProperties;
   protected final WalkingControllerParameters walkingControllerParameters;
   private final HighLevelControllerParameters highLevelControllerParameters;
   private final ZuluSensorInformation sensorInformation;
   protected final ZuluJointMap jointMap;
   protected final RobotContactPointParameters<RobotSide> contactPointParameters;
   private final CoPTrajectoryParameters copTrajectoryParameters = new CoPTrajectoryParameters();
   private final AlexanderDiagnosticParameters diagnosticParameters;
   private final StateEstimatorParameters stateEstimatorParameters;

   private final RobotDefinition scs1RobotDefinition;
   private final RobotDefinition controllerRobotDefinition;
   private final LogModelProvider logModelProvider;
   private final AlexanderModelFactory modelFactory;

   protected final RobotTarget robotTarget;
   protected final ZuluVersionInterface robotVersion;
   private final SideDependentList<HandModel> handModels = new SideDependentList<>();

   private final SideDependentList<RigidBodyTransform> handGraphicToHandFrameTransforms = new SideDependentList<>();

   public OpenAlexanderRobotModel(ZuluVersionInterface robotVersion)
   {
      this(robotVersion, RobotTarget.SCS);
   }

   public OpenAlexanderRobotModel(ZuluVersionInterface robotVersion, RobotTarget robotTarget)
   {
      this(robotVersion, robotTarget, null, true);
   }

   public OpenAlexanderRobotModel(ZuluVersionInterface robotVersion,
                                  RobotTarget robotTarget,
                                  RobotContactPointParameters<RobotSide> contactPointParameters)
   {
      this(robotVersion, robotTarget, null, contactPointParameters);
   }

   public OpenAlexanderRobotModel(ZuluVersionInterface robotVersion, RobotTarget robotTarget, MaterialDefinition robotMaterial)
   {
      this(robotVersion, robotTarget, robotMaterial, true);
   }

   public OpenAlexanderRobotModel(ZuluVersionInterface robotVersion,
                                  RobotTarget robotTarget,
                                  MaterialDefinition robotMaterial,
                                  boolean createHandContactPoints)
   {
      this(robotVersion,
           robotTarget,
           robotMaterial,
           new ZuluContactPointParameters(robotVersion.getJointMap(), robotVersion.getPhysicalProperties(), createHandContactPoints));
   }

   public OpenAlexanderRobotModel(ZuluVersionInterface robotVersion,
                                  RobotTarget robotTarget,
                                  MaterialDefinition robotMaterial,
                                  RobotContactPointParameters<RobotSide> contactPointParameters,
                                  String... imusToIgnore)
   {
      this.robotVersion = robotVersion;
      this.robotTarget = robotTarget;
      this.contactPointParameters = contactPointParameters;

      jointMap = robotVersion.getJointMap();
      sensorInformation = robotVersion.getSensorInformation();
      physicalProperties = robotVersion.getPhysicalProperties();

      walkingControllerParameters = new OpenAlexanderWalkingControllerParameters(robotVersion,
                                                                                 robotTarget,
                                                                                 jointMap,
                                                                                 physicalProperties,
                                                                                 contactPointParameters);
      highLevelControllerParameters = new ZuluHighLevelControllerParameters(robotVersion, jointMap, robotTarget);
      diagnosticParameters = new AlexanderDiagnosticParameters(robotTarget, jointMap, sensorInformation, highLevelControllerParameters);
      stateEstimatorParameters = new ZuluStateEstimatorParameters(getEstimatorDT(), robotTarget, sensorInformation, jointMap);

      modelFactory = new AlexanderModelFactory(robotVersion, jointMap, contactPointParameters, new AlexanderRigidBodyMutator(getPhysicalProperties(), imusToIgnore));
      logModelProvider = modelFactory.createLogModelProvider();
      scs1RobotDefinition = modelFactory.getSCS1RobotDefinition();
      controllerRobotDefinition = modelFactory.getControllerRobotDefinition();

      if (robotMaterial != null)
      {
         RobotDefinitionTools.setRobotDefinitionMaterial(scs1RobotDefinition, robotMaterial);
         RobotDefinitionTools.setRobotDefinitionMaterial(controllerRobotDefinition, robotMaterial);
      }

      for (RobotSide side : RobotSide.values)
      {
         if (robotVersion.hasHandWithFingers(side))
         {
            HandModel handModel = switch (robotVersion.getHandType(side))
            {
               case EZ_GRIPPER -> new EZGripperModel(true);
               case ABILITY_HAND -> new AbilityHandModel();
            };

            handModels.put(side, handModel);
         }
      }

      for (RobotSide side : RobotSide.values)
      {
         RigidBodyDefinition handBody = controllerRobotDefinition.getRigidBodyDefinition(jointMap.getHandName(side));
         if (handBody != null)
         {
            List<VisualDefinition> handVisualDefinitions = handBody.getVisualDefinitions();
            for (VisualDefinition visualDefinition : handVisualDefinitions)
            {
               if (visualDefinition.getGeometryDefinition() instanceof ModelFileGeometryDefinition)
               {
                  RigidBodyTransform handGraphicToHandFrameTransform = new RigidBodyTransform();
                  handGraphicToHandFrameTransform.set(visualDefinition.getOriginPose());
                  handGraphicToHandFrameTransforms.put(side, handGraphicToHandFrameTransform);
               }
            }
         }
      }
   }

   @Override
   public ZuluVersionInterface getRobotVersion()
   {
      return robotVersion;
   }

   public AlexanderPhysicalProperties getPhysicalProperties()
   {
      return physicalProperties;
   }

   @Override
   public RobotDefinition getRobotDefinition()
   {
      return controllerRobotDefinition;
   }

   @Override
   public HighLevelControllerParameters getHighLevelControllerParameters()
   {
      return highLevelControllerParameters;
   }

   @Override
   public WalkingControllerParameters getWalkingControllerParameters()
   {
      return walkingControllerParameters;
   }

   @Override
   public StateEstimatorParameters getStateEstimatorParameters()
   {
      return stateEstimatorParameters;
   }

   @Override
   public ZuluJointMap getJointMap()
   {
      return jointMap;
   }

   @Override
   public String toString()
   {
      return OpenAlexanderURDFParameters.URDF_MODEL_NAME;
   }

   @Override
   public HumanoidRobotInitialSetup getDefaultRobotInitialSetup()
   {
      return new AlexanderInitialSetup(getRobotVersion(), getRobotDefinition(), getJointMap());
   }

   @Override
   public double[] getPresetArmConfiguration(RobotSide side, PresetArmConfiguration presetArmConfiguration)
   {
      return AlexanderPresetArmConfigurations.getPresetArmConfiguration(robotVersion, side, presetArmConfiguration);
   }

   @Override
   public RobotContactPointParameters<RobotSide> getContactPointParameters()
   {
      return contactPointParameters;
   }

   @Override
   public HandModel getHandModel(RobotSide side)
   {
      return handModels.get(side);
   }

   @Override
   public ZuluSensorInformation getSensorInformation()
   {
      return sensorInformation;
   }

   @Override
   public FullHumanoidRobotModel createFullRobotModel()
   {
      return createFullRobotModel(ENFORCE_UNIQUE_REFERENCE_FRAMES);
   }

   @Override
   public FullHumanoidRobotModel createFullRobotModel(boolean enforceUniqueReferenceFrames)
   {
      return new FullHumanoidRobotModelWrapper(controllerRobotDefinition, jointMap, enforceUniqueReferenceFrames);
   }

   @Override
   public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes, boolean enableJointDamping)
   {
      boolean enableTorqueVelocityLimits = false;
      return new HumanoidFloatingRootJointRobot(scs1RobotDefinition, jointMap, enableJointDamping, enableTorqueVelocityLimits);
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
   public double getFeedbackControllerDT()
   {
      return feedbackControllerDT;
   }

   @Override
   public DRCSensorSuiteManager getSensorSuiteManager()
   {
      return null;
   }

   @Override
   public DRCSensorSuiteManager getSensorSuiteManager(ROS2Node ros2Node)
   {
      return null;
   }

   @Override
   public LogModelProvider getLogModelProvider()
   {
      return logModelProvider;
   }

   @Override
   public DataServerSettings getLogSettings()
   {
      return new DataServerSettings(true);
   }

   @Override
   public String getSimpleRobotName()
   {
      return "Alexander"; // TODO Should this just be robotName? Confusing which one to use
   }

   @Override
   public CollisionBoxProvider getCollisionBoxProvider()
   {
      return null;
   }

   @Override
   public InputStream getWholeBodyControllerParametersFile()
   {
      return getClass().getResourceAsStream(getParameterResourceName());
   }

   public String getParameterResourceName()
   {
      return "/us/ihmc/openAlexander/parameters/controller.xml";
   }

   @Override
   public String getParameterFileName()
   {
      return getParameterResourceName();
   }

   @Override
   public CoPTrajectoryParameters getCoPTrajectoryParameters()
   {
      return copTrajectoryParameters;
   }

   @Override
   public RobotCollisionModel getSimulationRobotCollisionModel(CollidableHelper helper, String robotCollisionMask, String... environmentCollisionMasks)
   {
      AlexanderSimulationCollisionModel collisionModel = new AlexanderSimulationCollisionModel(jointMap, physicalProperties);
      collisionModel.setCollidableHelper(helper, robotCollisionMask, environmentCollisionMasks);
      return collisionModel;
   }

   @Override
   public RobotCollisionModel getHumanoidRobotKinematicsCollisionModel()
   {
      return new AlexanderKSTKinematicsCollisionModel(getJointMap());
   }

   @Override
   public AvatarSimulatedHandControlThread createSimulatedHandController(RealtimeROS2Node realtimeROS2Node, boolean kinematicsSimulation)
   {
      return null;
   }

   @Override
   public SimulatedHandKinematicController createSimulatedHandKinematicController(FullHumanoidRobotModel fullHumanoidRobotModel,
                                                                                  RealtimeROS2Node realtimeROS2Node,
                                                                                  DoubleProvider controllerTime)
   {
      return null;
   }

   @Override
   public DiagnosticParameters getDiagnoticParameters()
   {
      return diagnosticParameters;
   }

   @Override
   public RobotLowLevelMessenger newRobotLowLevelMessenger(ROS2Node ros2Node)
   {
      return new AlexanderDirectRobotInterface(ros2Node, this);
   }

   @Override
   public LocomotionParameters getLocomotionParameters()
   {
      return new AlexanderLocomotionParameters();
   }

   @Override
   public DefaultFootstepPlannerParametersBasics getFootstepPlannerParameters()
   {
      return new AlexanderFootstepPlannerParameters();
   }

   //@Override
   //public AStarBodyPathPlannerParametersBasics getAStarBodyPathPlannerParameters()
   //{
   //  return new AStarBodyPathPlannerParameters();
   //}

   @Override
   public DefaultFootstepPlannerParametersBasics getFootstepPlannerParameters(String fileNameSuffix)
   {
      return new AlexanderFootstepPlannerParameters(fileNameSuffix);
   }

   @Override
   public AStarBodyPathPlannerParametersBasics getAStarBodyPathPlannerParameters()
   {
      return new AStarBodyPathPlannerParameters();
   }

   @Override
   public VisibilityGraphsParametersBasics getVisibilityGraphsParameters()
   {
      return new AlexanderVisibilityGraphParameters();
   }

   @Override
   public SwingPlannerParametersBasics getSwingPlannerParameters()
   {
      return new AlexanderSwingPlannerParameters();
   }

   @Override
   public SwingPlannerParametersBasics getSwingPlannerParameters(String fileNameSuffix)
   {
      return new AlexanderSwingPlannerParameters(fileNameSuffix);
   }

   @Override
   public SplitFractionCalculatorParametersReadOnly getSplitFractionCalculatorParameters()
   {
      return new DefaultSplitFractionCalculatorParameters();
   }

   @Override
   public double getStepGeneratorDT()
   {
      return stepGeneratorDT;
   }

   @Override
   public Transform getJmeTransformWristToHand(RobotSide robotSide)
   {
      Vector3f centerOfHandToWristTranslation = new Vector3f();
      float[] angles = new float[3];

      centerOfHandToWristTranslation = new Vector3f(0f, robotSide.negateIfLeftSide(0.015f), -0.06f);
      angles[0] = (float) robotSide.negateIfLeftSide(Math.toRadians(90));
      angles[1] = 0.0f;
      angles[2] = (float) robotSide.negateIfLeftSide(Math.toRadians(90));

      Quaternion centerOfHandToWristRotation = new Quaternion(angles);
      return new Transform(centerOfHandToWristTranslation, centerOfHandToWristRotation);
   }

   @Override
   public RigidBodyTransform getHandGraphicToHandFrameTransform(RobotSide side)
   {
      RigidBodyTransform handGraphicToHandTransform = new RigidBodyTransform();
      if (!robotVersion.hasNubHands(side))
         handGraphicToHandTransform.getRotation().setYawPitchRoll(0.0, Math.PI / 2.0, 0.0);

      return handGraphicToHandTransform;
   }

   @Override
   public RigidBodyTransform getChestGraphicToFrameTransform()
   {
      RigidBodyTransform chestGraphicToFrameTransform = new RigidBodyTransform();
      chestGraphicToFrameTransform.getTranslation().addZ(0.1);

      return chestGraphicToFrameTransform;
   }

   public void setControllerDT(double controllerDT)
   {
      this.controllerDT = controllerDT;
   }

   public void setFeedbackControllerDT(double feedbackControllerDT)
   {
      this.feedbackControllerDT = feedbackControllerDT;
   }
}
