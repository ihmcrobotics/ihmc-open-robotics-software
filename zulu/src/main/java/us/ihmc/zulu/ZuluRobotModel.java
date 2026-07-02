package us.ihmc.zulu;

import us.ihmc.avatar.AvatarSimulatedHandControlThread;
import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.HumanoidRobotInitialSetup;
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
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.zulu.parameters.controller.ZuluContactPointParameters;
import us.ihmc.zulu.parameters.controller.ZuluICPSplitFractionCalculatorParameters;
import us.ihmc.zulu.parameters.controller.ZuluHighLevelControllerParameters;
import us.ihmc.zulu.parameters.controller.ZuluStateEstimatorParameters;
import us.ihmc.zulu.parameters.controller.ZuluWalkingControllerParameters;
import us.ihmc.zulu.parameters.model.ZuluKSTKinematicsCollisionModel;
import us.ihmc.zulu.parameters.model.ZuluPhysicalProperties;
import us.ihmc.zulu.parameters.model.ZuluSimulationCollisionModel;
import us.ihmc.zulu.parameters.model.ZULUURDFParameters;
import us.ihmc.zulu.parameters.planning.ZuluFootstepPlannerParameters;
import us.ihmc.zulu.parameters.planning.ZuluLocomotionParameters;
import us.ihmc.zulu.parameters.planning.ZuluSwingPlannerParameters;
import us.ihmc.zulu.parameters.simulation.ZuluInitialSetup;
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

import java.io.InputStream;
import java.util.List;

public class ZuluRobotModel implements DRCRobotModel
{
   static final boolean ENFORCE_UNIQUE_REFERENCE_FRAMES = false;

   private static final double DEFAULT_SIMULATE_DT = 0.0001;
   private static final double DEFAULT_ESTIMATE_DT = 0.001;
   public static final double DEFAULT_CONTROL_DT = 0.003;
   private static final double DEFAULT_FEEDBACK_CONTROLLER_DT = 0.002;

   private double simulateDT = DEFAULT_SIMULATE_DT;
   private double estimatorDT = DEFAULT_ESTIMATE_DT;
   private double feedbackControllerDT = DEFAULT_FEEDBACK_CONTROLLER_DT;
   private double stepGeneratorDT = 10 * DEFAULT_CONTROL_DT;

   protected final ZuluPhysicalProperties physicalProperties;
   protected final WalkingControllerParameters walkingControllerParameters;
   private final ZuluHighLevelControllerParameters highLevelControllerParameters;
   private final ZuluSensorInformation sensorInformation;
   protected final ZuluJointMap jointMap;
   protected final RobotContactPointParameters<RobotSide> contactPointParameters;
   private final CoPTrajectoryParameters copTrajectoryParameters = new CoPTrajectoryParameters();
   private final StateEstimatorParameters stateEstimatorParameters;

   private final RobotDefinition scs1RobotDefinition;
   private final RobotDefinition controllerRobotDefinition;
   private final LogModelProvider logModelProvider;
   private final ZuluModelFactory modelFactory;

   protected final RobotTarget robotTarget;
   protected final ZuluVersion robotVersion;
   private final SideDependentList<HandModel> handModels = new SideDependentList<>();

   private final SideDependentList<RigidBodyTransform> handGraphicToHandFrameTransforms = new SideDependentList<>();

   public ZuluRobotModel(ZuluVersion robotVersion)
   {
      this(robotVersion, RobotTarget.SCS);
   }

   public ZuluRobotModel(ZuluVersion robotVersion, RobotTarget robotTarget)
   {
      this(robotVersion, robotTarget, null, true);
   }

   public ZuluRobotModel(ZuluVersion robotVersion,
                         RobotTarget robotTarget,
                         RobotContactPointParameters<RobotSide> contactPointParameters)
   {
      this(robotVersion, robotTarget, null, contactPointParameters);
   }

   public ZuluRobotModel(ZuluVersion robotVersion, RobotTarget robotTarget, MaterialDefinition robotMaterial)
   {
      this(robotVersion, robotTarget, robotMaterial, true);
   }

   public ZuluRobotModel(ZuluVersion robotVersion,
                         RobotTarget robotTarget,
                         MaterialDefinition robotMaterial,
                         boolean createHandContactPoints)
   {
      this(robotVersion,
           robotTarget,
           robotMaterial,
           new ZuluContactPointParameters(robotVersion.getJointMap(), robotVersion.getPhysicalProperties(), createHandContactPoints));
   }

   public ZuluRobotModel(ZuluVersion robotVersion,
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

      walkingControllerParameters = new ZuluWalkingControllerParameters(robotVersion,
                                                                        robotTarget,
                                                                        jointMap,
                                                                        physicalProperties,
                                                                        contactPointParameters);
      highLevelControllerParameters = new ZuluHighLevelControllerParameters(robotVersion, jointMap, robotTarget);
      stateEstimatorParameters = new ZuluStateEstimatorParameters(getEstimatorDT(), robotTarget, sensorInformation, jointMap);

      modelFactory = new ZuluModelFactory(robotVersion, jointMap, contactPointParameters, new ZuluRigidBodyMutator(getPhysicalProperties(), imusToIgnore));
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
   public ZuluVersion getRobotVersion()
   {
      return robotVersion;
   }

   public ZuluPhysicalProperties getPhysicalProperties()
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
      return ZULUURDFParameters.URDF_MODEL_NAME;
   }

   @Override
   public HumanoidRobotInitialSetup getDefaultRobotInitialSetup()
   {
      return new ZuluInitialSetup(getRobotVersion(), getRobotDefinition(), getJointMap());
   }

   @Override
   public double[] getPresetArmConfiguration(RobotSide side, PresetArmConfiguration presetArmConfiguration)
   {
      return ZuluPresetArmConfigurations.getPresetArmConfiguration(robotVersion, side, presetArmConfiguration);
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
   public double getFeedbackControllerDT()
   {
      return feedbackControllerDT;
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
      return "Zulu"; // TODO Should this just be robotName? Confusing which one to use
   }

   @Override
   public InputStream getWholeBodyControllerParametersFile()
   {
      return getClass().getResourceAsStream(getParameterResourceName());
   }

   public String getParameterResourceName()
   {
      return "/us/ihmc/zulu/parameters/controller.xml";
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
      ZuluSimulationCollisionModel collisionModel = new ZuluSimulationCollisionModel(jointMap);
      collisionModel.setCollidableHelper(helper, robotCollisionMask, environmentCollisionMasks);
      return collisionModel;
   }

   @Override
   public RobotCollisionModel getHumanoidRobotKinematicsCollisionModel()
   {
      return new ZuluKSTKinematicsCollisionModel(getJointMap());
   }

   @Override
   public AvatarSimulatedHandControlThread createSimulatedHandController(RealtimeROS2Node realtimeROS2Node, boolean kinematicsSimulation)
   {
      return null;
   }

   @Override
   public RobotLowLevelMessenger newRobotLowLevelMessenger(ROS2Node ros2Node)
   {
      return new ZuluDirectRobotInterface(ros2Node, this);
   }

   @Override
   public LocomotionParameters getLocomotionParameters()
   {
      return new ZuluLocomotionParameters();
   }

   @Override
   public DefaultFootstepPlannerParametersBasics getFootstepPlannerParameters()
   {
      return new ZuluFootstepPlannerParameters();
   }

   //@Override
   //public AStarBodyPathPlannerParametersBasics getAStarBodyPathPlannerParameters()
   //{
   //  return new AStarBodyPathPlannerParameters();
   //}

   @Override
   public DefaultFootstepPlannerParametersBasics getFootstepPlannerParameters(String fileNameSuffix)
   {
      return new ZuluFootstepPlannerParameters(fileNameSuffix);
   }

   @Override
   public AStarBodyPathPlannerParametersBasics getAStarBodyPathPlannerParameters()
   {
      return new AStarBodyPathPlannerParameters();
   }

   @Override
   public SwingPlannerParametersBasics getSwingPlannerParameters()
   {
      return new ZuluSwingPlannerParameters();
   }

   @Override
   public SwingPlannerParametersBasics getSwingPlannerParameters(String fileNameSuffix)
   {
      return new ZuluSwingPlannerParameters(fileNameSuffix);
   }

   @Override
   public SplitFractionCalculatorParametersReadOnly getSplitFractionCalculatorParameters()
   {
      return new ZuluICPSplitFractionCalculatorParameters();
   }

   @Override
   public double getStepGeneratorDT()
   {
      return stepGeneratorDT;
   }

   @Override
   public RigidBodyTransform getHandGraphicToHandFrameTransform(RobotSide side)
   {
      RigidBodyTransform handGraphicToHandTransform = new RigidBodyTransform();
      return handGraphicToHandTransform;
   }

   @Override
   public RigidBodyTransform getChestGraphicToFrameTransform()
   {
      RigidBodyTransform chestGraphicToFrameTransform = new RigidBodyTransform();
      chestGraphicToFrameTransform.getTranslation().addZ(0.1);

      return chestGraphicToFrameTransform;
   }

   public void setControllerDT(HighLevelControllerName controllerName, double controllerDT)
   {
      highLevelControllerParameters.setControlDT(controllerName, controllerDT);
   }

   public void setFeedbackControllerDT(double feedbackControllerDT)
   {
      this.feedbackControllerDT = feedbackControllerDT;
   }
}
