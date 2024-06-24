package us.ihmc.atlas;

import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.avatar.scs2.SCS2AvatarSimulationFactory;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.controllers.OscillateFeetPerturber;

public class AtlasFlatGroundWalkingTrackSCS2
{
   private static final boolean USE_FEET_PERTURBER = true;
   private static final boolean USE_STAND_PREP = false;
   private static final boolean USE_IMPULSE_BASE_PHYSICS_ENGINE = false;

   private static boolean createYoVariableServer = System.getProperty("create.yovariable.server") != null
         && Boolean.parseBoolean(System.getProperty("create.yovariable.server"));

   private final RealtimeROS2Node realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(PubSubImplementation.INTRAPROCESS,
                                                                                      "flat_ground_walking_track_simulation");

   public AtlasFlatGroundWalkingTrackSCS2()
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS);
      FlatGroundEnvironment environment = new FlatGroundEnvironment();

      int recordFrequency = (int) Math.max(1.0, Math.round(robotModel.getControllerDT() / robotModel.getSimulateDT()));

      boolean useVelocityAndHeadingScript = true;
      HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters = new HeadingAndVelocityEvaluationScriptParameters();

      double initialYaw = 0.3;
      RobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, initialYaw);

      SCS2AvatarSimulationFactory avatarSimulationFactory = new SCS2AvatarSimulationFactory();
      avatarSimulationFactory.setRobotModel(robotModel);
      avatarSimulationFactory.setRealtimeROS2Node(realtimeROS2Node);
      if (USE_STAND_PREP)
         AtlasFlatGroundWalkingTrackSCS2Bullet.setupStandPrepController(robotModel, realtimeROS2Node, avatarSimulationFactory);
      else
         avatarSimulationFactory.setDefaultHighLevelHumanoidControllerFactory(useVelocityAndHeadingScript, walkingScriptParameters);
      avatarSimulationFactory.setCommonAvatarEnvrionmentInterface(environment);
      avatarSimulationFactory.setRobotInitialSetup(robotInitialSetup);
      avatarSimulationFactory.setSimulationDataRecordTickPeriod(recordFrequency);
      avatarSimulationFactory.setCreateYoVariableServer(createYoVariableServer);
      avatarSimulationFactory.setUseImpulseBasedPhysicsEngine(USE_IMPULSE_BASE_PHYSICS_ENGINE);

      SCS2AvatarSimulation avatarSimulation = avatarSimulationFactory.createAvatarSimulation();

      if (USE_FEET_PERTURBER)
         createOscillateFeetPerturber(avatarSimulation);

      avatarSimulation.start();
   }

   private static void createOscillateFeetPerturber(SCS2AvatarSimulation avatarSimulation)
   {
      Robot robot = avatarSimulation.getRobot();
      SimulationConstructionSet2 scs = avatarSimulation.getSimulationConstructionSet();
      SideDependentList<String> footNames = new SideDependentList<>(side -> avatarSimulation.getRobotModel().getJointMap().getFootName(side));

      int ticksPerPerturbation = 10;
      OscillateFeetPerturber oscillateFeetPerturber = new OscillateFeetPerturber(robot, footNames, scs.getDT() * ticksPerPerturbation);
      oscillateFeetPerturber.setTranslationMagnitude(new double[] {0.01, 0.015, 0.005});
      oscillateFeetPerturber.setRotationMagnitudeYawPitchRoll(new double[] {0.017, 0.012, 0.011});

      oscillateFeetPerturber.setTranslationFrequencyHz(RobotSide.LEFT, new double[] {0.0, 0, 3.3});
      oscillateFeetPerturber.setTranslationFrequencyHz(RobotSide.RIGHT, new double[] {0.0, 0, 1.3});

      oscillateFeetPerturber.setRotationFrequencyHzYawPitchRoll(RobotSide.LEFT, new double[] {0.0, 0, 7.3});
      oscillateFeetPerturber.setRotationFrequencyHzYawPitchRoll(RobotSide.RIGHT, new double[] {0., 0, 1.11});

      robot.addThrottledController(oscillateFeetPerturber, scs.getDT() * ticksPerPerturbation);
   }

   public static void main(String[] args)
   {
      new AtlasFlatGroundWalkingTrackSCS2();
   }
}
