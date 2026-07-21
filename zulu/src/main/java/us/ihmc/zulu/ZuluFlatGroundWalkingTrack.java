package us.ihmc.zulu;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.avatar.scs2.SCS2AvatarSimulationFactory;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.jros2.AsyncROS2Node;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.yoVariables.variable.YoDouble;

public class ZuluFlatGroundWalkingTrack
{
   private static final boolean createYoVariableServer = Boolean.parseBoolean(System.getProperty("create.yovariable.server", "true"));
   private static final boolean kinematicsSimulation = Boolean.parseBoolean(System.getProperty("kinematics.simulation", "false"));

   private final AsyncROS2Node asyncROS2Node = new AsyncROS2Node("flat_ground_walking_track_simulation");

   public ZuluFlatGroundWalkingTrack()
   {
      DRCRobotModel robotModel = new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);
      FlatGroundEnvironment environment = new FlatGroundEnvironment();

      int recordFrequency = (int) Math.max(1.0, Math.round(robotModel.getFastestControllerDT() / robotModel.getSimulateDT()));

      boolean useVelocityAndHeadingScript = true;
      HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters = new HeadingAndVelocityEvaluationScriptParameters();

      double initialYaw = 0.0;
      RobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, initialYaw);

      SCS2AvatarSimulationFactory avatarSimulationFactory = new SCS2AvatarSimulationFactory();
      avatarSimulationFactory.setRobotModel(robotModel);
      avatarSimulationFactory.setAsyncROS2Node(asyncROS2Node);
      avatarSimulationFactory.setDefaultHighLevelHumanoidControllerFactory(useVelocityAndHeadingScript, walkingScriptParameters);
      avatarSimulationFactory.setCommonAvatarEnvrionmentInterface(environment);
      avatarSimulationFactory.setRobotInitialSetup(robotInitialSetup);
      avatarSimulationFactory.setSimulationDataRecordTickPeriod(recordFrequency);
      avatarSimulationFactory.setCreateYoVariableServer(createYoVariableServer);
      avatarSimulationFactory.setInitializeEstimatorToActual(true);
      avatarSimulationFactory.setUseImpulseBasedPhysicsEngine(false);
      avatarSimulationFactory.setUseBulletPhysicsEngine(false);
      avatarSimulationFactory.setUsePerfectSensors(true);
      if (kinematicsSimulation)
      {
         avatarSimulationFactory.setKinematicsSimulation(true);
         avatarSimulationFactory.setUsePerfectSensors(true);
      }

      SCS2AvatarSimulation avatarSimulation = avatarSimulationFactory.createAvatarSimulation();

      avatarSimulation.start();

      SimulationConstructionSet2 scs = avatarSimulation.getSimulationConstructionSet();
      ((YoDouble) scs.findVariable("transferTimeCSG")).set(0.4);
      ((YoDouble) scs.findVariable("swingTimeCSG")).set(0.8);
      //      ((YoDouble) scs.findVariable("swingHeight")).set(0.1);
      ((YoDouble) scs.findVariable("maxStepWidthCSG")).set(0.6);
      ((YoDouble) scs.findVariable("maxStepLengthForwardsCSG")).set(0.5);
   }

   public static void main(String[] args)
   {
      new ZuluFlatGroundWalkingTrack();
   }
}
