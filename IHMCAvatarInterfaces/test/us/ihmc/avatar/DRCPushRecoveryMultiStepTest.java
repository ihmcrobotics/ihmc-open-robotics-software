package us.ihmc.avatar;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class DRCPushRecoveryMultiStepTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private DRCFlatGroundWalkingTrack drcFlatGroundWalkingTrack;

   private static final boolean VISUALIZE_FORCE = false;

   private static final double PUSH_DELAY = 0.5;

   protected PushRobotController pushRobotController;

   protected BlockingSimulationRunner blockingSimulationRunner;

   protected double forceMagnitude;

   protected double forceDuration;

   protected SideDependentList<StateTransitionCondition> doubleSupportStartConditions = new SideDependentList<>();

   StateTransitionCondition pushCondition = doubleSupportStartConditions.get(RobotSide.LEFT);

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcFlatGroundWalkingTrack != null)
      {
         drcFlatGroundWalkingTrack.destroySimulation();
         drcFlatGroundWalkingTrack = null;
      }

      if (blockingSimulationRunner != null)
      {
         blockingSimulationRunner.destroySimulation();
         blockingSimulationRunner = null;
      }

      pushRobotController = null;
      doubleSupportStartConditions = null;
      pushCondition = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Ignore("Needs to be improved")
   @ContinuousIntegrationTest(estimatedDuration = 67.1)
   @Test(timeout = 340000)
   public void testMultiStepForwardAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      setupTest(getRobotModel());
      SimulationConstructionSet scs = drcFlatGroundWalkingTrack.getSimulationConstructionSet();
      setForwardPushParameters();
      Vector3d forceDirection = new Vector3d(1.0, 0.0, 0.0);
      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);
      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("ComponentBasedFootstepDataMessageGenerator", "walk");

      // disable walking
      walk.set(false);
      blockingSimulationRunner.simulateAndBlock(4.0);

      // push the robot
      pushRobotController.applyForceDelayed(pushCondition, PUSH_DELAY, forceDirection, forceMagnitude, forceDuration);

      // simulate for a little while longer
      blockingSimulationRunner.simulateAndBlock(forceDuration + 6.0);

      // re-enable walking
      walk.set(true);
      blockingSimulationRunner.simulateAndBlock(6.0);

      Point3d center = new Point3d(0.0, 0.0, 0.8882009563211146);
      Vector3d plusMinusVector = new Vector3d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      DRCSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox, drcFlatGroundWalkingTrack.getAvatarSimulation().getHumanoidFloatingRootJointRobot());

      createVideo(scs);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @ContinuousIntegrationTest(estimatedDuration = 53.2)
   @Test(timeout = 270000)
   public void testMultiStepBackwardAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      setupTest(getRobotModel());
      SimulationConstructionSet scs = drcFlatGroundWalkingTrack.getSimulationConstructionSet();
      setBackwardPushParameters();
      Vector3d forceDirection = new Vector3d(1.0, 0.0, 0.0);
      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);
      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("ComponentBasedFootstepDataMessageGenerator", "walk");

      // disable walking
      walk.set(false);
      blockingSimulationRunner.simulateAndBlock(1.0);

      // push the robot
      pushRobotController.applyForceDelayed(pushCondition, PUSH_DELAY, forceDirection, forceMagnitude, forceDuration);

      // simulate for a little while longer
      blockingSimulationRunner.simulateAndBlock(forceDuration + 5.0);

      // re-enable walking
      walk.set(true);
      blockingSimulationRunner.simulateAndBlock(6.0);

      Point3d center = new Point3d(0.0, 0.0, 0.8882009563211146);
      Vector3d plusMinusVector = new Vector3d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      DRCSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox, drcFlatGroundWalkingTrack.getAvatarSimulation().getHumanoidFloatingRootJointRobot());

      createVideo(scs);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   protected void setupTest(DRCRobotModel robotModel) throws SimulationExceededMaximumTimeException, InterruptedException
   {
      boolean runMultiThreaded = false;
      setupTrack(runMultiThreaded, robotModel);
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      pushRobotController = new PushRobotController(drcFlatGroundWalkingTrack.getAvatarSimulation().getHumanoidFloatingRootJointRobot(), fullRobotModel);

      if (VISUALIZE_FORCE)
      {
         drcFlatGroundWalkingTrack.getSimulationConstructionSet().addYoGraphic(pushRobotController.getForceVisualizer());
      }

      SimulationConstructionSet scs = drcFlatGroundWalkingTrack.getSimulationConstructionSet();
      BooleanYoVariable enable = (BooleanYoVariable) scs.getVariable("PushRecoveryControlModule", "enablePushRecovery");

      // enable push recovery
      enable.set(true);

      for (RobotSide robotSide : RobotSide.values)
      {
         String prefix = fullRobotModel.getFoot(robotSide).getName();
         scs.getVariable(prefix + "FootControlModule", prefix + "State");
         @SuppressWarnings("unchecked")
         final EnumYoVariable<WalkingStateEnum> walkingState = (EnumYoVariable<WalkingStateEnum>) scs.getVariable("WalkingHighLevelHumanoidController", "walkingState");
         doubleSupportStartConditions.put(robotSide, new DoubleSupportStartCondition(walkingState, robotSide));
      }
   }

   private void setupTrack(boolean runMultiThreaded, DRCRobotModel robotModel)
   {
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false, simulationTestingParameters);
      GroundProfile3D groundProfile = new FlatGroundProfile();
      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotModel.getSimulateDT());

      // scsInitialSetup.setInitializeEstimatorToActual(true);
      scsInitialSetup.setDrawGroundProfile(true);
      scsInitialSetup.setRunMultiThreaded(runMultiThreaded);
      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);
      drcFlatGroundWalkingTrack = new DRCFlatGroundWalkingTrack(robotInitialSetup, guiInitialSetup, scsInitialSetup, true, false, robotModel);
      drcFlatGroundWalkingTrack.getAvatarSimulation();
   }

   private class DoubleSupportStartCondition implements StateTransitionCondition
   {
      private final EnumYoVariable<WalkingStateEnum> walkingState;

      private final RobotSide side;

      public DoubleSupportStartCondition(EnumYoVariable<WalkingStateEnum> walkingState, RobotSide side)
      {
         this.walkingState = walkingState;
         this.side = side;
      }

      @Override
      public boolean checkCondition()
      {
         if (side == RobotSide.LEFT)
         {
            return (walkingState.getEnumValue() == WalkingStateEnum.TO_STANDING) || (walkingState.getEnumValue() == WalkingStateEnum.TO_WALKING_LEFT_SUPPORT);
         }
         else
         {
            return (walkingState.getEnumValue() == WalkingStateEnum.TO_STANDING) || (walkingState.getEnumValue() == WalkingStateEnum.TO_WALKING_RIGHT_SUPPORT);
         }
      }
   }


   private void createVideo(SimulationConstructionSet scs)
   {
      if (simulationTestingParameters.getCreateSCSVideos())
      {
         BambooTools.createVideoAndDataWithDateTimeClassMethodAndShareOnSharedDriveIfAvailable(getSimpleRobotName(), scs, 1);
      }
   }

   protected abstract void setForwardPushParameters();

   protected abstract void setBackwardPushParameters();
}
