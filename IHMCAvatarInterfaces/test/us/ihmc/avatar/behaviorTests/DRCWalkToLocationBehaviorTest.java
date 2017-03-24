package us.ihmc.avatar.behaviorTests;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.Random;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationBehavior.WalkingOrientation;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.utilities.TrajectoryBasedStopThreadUpdatable;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModePacket.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionsettools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class DRCWalkToLocationBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

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
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.closeAndDispose();
         drcBehaviorTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @AfterClass
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCWalkToLocationBehaviorTest.class + " after class.");
   }

   private static final boolean DEBUG = false;

   // Atlas typically achieves between 0.02-0.03 position threshold
   private final double POSITION_THRESHOLD = 0.06;

   // Atlas typically achieves between .005-0.1 orientation threshold (more accurate when turning in place at final target)
   private final double ORIENTATION_THRESHOLD = 0.2;

   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   @Before
   public void setUp()
   {
      FlatGroundEnvironment testEnvironment = new FlatGroundEnvironment();
      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, getSimpleRobotName(), DRCObstacleCourseStartingLocation.DEFAULT,
            simulationTestingParameters, getRobotModel());
   }

   public void testTurn361DegreesInPlace() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      PrintTools.debug(this, "Initializing Sim");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      PrintTools.debug(this, "Initializing Behavior");
      FramePose2d desiredMidFeetPose2d = getCurrentMidFeetPose2dCopy();
      double currentYaw = desiredMidFeetPose2d.getYaw();
      desiredMidFeetPose2d.setYaw(currentYaw + Math.toRadians(361.0));
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose2d);
      PrintTools.debug(this, "Starting to Execute Behavior");
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(walkToLocationBehavior);
      assertTrue(success);
      PrintTools.debug(this, "Behavior Should be done");
      assertCurrentMidFeetPoseIsWithinThreshold(desiredMidFeetPose2d);
      assertTrue(walkToLocationBehavior.isDone());
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testWalkForwardsX() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      PrintTools.debug(this, "Initializing Sim");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      PrintTools.debug(this, "Initializing Behavior");
      double walkDistance = RandomNumbers.nextDouble(new Random(), 1.0, 2.0);
      Vector2D walkDirection = new Vector2D(1, 0);
      FramePose2d desiredMidFeetPose2d = copyAndOffsetCurrentMidfeetPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose2d);
      PrintTools.debug(this, "Starting to Execute Behavior");
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(walkToLocationBehavior);
      assertTrue(success);
      PrintTools.debug(this, "Behavior Should be done");
      assertCurrentMidFeetPoseIsWithinThreshold(desiredMidFeetPose2d);
      assertTrue(walkToLocationBehavior.isDone());
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testWalkBackwardsASmallAmountWithoutTurningInPlace() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      PrintTools.debug(this, "Initializing Sim");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      PrintTools.debug(this, "Initializing Behavior");
      double walkDistance = 2.0 * getRobotModel().getWalkingControllerParameters().getMinStepLengthForToeOff();
      Vector2D walkDirection = new Vector2D(-1, 0);
      FramePose2d desiredMidFeetPose2d = copyAndOffsetCurrentMidfeetPose2d(walkDistance, walkDirection);
      int randomZeroOrOne = RandomNumbers.nextInt(new Random(), 0, 1);
      double walkingOrientationRelativeToPathDirection;
      if (randomZeroOrOne == 0)
      {
         walkingOrientationRelativeToPathDirection = Math.PI;
      }
      else
      {
         walkingOrientationRelativeToPathDirection = -Math.PI;
      }

      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose2d, walkingOrientationRelativeToPathDirection);
      int numberOfFootsteps = walkToLocationBehavior.getNumberOfFootSteps();
      if (DEBUG)
         PrintTools.debug(this, "Number of Footsteps: " + numberOfFootsteps);
      assertTrue(numberOfFootsteps <= 4.0);
      PrintTools.debug(this, "Starting to Execute Behavior");
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(walkToLocationBehavior);
      assertTrue(success);
      PrintTools.debug(this, "Behavior Should be done");

      // TODO: Determine why position error is so large when walking backwards
      assertPosesAreWithinThresholds(desiredMidFeetPose2d, getCurrentMidFeetPose2dCopy(), 10.0 * POSITION_THRESHOLD);
      assertTrue(walkToLocationBehavior.isDone());
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testWalkAtAngleUsingStartOrientation() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      PrintTools.debug(this, "Initializing Sim");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      PrintTools.debug(this, "Initializing Behavior");
      int numberOfFootstepsBetweenStartAndTarget = 4;
      double walkDistance = numberOfFootstepsBetweenStartAndTarget * getRobotModel().getWalkingControllerParameters().getMaxStepLength();
      Vector2D walkDirection = new Vector2D(0.5, 0.5);
      FramePose2d startMidFeetPose2d = getCurrentMidFeetPose2dCopy();
      FramePose2d targetMidFeetPose2d = copyAndOffsetCurrentMidfeetPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createNewWalkToLocationBehavior();
      walkToLocationBehavior.initialize();
      walkToLocationBehavior.setTarget(targetMidFeetPose2d, WalkingOrientation.START_ORIENTATION);
      FramePose2d currentFootstepPose = new FramePose2d();
      for (Footstep footstep : walkToLocationBehavior.getFootSteps())
      {
         footstep.getPose2d(currentFootstepPose);
         assertEquals("Current footstep orientation does not match start orientation.", 0.0, currentFootstepPose.getOrientationDistance(startMidFeetPose2d),
               ORIENTATION_THRESHOLD);
      }

      PrintTools.debug(this, "Starting to Execute Behavior");
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(walkToLocationBehavior);
      assertTrue(success);
      PrintTools.debug(this, "Behavior Should be done");
      assertCurrentMidFeetPoseIsWithinThreshold(targetMidFeetPose2d);
      assertTrue(walkToLocationBehavior.isDone());
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testWalkAtAngleUsingTargetOrientation() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      PrintTools.debug(this, "Initializing Sim");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      PrintTools.debug(this, "Initializing Behavior");
      int numberOfFootstepsBetweenStartAndTarget = 4;
      double walkDistance = numberOfFootstepsBetweenStartAndTarget * getRobotModel().getWalkingControllerParameters().getMaxStepLength();
      Vector2D walkDirection = new Vector2D(-0.5, -0.5);
      double walkDirectionYaw = Math.atan2(walkDirection.getY(), walkDirection.getX());
      FramePose2d targetMidFeetPose2d = copyOffsetAndYawCurrentMidfeetPose2d(walkDistance, walkDirection, walkDirectionYaw);
      WalkToLocationBehavior walkToLocationBehavior = createNewWalkToLocationBehavior();
      walkToLocationBehavior.initialize();
      walkToLocationBehavior.setTarget(targetMidFeetPose2d, WalkingOrientation.TARGET_ORIENTATION);
      walkToLocationBehavior.doControl();

      ArrayList<Footstep> footsteps = walkToLocationBehavior.getFootSteps();
      int numberOfFootsteps = footsteps.size();
      FramePose2d currentFootstepPose = new FramePose2d();
      for (int numberOfStepsFromTarget = 0; numberOfStepsFromTarget <= numberOfFootstepsBetweenStartAndTarget; numberOfStepsFromTarget++)
      {
         footsteps.get(numberOfFootsteps - numberOfStepsFromTarget - 1).getPose2d(currentFootstepPose);
         assertEquals("Current footstep orientation does not match end orientation.", 0.0, currentFootstepPose.getOrientationDistance(targetMidFeetPose2d),
               ORIENTATION_THRESHOLD);
      }

      PrintTools.debug(this, "Starting to Execute Behavior");
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(walkToLocationBehavior);
      assertTrue(success);
      PrintTools.debug(this, "Behavior Should be done");
      assertCurrentMidFeetPoseIsWithinThreshold(targetMidFeetPose2d);
      assertTrue(walkToLocationBehavior.isDone());
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testWalkAtAngleUsingStartTargetMeanOrientation() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      PrintTools.debug(this, "Initializing Sim");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      PrintTools.debug(this, "Initializing Behavior");
      int numberOfFootstepsBetweenStartAndTarget = 4;
      double walkDistance = numberOfFootstepsBetweenStartAndTarget * getRobotModel().getWalkingControllerParameters().getMaxStepLength();
      Vector2D walkDirection = new Vector2D(0.5, 0.5);
      double walkDirectionYaw = Math.atan2(walkDirection.getY(), walkDirection.getX());
      FramePose2d startMidFeetPose2d = getCurrentMidFeetPose2dCopy();
      FramePose2d targetMidFeetPose2d = copyOffsetAndYawCurrentMidfeetPose2d(walkDistance, walkDirection, walkDirectionYaw);
      FramePose2d startTargetMidPose2dMean = new FramePose2d();
      startTargetMidPose2dMean.interpolate(startMidFeetPose2d, targetMidFeetPose2d, 0.5);
      WalkToLocationBehavior walkToLocationBehavior = createNewWalkToLocationBehavior();
      walkToLocationBehavior.initialize();
      walkToLocationBehavior.setTarget(targetMidFeetPose2d, WalkingOrientation.START_TARGET_ORIENTATION_MEAN);
      walkToLocationBehavior.doControl();

      ArrayList<Footstep> footsteps = walkToLocationBehavior.getFootSteps();
      int numberOfFootsteps = footsteps.size();
      FramePose2d currentFootstepPose = new FramePose2d();
      int numberOfStepsAlignedWithMeanOrientation = 0;
      for (Footstep footstep : footsteps)
      {
         footstep.getPose2d(currentFootstepPose);
         if (currentFootstepPose.getOrientationDistance(startTargetMidPose2dMean) < ORIENTATION_THRESHOLD)
            numberOfStepsAlignedWithMeanOrientation++;
      }

      PrintTools.info(this, "Total number of footsteps: " + numberOfFootsteps + ", number Of Footsteps aligned with mean orientation: "
            + numberOfStepsAlignedWithMeanOrientation);
      assertTrue("Number Of Footsteps aligned with mean orientation !> total number of footsteps",
            numberOfStepsAlignedWithMeanOrientation > 0.5 * numberOfFootsteps);
      PrintTools.debug(this, "Starting to Execute Behavior");
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(walkToLocationBehavior);
      assertTrue(success);
      PrintTools.debug(this, "Behavior Should be done");
      assertCurrentMidFeetPoseIsWithinThreshold(targetMidFeetPose2d);
      assertTrue(walkToLocationBehavior.isDone());
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testWalkAtAngleAndFinishAlignedWithWalkingPath() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      PrintTools.debug(this, "Initializing Sim");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      PrintTools.debug(this, "Initializing Behavior");
      double walkDistance = RandomNumbers.nextDouble(new Random(), 1.0, 2.0);
      double walkAngleDegrees = RandomNumbers.nextDouble(new Random(), 45.0);
      Vector2D walkDirection = new Vector2D(Math.cos(Math.toRadians(walkAngleDegrees)), Math.sin(Math.toRadians(walkAngleDegrees)));
      FramePose2d desiredMidFeetPose2d = copyOffsetAndYawCurrentMidfeetPose2d(walkDistance, walkDirection, Math.toRadians(walkAngleDegrees));
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose2d);
      PrintTools.debug(this, "Starting to Execute Behavior");
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(walkToLocationBehavior);
      assertTrue(success);
      PrintTools.debug(this, "Behavior Should be done");
      assertCurrentMidFeetPoseIsWithinThreshold(desiredMidFeetPose2d);
      assertTrue(walkToLocationBehavior.isDone());
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testWalkAtAngleAndFinishAlignedWithInitialOrientation() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      PrintTools.debug(this, "Initializing Sim");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      PrintTools.debug(this, "Initializing Behavior");
      double walkDistance = RandomNumbers.nextDouble(new Random(), 1.0, 2.0);
      double walkAngleDegrees = RandomNumbers.nextDouble(new Random(), 45.0);
      Vector2D walkDirection = new Vector2D(Math.cos(Math.toRadians(walkAngleDegrees)), Math.sin(Math.toRadians(walkAngleDegrees)));
      FramePose2d desiredMidFeetPose2d = copyAndOffsetCurrentMidfeetPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose2d);
      PrintTools.debug(this, "Starting to Execute Behavior");
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(walkToLocationBehavior);
      assertTrue(success);
      PrintTools.debug(this, "Behavior Should be done");
      assertCurrentMidFeetPoseIsWithinThreshold(desiredMidFeetPose2d);
      assertTrue(walkToLocationBehavior.isDone());
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testWalkAndStopBehavior() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      PrintTools.debug(this, "Initializing Sim");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      PrintTools.debug(this, "Initializing Behavior");
      double walkDistance = 4.0;
      Vector2D walkDirection = new Vector2D(1, 0);
      FramePose2d desiredMidFeetPose2d = copyAndOffsetCurrentMidfeetPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose2d);
      PrintTools.debug(this, "Starting to Execute Behavior");
      double pausePercent = Double.POSITIVE_INFINITY;
      double pauseDuration = Double.POSITIVE_INFINITY;
      double stopPercent = 20.0;
      ReferenceFrame frameToKeepTrackOf = drcBehaviorTestHelper.getReferenceFrames().getMidFeetZUpFrame();
      TrajectoryBasedStopThreadUpdatable stopThreadUpdatable = new TrajectoryBasedStopThreadUpdatable(drcBehaviorTestHelper.getRobotDataReceiver(),
            walkToLocationBehavior, pausePercent, pauseDuration, stopPercent, desiredMidFeetPose2d, frameToKeepTrackOf);
      success = drcBehaviorTestHelper.executeBehaviorPauseAndResumeOrStop(walkToLocationBehavior, stopThreadUpdatable);
      assertTrue(success);
      PrintTools.debug(this, "Stop Simulating Behavior");
      FramePose2d midFeetPose2dAtStop = stopThreadUpdatable.getTestFramePose2dAtTransition(BehaviorControlModeEnum.STOP);
      FramePose2d midFeetPose2dFinal = stopThreadUpdatable.getCurrentTestFramePose2dCopy();

      // Position and orientation may change after stop command if the robot is currently in single support,
      // since the robot will complete the current step (to get back into double support) before actually stopping
      double positionThreshold = getRobotModel().getWalkingControllerParameters().getMaxStepLength();
      double orientationThreshold = Math.PI;
      assertPosesAreWithinThresholds(midFeetPose2dAtStop, midFeetPose2dFinal, positionThreshold, orientationThreshold);
      assertTrue(!walkToLocationBehavior.isDone());
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testWalkPauseAndResumeBehavior() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      PrintTools.debug(this, "Initializing Sim");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      PrintTools.debug(this, "Initializing Behavior");
      double walkDistance = 3.0;
      Vector2D walkDirection = new Vector2D(1, 0);
      FramePose2d desiredMidFeetPose2d = copyAndOffsetCurrentMidfeetPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose2d);
      PrintTools.debug(this, "Starting to Execute Behavior");
      double pausePercent = 20.0;
      double pauseDuration = 2.0;
      double stopPercent = Double.POSITIVE_INFINITY;
      ReferenceFrame frameToKeepTrackOf = drcBehaviorTestHelper.getReferenceFrames().getMidFeetZUpFrame();
      TrajectoryBasedStopThreadUpdatable stopThreadUpdatable = new TrajectoryBasedStopThreadUpdatable(drcBehaviorTestHelper.getRobotDataReceiver(),
            walkToLocationBehavior, pausePercent, pauseDuration, stopPercent, desiredMidFeetPose2d, frameToKeepTrackOf);
      success = drcBehaviorTestHelper.executeBehaviorPauseAndResumeOrStop(walkToLocationBehavior, stopThreadUpdatable);
      assertTrue(success);
      PrintTools.debug(this, "Stop Simulating Behavior");
      FramePose2d midFeetPoseAtPause = stopThreadUpdatable.getTestFramePose2dAtTransition(BehaviorControlModeEnum.PAUSE);
      FramePose2d midFeetPoseAtResume = stopThreadUpdatable.getTestFramePose2dAtTransition(BehaviorControlModeEnum.RESUME);
      FramePose2d midFeetPoseFinal = stopThreadUpdatable.getCurrentTestFramePose2dCopy();

      // Position and orientation may change after pause command if the robot is currently in single support,
      // since the robot will complete the current step (to get back into double support) before actually pausing
      double positionThreshold = getRobotModel().getWalkingControllerParameters().getMaxStepLength();
      double orientationThreshold = Math.PI;
      assertPosesAreWithinThresholds(midFeetPoseAtPause, midFeetPoseAtResume, positionThreshold, orientationThreshold);
      assertTrue(walkToLocationBehavior.isDone());
      assertPosesAreWithinThresholds(desiredMidFeetPose2d, midFeetPoseFinal, POSITION_THRESHOLD, ORIENTATION_THRESHOLD);
      assertPosesAreWithinThresholds(desiredMidFeetPose2d, midFeetPoseFinal, 0.9 * POSITION_THRESHOLD, ORIENTATION_THRESHOLD);
      assertPosesAreWithinThresholds(desiredMidFeetPose2d, midFeetPoseFinal, POSITION_THRESHOLD, 0.9 * ORIENTATION_THRESHOLD);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testWalkPauseAndResumeOnLastStepBehavior() throws SimulationExceededMaximumTimeException
   {
      // This test makes sure that walking behavior doesn't declare isDone() when *starting/resuming* walking
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      PrintTools.debug(this, "Initializing Sim");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      PrintTools.debug(this, "Initializing Behavior");
      double walkDistance = 3.0;
      Vector2D walkDirection = new Vector2D(1, 0);
      FramePose2d desiredMidFeetPose2d = copyAndOffsetCurrentMidfeetPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose2d);
      PrintTools.debug(this, "Starting to Execute Behavior");
      double pausePercent = 80.0;
      double pauseDuration = 2.0;
      double stopPercent = Double.POSITIVE_INFINITY;
      ReferenceFrame frameToKeepTrackOf = drcBehaviorTestHelper.getReferenceFrames().getMidFeetZUpFrame();
      TrajectoryBasedStopThreadUpdatable stopThreadUpdatable = new TrajectoryBasedStopThreadUpdatable(drcBehaviorTestHelper.getRobotDataReceiver(),
            walkToLocationBehavior, pausePercent, pauseDuration, stopPercent, desiredMidFeetPose2d, frameToKeepTrackOf);
      success = drcBehaviorTestHelper.executeBehaviorPauseAndResumeOrStop(walkToLocationBehavior, stopThreadUpdatable);
      assertTrue(success);
      PrintTools.debug(this, "Stop Simulating Behavior");
      FramePose2d midFeetPoseAtPause = stopThreadUpdatable.getTestFramePose2dAtTransition(BehaviorControlModeEnum.PAUSE);
      FramePose2d midFeetPoseAtResume = stopThreadUpdatable.getTestFramePose2dAtTransition(BehaviorControlModeEnum.RESUME);
      FramePose2d midFeetPoseFinal = stopThreadUpdatable.getCurrentTestFramePose2dCopy();

      // Position and orientation may change after pause command if the robot is currently in single support,
      // since the robot will complete the current step (to get back into double support) before actually pausing
      double positionThreshold = getRobotModel().getWalkingControllerParameters().getMaxStepLength();
      double orientationThreshold = Math.PI;
      assertPosesAreWithinThresholds(midFeetPoseAtPause, midFeetPoseAtResume, positionThreshold, orientationThreshold);
      assertTrue(walkToLocationBehavior.isDone());
      assertPosesAreWithinThresholds(desiredMidFeetPose2d, midFeetPoseFinal);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testWalkStopAndWalkToDifferentLocation() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      PrintTools.debug(this, "Initializing Sim");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      PrintTools.debug(this, "Initializing Behavior");
      double walkDistance = 4.0;
      Vector2D walkDirection = new Vector2D(1.0, 0.0);
      FramePose2d desiredMidFeetPose2d = copyAndOffsetCurrentMidfeetPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose2d);
      double pausePercent = Double.POSITIVE_INFINITY;
      double pauseDuration = Double.POSITIVE_INFINITY;
      double stopPercent = 20.0;
      ReferenceFrame frameToKeepTrackOf = drcBehaviorTestHelper.getReferenceFrames().getMidFeetZUpFrame();
      TrajectoryBasedStopThreadUpdatable stopThreadUpdatable = new TrajectoryBasedStopThreadUpdatable(drcBehaviorTestHelper.getRobotDataReceiver(),
            walkToLocationBehavior, pausePercent, pauseDuration, stopPercent, desiredMidFeetPose2d, frameToKeepTrackOf);
      PrintTools.debug(this, "Starting to Execute Behavior");
      success = drcBehaviorTestHelper.executeBehaviorPauseAndResumeOrStop(walkToLocationBehavior, stopThreadUpdatable);
      assertTrue(success);
      PrintTools.debug(this, "Stop Simulating Behavior");
      FramePose2d midFeetPose2dAtStop = stopThreadUpdatable.getTestFramePose2dAtTransition(BehaviorControlModeEnum.STOP);
      FramePose2d midFeetPose2dFinal = stopThreadUpdatable.getCurrentTestFramePose2dCopy();

      // Position and orientation may change after stop command if the robot is currently in single support,
      // since the robot will complete the current step (to get back into double support) before actually stopping
      double positionThreshold = getRobotModel().getWalkingControllerParameters().getMaxStepLength();
      double orientationThreshold = Math.PI;
      assertPosesAreWithinThresholds(midFeetPose2dAtStop, midFeetPose2dFinal, positionThreshold, orientationThreshold);
      assertTrue(!walkToLocationBehavior.isDone());
      PrintTools.debug(this, "Setting New Behavior Inputs");
      walkDistance = 1.0;
      walkDirection.set(0.0, 1.0);
      double desiredYawAngle = Math.atan2(walkDirection.getY(), walkDirection.getX());
      FramePose2d newDesiredMidFeetPose2d = copyOffsetAndYawCurrentMidfeetPose2d(walkDistance, walkDirection, desiredYawAngle);
      walkToLocationBehavior.setTarget(newDesiredMidFeetPose2d);
      walkToLocationBehavior.resume();
      assertTrue(walkToLocationBehavior.hasInputBeenSet());
      PrintTools.debug(this, "Starting to Execute Behavior");
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(walkToLocationBehavior);
      assertTrue(success);
      PrintTools.debug(this, "Stop Simulating Behavior");
      assertCurrentMidFeetPoseIsWithinThreshold(newDesiredMidFeetPose2d);
      assertTrue(walkToLocationBehavior.isDone());
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private FramePose2d copyAndOffsetCurrentMidfeetPose2d(double walkDistance, Vector2D walkDirection)
   {
      FramePose2d desiredMidFeetPose = getCurrentMidFeetPose2dCopy();
      walkDirection.normalize();
      desiredMidFeetPose.setX(desiredMidFeetPose.getX() + walkDistance * walkDirection.getX());
      desiredMidFeetPose.setY(desiredMidFeetPose.getY() + walkDistance * walkDirection.getY());

      return desiredMidFeetPose;
   }

   private FramePose2d copyOffsetAndYawCurrentMidfeetPose2d(double walkDistance, Vector2D walkDirection, double desiredYawAngle)
   {
      FramePose2d desiredMidFeetPose = getCurrentMidFeetPose2dCopy();
      walkDirection.normalize();
      double xDesired = desiredMidFeetPose.getX() + walkDistance * walkDirection.getX();
      double yDesired = desiredMidFeetPose.getY() + walkDistance * walkDirection.getY();
      desiredMidFeetPose.setPoseIncludingFrame(ReferenceFrame.getWorldFrame(), xDesired, yDesired, desiredYawAngle);

      return desiredMidFeetPose;
   }

   private WalkToLocationBehavior createAndSetupWalkToLocationBehavior(FramePose2d desiredMidFeetPose)
   {
      return createAndSetupWalkToLocationBehavior(desiredMidFeetPose, 0.0);
   }

   private WalkToLocationBehavior createAndSetupWalkToLocationBehavior(FramePose2d desiredMidFeetPose, double walkingOrientationRelativeToPathDirection)
   {
      final WalkToLocationBehavior walkToLocationBehavior = createNewWalkToLocationBehavior();
      walkToLocationBehavior.initialize();
      walkToLocationBehavior.setWalkingOrientationRelativeToPathDirection(walkingOrientationRelativeToPathDirection);
      walkToLocationBehavior.setTarget(desiredMidFeetPose);
      assertTrue(walkToLocationBehavior.hasInputBeenSet());

      return walkToLocationBehavior;
   }

   private WalkToLocationBehavior createNewWalkToLocationBehavior()
   {
      CommunicationBridge communicationBridge = drcBehaviorTestHelper.getBehaviorCommunicationBridge();
      FullHumanoidRobotModel fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();
      HumanoidReferenceFrames referenceFrames = drcBehaviorTestHelper.getReferenceFrames();
      WalkingControllerParameters walkingControllerParams = getRobotModel().getWalkingControllerParameters();
      final WalkToLocationBehavior walkToLocationBehavior = new WalkToLocationBehavior(communicationBridge, fullRobotModel, referenceFrames,
            walkingControllerParams);

      return walkToLocationBehavior;
   }

   private FramePose2d getCurrentMidFeetPose2dCopy()
   {
      drcBehaviorTestHelper.updateRobotModel();
      ReferenceFrame midFeetFrame = drcBehaviorTestHelper.getReferenceFrames().getMidFeetZUpFrame();
      FramePose midFeetPose = new FramePose();
      midFeetPose.setToZero(midFeetFrame);
      midFeetPose.changeFrame(ReferenceFrame.getWorldFrame());
      FramePose2d ret = new FramePose2d();
      ret.setPoseIncludingFrame(midFeetPose.getReferenceFrame(), midFeetPose.getX(), midFeetPose.getY(), midFeetPose.getYaw());

      return ret;
   }

   private void assertCurrentMidFeetPoseIsWithinThreshold(FramePose2d desiredMidFeetPose)
   {
      FramePose2d currentMidFeetPose = getCurrentMidFeetPose2dCopy();
      assertPosesAreWithinThresholds(desiredMidFeetPose, currentMidFeetPose);
   }

   private void assertPosesAreWithinThresholds(FramePose2d desiredPose, FramePose2d actualPose)
   {
      assertPosesAreWithinThresholds(desiredPose, actualPose, POSITION_THRESHOLD);
   }

   private void assertPosesAreWithinThresholds(FramePose2d desiredPose, FramePose2d actualPose, double positionThreshold)
   {
      assertPosesAreWithinThresholds(desiredPose, actualPose, positionThreshold, ORIENTATION_THRESHOLD);
   }

   private void assertPosesAreWithinThresholds(FramePose2d desiredPose, FramePose2d actualPose, double positionThreshold, double orientationThreshold)
   {
      double positionDistance = desiredPose.getPositionDistance(actualPose);
      double orientationDistance = desiredPose.getOrientationDistance(actualPose);
      if (DEBUG)
      {
         PrintTools.debug(this, " desired Midfeet Pose :\n" + desiredPose + "\n");
         PrintTools.debug(this, " actual Midfeet Pose :\n" + actualPose + "\n");
         PrintTools.debug(this, " positionDistance = " + positionDistance);
         PrintTools.debug(this, " orientationDistance = " + orientationDistance);
      }

      assertEquals("Pose position error :" + positionDistance + " exceeds threshold: " + positionThreshold, 0.0, positionDistance, positionThreshold);
      assertEquals("Pose orientation error :" + orientationDistance + " exceeds threshold: " + orientationThreshold, 0.0, orientationDistance,
            orientationThreshold);
   }
}
