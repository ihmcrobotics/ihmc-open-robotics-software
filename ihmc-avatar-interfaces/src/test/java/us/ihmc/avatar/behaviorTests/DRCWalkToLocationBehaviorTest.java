package us.ihmc.avatar.behaviorTests;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.avatar.testTools.scs2.SCS2BehaviorTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationBehavior.WalkingOrientation;
import us.ihmc.humanoidBehaviors.utilities.TrajectoryBasedStopThreadUpdatable;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

@Disabled
@Deprecated
public abstract class DRCWalkToLocationBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      // Do this here in case a test fails. That way the memory will be recycled.
      if (behaviorTestHelper != null)
      {
         behaviorTestHelper.finishTest();
         behaviorTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @AfterAll
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCWalkToLocationBehaviorTest.class + " after class.");
   }

   private static final boolean DEBUG = false;

   // Atlas typically achieves between 0.02-0.03 position threshold
   private final double POSITION_THRESHOLD = 0.06;

   // Atlas typically achieves between .005-0.1 orientation threshold (more accurate when turning in place at final target)
   private final double ORIENTATION_THRESHOLD = 0.2;

   private SCS2BehaviorTestHelper behaviorTestHelper;

   @BeforeEach
   public void setUp()
   {
      FlatGroundEnvironment testEnvironment = new FlatGroundEnvironment();
      SCS2AvatarTestingSimulation simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), testEnvironment, simulationTestingParameters);
      simulationTestHelper.start();
      behaviorTestHelper = new SCS2BehaviorTestHelper(simulationTestHelper);
   }

   @Test
   public void testTurn361DegreesInPlace()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      LogTools.debug("Initializing Sim");
      boolean success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);
      LogTools.debug("Initializing Behavior");
      FramePose2D desiredMidFeetPose2d = getCurrentMidFeetPose2dCopy();
      double currentYaw = desiredMidFeetPose2d.getYaw();
      desiredMidFeetPose2d.setYaw(currentYaw + Math.toRadians(361.0));
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose2d);
      LogTools.debug("Starting to Execute Behavior");
      success = behaviorTestHelper.executeBehaviorUntilDone(walkToLocationBehavior);
      assertTrue(success);
      LogTools.debug("Behavior Should be done");
      assertCurrentMidFeetPoseIsWithinThreshold(desiredMidFeetPose2d);
      assertTrue(walkToLocationBehavior.isDone());
      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkForwardsX()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      LogTools.debug("Initializing Sim");
      boolean success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);
      LogTools.debug("Initializing Behavior");
      double walkDistance = RandomNumbers.nextDouble(new Random(), 1.0, 2.0);
      Vector2D walkDirection = new Vector2D(1, 0);
      FramePose2D desiredMidFeetPose2d = copyAndOffsetCurrentMidfeetPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose2d);
      LogTools.debug("Starting to Execute Behavior");
      success = behaviorTestHelper.executeBehaviorUntilDone(walkToLocationBehavior);
      assertTrue(success);
      LogTools.debug("Behavior Should be done");
      assertCurrentMidFeetPoseIsWithinThreshold(desiredMidFeetPose2d);
      assertTrue(walkToLocationBehavior.isDone());
      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkBackwardsASmallAmountWithoutTurningInPlace()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      LogTools.debug("Initializing Sim");
      boolean success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);
      LogTools.debug("Initializing Behavior");
      double walkDistance = 2.0 * getRobotModel().getWalkingControllerParameters().getToeOffParameters().getMinStepLengthForToeOff();
      Vector2D walkDirection = new Vector2D(-1, 0);
      FramePose2D desiredMidFeetPose2d = copyAndOffsetCurrentMidfeetPose2d(walkDistance, walkDirection);
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
         LogTools.debug("Number of Footsteps: " + numberOfFootsteps);
      assertTrue(numberOfFootsteps <= 4.0);
      LogTools.debug("Starting to Execute Behavior");
      success = behaviorTestHelper.executeBehaviorUntilDone(walkToLocationBehavior);
      assertTrue(success);
      LogTools.debug("Behavior Should be done");

      // TODO: Determine why position error is so large when walking backwards
      assertPosesAreWithinThresholds(desiredMidFeetPose2d, getCurrentMidFeetPose2dCopy(), 10.0 * POSITION_THRESHOLD);
      assertTrue(walkToLocationBehavior.isDone());
      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkAtAngleUsingStartOrientation()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      LogTools.debug("Initializing Sim");
      boolean success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);
      LogTools.debug("Initializing Behavior");
      int numberOfFootstepsBetweenStartAndTarget = 4;
      double walkDistance = numberOfFootstepsBetweenStartAndTarget
            * getRobotModel().getWalkingControllerParameters().getSteppingParameters().getMaxStepLength();
      Vector2D walkDirection = new Vector2D(0.5, 0.5);
      FramePose2D startMidFeetPose2d = getCurrentMidFeetPose2dCopy();
      FramePose2D targetMidFeetPose2d = copyAndOffsetCurrentMidfeetPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createNewWalkToLocationBehavior();
      walkToLocationBehavior.initialize();
      walkToLocationBehavior.setTarget(targetMidFeetPose2d, WalkingOrientation.START_ORIENTATION);
      for (Footstep footstep : walkToLocationBehavior.getFootSteps())
      {
         FramePose2D currentFootstepPose = new FramePose2D(footstep.getFootstepPose());
         assertEquals("Current footstep orientation does not match start orientation.",
                      0.0,
                      currentFootstepPose.getOrientationDistance(startMidFeetPose2d),
                      ORIENTATION_THRESHOLD);
      }

      LogTools.debug("Starting to Execute Behavior");
      success = behaviorTestHelper.executeBehaviorUntilDone(walkToLocationBehavior);
      assertTrue(success);
      LogTools.debug("Behavior Should be done");
      assertCurrentMidFeetPoseIsWithinThreshold(targetMidFeetPose2d);
      assertTrue(walkToLocationBehavior.isDone());
      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkAtAngleUsingTargetOrientation()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      LogTools.debug("Initializing Sim");
      boolean success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);
      LogTools.debug("Initializing Behavior");
      int numberOfFootstepsBetweenStartAndTarget = 4;
      double walkDistance = numberOfFootstepsBetweenStartAndTarget
            * getRobotModel().getWalkingControllerParameters().getSteppingParameters().getMaxStepLength();
      Vector2D walkDirection = new Vector2D(-0.5, -0.5);
      double walkDirectionYaw = Math.atan2(walkDirection.getY(), walkDirection.getX());
      FramePose2D targetMidFeetPose2d = copyOffsetAndYawCurrentMidfeetPose2d(walkDistance, walkDirection, walkDirectionYaw);
      WalkToLocationBehavior walkToLocationBehavior = createNewWalkToLocationBehavior();
      walkToLocationBehavior.initialize();
      walkToLocationBehavior.setTarget(targetMidFeetPose2d, WalkingOrientation.TARGET_ORIENTATION);
      walkToLocationBehavior.doControl();

      ArrayList<Footstep> footsteps = walkToLocationBehavior.getFootSteps();
      int numberOfFootsteps = footsteps.size();
      for (int numberOfStepsFromTarget = 0; numberOfStepsFromTarget <= numberOfFootstepsBetweenStartAndTarget; numberOfStepsFromTarget++)
      {
         FramePose2D currentFootstepPose = new FramePose2D(footsteps.get(numberOfFootsteps - numberOfStepsFromTarget - 1).getFootstepPose());
         assertEquals("Current footstep orientation does not match end orientation.",
                      0.0,
                      currentFootstepPose.getOrientationDistance(targetMidFeetPose2d),
                      ORIENTATION_THRESHOLD);
      }

      LogTools.debug("Starting to Execute Behavior");
      success = behaviorTestHelper.executeBehaviorUntilDone(walkToLocationBehavior);
      assertTrue(success);
      LogTools.debug("Behavior Should be done");
      assertCurrentMidFeetPoseIsWithinThreshold(targetMidFeetPose2d);
      assertTrue(walkToLocationBehavior.isDone());
      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkAtAngleUsingStartTargetMeanOrientation()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      LogTools.debug("Initializing Sim");
      boolean success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);
      LogTools.debug("Initializing Behavior");
      int numberOfFootstepsBetweenStartAndTarget = 4;
      double walkDistance = numberOfFootstepsBetweenStartAndTarget
            * getRobotModel().getWalkingControllerParameters().getSteppingParameters().getMaxStepLength();
      Vector2D walkDirection = new Vector2D(0.5, 0.5);
      double walkDirectionYaw = Math.atan2(walkDirection.getY(), walkDirection.getX());
      FramePose2D startMidFeetPose2d = getCurrentMidFeetPose2dCopy();
      FramePose2D targetMidFeetPose2d = copyOffsetAndYawCurrentMidfeetPose2d(walkDistance, walkDirection, walkDirectionYaw);
      FramePose2D startTargetMidPose2dMean = new FramePose2D();
      startTargetMidPose2dMean.interpolate(startMidFeetPose2d, targetMidFeetPose2d, 0.5);
      WalkToLocationBehavior walkToLocationBehavior = createNewWalkToLocationBehavior();
      walkToLocationBehavior.initialize();
      walkToLocationBehavior.setTarget(targetMidFeetPose2d, WalkingOrientation.START_TARGET_ORIENTATION_MEAN);
      walkToLocationBehavior.doControl();

      ArrayList<Footstep> footsteps = walkToLocationBehavior.getFootSteps();
      int numberOfFootsteps = footsteps.size();
      int numberOfStepsAlignedWithMeanOrientation = 0;
      for (Footstep footstep : footsteps)
      {
         FramePose2D currentFootstepPose = new FramePose2D(footstep.getFootstepPose());
         if (currentFootstepPose.getOrientationDistance(startTargetMidPose2dMean) < ORIENTATION_THRESHOLD)
            numberOfStepsAlignedWithMeanOrientation++;
      }

      LogTools.info("Total number of footsteps: " + numberOfFootsteps + ", number Of Footsteps aligned with mean orientation: "
            + numberOfStepsAlignedWithMeanOrientation);
      assertTrue("Number Of Footsteps aligned with mean orientation !> total number of footsteps",
                 numberOfStepsAlignedWithMeanOrientation > 0.5 * numberOfFootsteps);
      LogTools.debug("Starting to Execute Behavior");
      success = behaviorTestHelper.executeBehaviorUntilDone(walkToLocationBehavior);
      assertTrue(success);
      LogTools.debug("Behavior Should be done");
      assertCurrentMidFeetPoseIsWithinThreshold(targetMidFeetPose2d);
      assertTrue(walkToLocationBehavior.isDone());
      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkAtAngleAndFinishAlignedWithWalkingPath()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      LogTools.debug("Initializing Sim");
      boolean success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);
      LogTools.debug("Initializing Behavior");
      double walkDistance = RandomNumbers.nextDouble(new Random(), 1.0, 2.0);
      double walkAngleDegrees = RandomNumbers.nextDouble(new Random(), 45.0);
      Vector2D walkDirection = new Vector2D(Math.cos(Math.toRadians(walkAngleDegrees)), Math.sin(Math.toRadians(walkAngleDegrees)));
      FramePose2D desiredMidFeetPose2d = copyOffsetAndYawCurrentMidfeetPose2d(walkDistance, walkDirection, Math.toRadians(walkAngleDegrees));
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose2d);
      LogTools.debug("Starting to Execute Behavior");
      success = behaviorTestHelper.executeBehaviorUntilDone(walkToLocationBehavior);
      assertTrue(success);
      LogTools.debug("Behavior Should be done");
      assertCurrentMidFeetPoseIsWithinThreshold(desiredMidFeetPose2d);
      assertTrue(walkToLocationBehavior.isDone());
      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkAtAngleAndFinishAlignedWithInitialOrientation()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      LogTools.debug("Initializing Sim");
      boolean success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);
      LogTools.debug("Initializing Behavior");
      double walkDistance = RandomNumbers.nextDouble(new Random(), 1.0, 2.0);
      double walkAngleDegrees = RandomNumbers.nextDouble(new Random(), 45.0);
      Vector2D walkDirection = new Vector2D(Math.cos(Math.toRadians(walkAngleDegrees)), Math.sin(Math.toRadians(walkAngleDegrees)));
      FramePose2D desiredMidFeetPose2d = copyAndOffsetCurrentMidfeetPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose2d);
      LogTools.debug("Starting to Execute Behavior");
      success = behaviorTestHelper.executeBehaviorUntilDone(walkToLocationBehavior);
      assertTrue(success);
      LogTools.debug("Behavior Should be done");
      assertCurrentMidFeetPoseIsWithinThreshold(desiredMidFeetPose2d);
      assertTrue(walkToLocationBehavior.isDone());
      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkAndStopBehavior()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      LogTools.debug("Initializing Sim");
      boolean success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);
      LogTools.debug("Initializing Behavior");
      double walkDistance = 4.0;
      Vector2D walkDirection = new Vector2D(1, 0);
      FramePose2D desiredMidFeetPose2d = copyAndOffsetCurrentMidfeetPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose2d);
      LogTools.debug("Starting to Execute Behavior");
      double pausePercent = Double.POSITIVE_INFINITY;
      double pauseDuration = Double.POSITIVE_INFINITY;
      double stopPercent = 20.0;
      ReferenceFrame frameToKeepTrackOf = behaviorTestHelper.getReferenceFrames().getMidFeetZUpFrame();
      TrajectoryBasedStopThreadUpdatable stopThreadUpdatable = new TrajectoryBasedStopThreadUpdatable(behaviorTestHelper.getRobotDataReceiver(),
                                                                                                      walkToLocationBehavior,
                                                                                                      pausePercent,
                                                                                                      pauseDuration,
                                                                                                      stopPercent,
                                                                                                      desiredMidFeetPose2d,
                                                                                                      frameToKeepTrackOf);
      success = behaviorTestHelper.executeBehaviorPauseAndResumeOrStop(walkToLocationBehavior, stopThreadUpdatable);
      assertTrue(success);
      LogTools.debug("Stop Simulating Behavior");
      FramePose2D midFeetPose2dAtStop = stopThreadUpdatable.getTestFramePose2dAtTransition(BehaviorControlModeEnum.STOP);
      FramePose2D midFeetPose2dFinal = stopThreadUpdatable.getCurrentTestFramePose2dCopy();

      // Position and orientation may change after stop command if the robot is currently in single support,
      // since the robot will complete the current step (to get back into double support) before actually stopping
      double positionThreshold = getRobotModel().getWalkingControllerParameters().getSteppingParameters().getMaxStepLength();
      double orientationThreshold = Math.PI;
      assertPosesAreWithinThresholds(midFeetPose2dAtStop, midFeetPose2dFinal, positionThreshold, orientationThreshold);
      assertTrue(!walkToLocationBehavior.isDone());
      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkPauseAndResumeBehavior()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      LogTools.debug("Initializing Sim");
      boolean success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);
      LogTools.debug("Initializing Behavior");
      double walkDistance = 3.0;
      Vector2D walkDirection = new Vector2D(1, 0);
      FramePose2D desiredMidFeetPose2d = copyAndOffsetCurrentMidfeetPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose2d);
      LogTools.debug("Starting to Execute Behavior");
      double pausePercent = 20.0;
      double pauseDuration = 2.0;
      double stopPercent = Double.POSITIVE_INFINITY;
      ReferenceFrame frameToKeepTrackOf = behaviorTestHelper.getReferenceFrames().getMidFeetZUpFrame();
      TrajectoryBasedStopThreadUpdatable stopThreadUpdatable = new TrajectoryBasedStopThreadUpdatable(behaviorTestHelper.getRobotDataReceiver(),
                                                                                                      walkToLocationBehavior,
                                                                                                      pausePercent,
                                                                                                      pauseDuration,
                                                                                                      stopPercent,
                                                                                                      desiredMidFeetPose2d,
                                                                                                      frameToKeepTrackOf);
      success = behaviorTestHelper.executeBehaviorPauseAndResumeOrStop(walkToLocationBehavior, stopThreadUpdatable);
      assertTrue(success);
      LogTools.debug("Stop Simulating Behavior");
      FramePose2D midFeetPoseAtPause = stopThreadUpdatable.getTestFramePose2dAtTransition(BehaviorControlModeEnum.PAUSE);
      FramePose2D midFeetPoseAtResume = stopThreadUpdatable.getTestFramePose2dAtTransition(BehaviorControlModeEnum.RESUME);
      FramePose2D midFeetPoseFinal = stopThreadUpdatable.getCurrentTestFramePose2dCopy();

      // Position and orientation may change after pause command if the robot is currently in single support,
      // since the robot will complete the current step (to get back into double support) before actually pausing
      double positionThreshold = getRobotModel().getWalkingControllerParameters().getSteppingParameters().getMaxStepLength();
      double orientationThreshold = Math.PI;
      assertPosesAreWithinThresholds(midFeetPoseAtPause, midFeetPoseAtResume, positionThreshold, orientationThreshold);
      assertTrue(walkToLocationBehavior.isDone());
      assertPosesAreWithinThresholds(desiredMidFeetPose2d, midFeetPoseFinal, POSITION_THRESHOLD, ORIENTATION_THRESHOLD);
      assertPosesAreWithinThresholds(desiredMidFeetPose2d, midFeetPoseFinal, 0.9 * POSITION_THRESHOLD, ORIENTATION_THRESHOLD);
      assertPosesAreWithinThresholds(desiredMidFeetPose2d, midFeetPoseFinal, POSITION_THRESHOLD, 0.9 * ORIENTATION_THRESHOLD);
      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkPauseAndResumeOnLastStepBehavior()
   {
      // This test makes sure that walking behavior doesn't declare isDone() when *starting/resuming* walking
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      LogTools.debug("Initializing Sim");
      boolean success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);
      LogTools.debug("Initializing Behavior");
      double walkDistance = 3.0;
      Vector2D walkDirection = new Vector2D(1, 0);
      FramePose2D desiredMidFeetPose2d = copyAndOffsetCurrentMidfeetPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose2d);
      LogTools.debug("Starting to Execute Behavior");
      double pausePercent = 80.0;
      double pauseDuration = 2.0;
      double stopPercent = Double.POSITIVE_INFINITY;
      ReferenceFrame frameToKeepTrackOf = behaviorTestHelper.getReferenceFrames().getMidFeetZUpFrame();
      TrajectoryBasedStopThreadUpdatable stopThreadUpdatable = new TrajectoryBasedStopThreadUpdatable(behaviorTestHelper.getRobotDataReceiver(),
                                                                                                      walkToLocationBehavior,
                                                                                                      pausePercent,
                                                                                                      pauseDuration,
                                                                                                      stopPercent,
                                                                                                      desiredMidFeetPose2d,
                                                                                                      frameToKeepTrackOf);
      success = behaviorTestHelper.executeBehaviorPauseAndResumeOrStop(walkToLocationBehavior, stopThreadUpdatable);
      assertTrue(success);
      LogTools.debug("Stop Simulating Behavior");
      FramePose2D midFeetPoseAtPause = stopThreadUpdatable.getTestFramePose2dAtTransition(BehaviorControlModeEnum.PAUSE);
      FramePose2D midFeetPoseAtResume = stopThreadUpdatable.getTestFramePose2dAtTransition(BehaviorControlModeEnum.RESUME);
      FramePose2D midFeetPoseFinal = stopThreadUpdatable.getCurrentTestFramePose2dCopy();

      // Position and orientation may change after pause command if the robot is currently in single support,
      // since the robot will complete the current step (to get back into double support) before actually pausing
      double positionThreshold = getRobotModel().getWalkingControllerParameters().getSteppingParameters().getMaxStepLength();
      double orientationThreshold = Math.PI;
      assertPosesAreWithinThresholds(midFeetPoseAtPause, midFeetPoseAtResume, positionThreshold, orientationThreshold);
      assertTrue(walkToLocationBehavior.isDone());
      assertPosesAreWithinThresholds(desiredMidFeetPose2d, midFeetPoseFinal);
      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkStopAndWalkToDifferentLocation()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      LogTools.debug("Initializing Sim");
      boolean success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);
      LogTools.debug("Initializing Behavior");
      double walkDistance = 4.0;
      Vector2D walkDirection = new Vector2D(1.0, 0.0);
      FramePose2D desiredMidFeetPose2d = copyAndOffsetCurrentMidfeetPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose2d);
      double pausePercent = Double.POSITIVE_INFINITY;
      double pauseDuration = Double.POSITIVE_INFINITY;
      double stopPercent = 20.0;
      ReferenceFrame frameToKeepTrackOf = behaviorTestHelper.getReferenceFrames().getMidFeetZUpFrame();
      TrajectoryBasedStopThreadUpdatable stopThreadUpdatable = new TrajectoryBasedStopThreadUpdatable(behaviorTestHelper.getRobotDataReceiver(),
                                                                                                      walkToLocationBehavior,
                                                                                                      pausePercent,
                                                                                                      pauseDuration,
                                                                                                      stopPercent,
                                                                                                      desiredMidFeetPose2d,
                                                                                                      frameToKeepTrackOf);
      LogTools.debug("Starting to Execute Behavior");
      success = behaviorTestHelper.executeBehaviorPauseAndResumeOrStop(walkToLocationBehavior, stopThreadUpdatable);
      assertTrue(success);
      LogTools.debug("Stop Simulating Behavior");
      FramePose2D midFeetPose2dAtStop = stopThreadUpdatable.getTestFramePose2dAtTransition(BehaviorControlModeEnum.STOP);
      FramePose2D midFeetPose2dFinal = stopThreadUpdatable.getCurrentTestFramePose2dCopy();

      // Position and orientation may change after stop command if the robot is currently in single support,
      // since the robot will complete the current step (to get back into double support) before actually stopping
      double positionThreshold = getRobotModel().getWalkingControllerParameters().getSteppingParameters().getMaxStepLength();
      double orientationThreshold = Math.PI;
      assertPosesAreWithinThresholds(midFeetPose2dAtStop, midFeetPose2dFinal, positionThreshold, orientationThreshold);
      assertTrue(!walkToLocationBehavior.isDone());
      LogTools.debug("Setting New Behavior Inputs");
      walkDistance = 1.0;
      walkDirection.set(0.0, 1.0);
      double desiredYawAngle = Math.atan2(walkDirection.getY(), walkDirection.getX());
      FramePose2D newDesiredMidFeetPose2d = copyOffsetAndYawCurrentMidfeetPose2d(walkDistance, walkDirection, desiredYawAngle);
      walkToLocationBehavior.setTarget(newDesiredMidFeetPose2d);
      walkToLocationBehavior.resume();
      assertTrue(walkToLocationBehavior.hasInputBeenSet());
      LogTools.debug("Starting to Execute Behavior");
      success = behaviorTestHelper.executeBehaviorUntilDone(walkToLocationBehavior);
      assertTrue(success);
      LogTools.debug("Stop Simulating Behavior");
      assertCurrentMidFeetPoseIsWithinThreshold(newDesiredMidFeetPose2d);
      assertTrue(walkToLocationBehavior.isDone());
      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private FramePose2D copyAndOffsetCurrentMidfeetPose2d(double walkDistance, Vector2D walkDirection)
   {
      FramePose2D desiredMidFeetPose = getCurrentMidFeetPose2dCopy();
      walkDirection.normalize();
      desiredMidFeetPose.setX(desiredMidFeetPose.getX() + walkDistance * walkDirection.getX());
      desiredMidFeetPose.setY(desiredMidFeetPose.getY() + walkDistance * walkDirection.getY());

      return desiredMidFeetPose;
   }

   private FramePose2D copyOffsetAndYawCurrentMidfeetPose2d(double walkDistance, Vector2D walkDirection, double desiredYawAngle)
   {
      FramePose2D desiredMidFeetPose = getCurrentMidFeetPose2dCopy();
      walkDirection.normalize();
      double xDesired = desiredMidFeetPose.getX() + walkDistance * walkDirection.getX();
      double yDesired = desiredMidFeetPose.getY() + walkDistance * walkDirection.getY();
      desiredMidFeetPose.setIncludingFrame(ReferenceFrame.getWorldFrame(), xDesired, yDesired, desiredYawAngle);

      return desiredMidFeetPose;
   }

   private WalkToLocationBehavior createAndSetupWalkToLocationBehavior(FramePose2D desiredMidFeetPose)
   {
      return createAndSetupWalkToLocationBehavior(desiredMidFeetPose, 0.0);
   }

   private WalkToLocationBehavior createAndSetupWalkToLocationBehavior(FramePose2D desiredMidFeetPose, double walkingOrientationRelativeToPathDirection)
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
      ROS2Node ros2Node = behaviorTestHelper.getROS2Node();
      FullHumanoidRobotModel fullRobotModel = behaviorTestHelper.getSDFFullRobotModel();
      HumanoidReferenceFrames referenceFrames = behaviorTestHelper.getReferenceFrames();
      WalkingControllerParameters walkingControllerParams = getRobotModel().getWalkingControllerParameters();
      final WalkToLocationBehavior walkToLocationBehavior = new WalkToLocationBehavior(behaviorTestHelper.getRobotName(),
                                                                                       ros2Node,
                                                                                       fullRobotModel,
                                                                                       referenceFrames,
                                                                                       walkingControllerParams);

      return walkToLocationBehavior;
   }

   private FramePose2D getCurrentMidFeetPose2dCopy()
   {
      behaviorTestHelper.updateRobotModel();
      ReferenceFrame midFeetFrame = behaviorTestHelper.getReferenceFrames().getMidFeetZUpFrame();
      FramePose3D midFeetPose = new FramePose3D();
      midFeetPose.setToZero(midFeetFrame);
      midFeetPose.changeFrame(ReferenceFrame.getWorldFrame());
      FramePose2D ret = new FramePose2D();
      ret.setIncludingFrame(midFeetPose.getReferenceFrame(), midFeetPose.getX(), midFeetPose.getY(), midFeetPose.getYaw());

      return ret;
   }

   private void assertCurrentMidFeetPoseIsWithinThreshold(FramePose2D desiredMidFeetPose)
   {
      FramePose2D currentMidFeetPose = getCurrentMidFeetPose2dCopy();
      assertPosesAreWithinThresholds(desiredMidFeetPose, currentMidFeetPose);
   }

   private void assertPosesAreWithinThresholds(FramePose2D desiredPose, FramePose2D actualPose)
   {
      assertPosesAreWithinThresholds(desiredPose, actualPose, POSITION_THRESHOLD);
   }

   private void assertPosesAreWithinThresholds(FramePose2D desiredPose, FramePose2D actualPose, double positionThreshold)
   {
      assertPosesAreWithinThresholds(desiredPose, actualPose, positionThreshold, ORIENTATION_THRESHOLD);
   }

   private void assertPosesAreWithinThresholds(FramePose2D desiredPose, FramePose2D actualPose, double positionThreshold, double orientationThreshold)
   {
      double positionDistance = desiredPose.getPositionDistance(actualPose);
      double orientationDistance = desiredPose.getOrientationDistance(actualPose);
      if (DEBUG)
      {
         LogTools.debug(" desired Midfeet Pose :\n" + desiredPose + "\n");
         LogTools.debug(" actual Midfeet Pose :\n" + actualPose + "\n");
         LogTools.debug(" positionDistance = " + positionDistance);
         LogTools.debug(" orientationDistance = " + orientationDistance);
      }

      assertEquals("Pose position error :" + positionDistance + " exceeds threshold: " + positionThreshold, 0.0, positionDistance, positionThreshold);
      assertEquals("Pose orientation error :" + orientationDistance + " exceeds threshold: " + orientationThreshold,
                   0.0,
                   orientationDistance,
                   orientationThreshold);
   }
}
