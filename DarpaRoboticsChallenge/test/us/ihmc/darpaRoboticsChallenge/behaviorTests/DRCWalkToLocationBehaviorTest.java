package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Vector2d;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoPacketCommunicator;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorControlModePacket.HumanoidBehaviorControlModeEnum;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.utilities.TrajectoryBasedStopThreadUpdatable;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.AsyncContinuousExecutor;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.TimerTaskScheduler;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.io.printing.SysoutTool;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FramePose2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.time.GlobalTimer;

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

      GlobalTimer.clearTimers();
      TimerTaskScheduler.cancelAndReset();
      AsyncContinuousExecutor.cancelAndReset();

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @AfterClass
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCWalkToLocationBehaviorTest.class + " after class.");
   }

   private static final boolean DEBUG = false;

   private final double POSITION_THRESHOLD = 0.1;
   private final double ORIENTATION_THRESHOLD = 0.1;

   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   @Before
   public void setUp()
   {
      if (NetworkConfigParameters.USE_BEHAVIORS_MODULE)
      {
         throw new RuntimeException("Must set NetworkConfigParameters.USE_BEHAVIORS_MODULE = false in order to perform this test!");
      }

      DRCDemo01NavigationEnvironment testEnvironment = new DRCDemo01NavigationEnvironment();

      KryoPacketCommunicator controllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10, "DRCControllerCommunicator");
      KryoPacketCommunicator networkObjectCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10, "DRCJunkyCommunicator");

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, networkObjectCommunicator, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, false, getRobotModel(), controllerCommunicator);
   }

   @AverageDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testWalkForwardsX() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      SysoutTool.println("Initializing Sim", DEBUG);
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      SysoutTool.println("Initializing Behavior", DEBUG);
      double walkDistance = RandomTools.generateRandomDouble(new Random(), 1.0, 2.0);
      Vector2d walkDirection = new Vector2d(1, 0);
      FramePose2d desiredMidFeetPose2d = createDesiredPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose2d);

      SysoutTool.println("Starting to Execute Behavior", DEBUG);
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(walkToLocationBehavior);
      assertTrue(success);
      SysoutTool.println("Behavior Should be done", DEBUG);

      assertCurrentMidFeetPoseIsWithinThreshold(desiredMidFeetPose2d);
      assertTrue(walkToLocationBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   @AverageDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testWalkAndStopBehavior() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      SysoutTool.println("Initializing Sim", DEBUG);
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      SysoutTool.println("Initializing Behavior", DEBUG);
      double walkDistance = 4.0;
      Vector2d walkDirection = new Vector2d(1, 0);
      FramePose2d desiredMidFeetPose2d = createDesiredPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose2d);

      SysoutTool.println("Starting to Execute Behavior", DEBUG);
      double pausePercent = Double.POSITIVE_INFINITY;
      double pauseDuration = Double.POSITIVE_INFINITY;
      double stopPercent = 20.0;

      ReferenceFrame frameToKeepTrackOf = drcBehaviorTestHelper.getReferenceFrames().getMidFeetZUpFrame();

      TrajectoryBasedStopThreadUpdatable stopThreadUpdatable = new TrajectoryBasedStopThreadUpdatable(drcBehaviorTestHelper.getRobotDataReceiver(),
            walkToLocationBehavior, pausePercent, pauseDuration, stopPercent, desiredMidFeetPose2d, frameToKeepTrackOf);

      success = drcBehaviorTestHelper.executeBehaviorPauseAndResumeOrStop(walkToLocationBehavior, stopThreadUpdatable);
      assertTrue(success);
      SysoutTool.println("Stop Simulating Behavior", DEBUG);

      FramePose2d midFeetPose2dAtStop = stopThreadUpdatable.getTestFramePose2dAtTransition(HumanoidBehaviorControlModeEnum.STOP);
      FramePose2d midFeetPose2dFinal = stopThreadUpdatable.getCurrentTestFramePose2dCopy();

      // Position and orientation may change after stop command if the robot is currently in single support, 
      // since the robot will complete the current step (to get back into double support) before actually stopping
      double positionThreshold = getRobotModel().getWalkingControllerParameters().getMaxStepLength();
      double orientationThreshold = Math.PI;
      assertPosesAreWithinThresholds(midFeetPose2dAtStop, midFeetPose2dFinal, positionThreshold, orientationThreshold);
      assertTrue(!walkToLocationBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   @AverageDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testWalkPauseAndResumeBehavior() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      SysoutTool.println("Initializing Sim", DEBUG);
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      SysoutTool.println("Initializing Behavior", DEBUG);
      double walkDistance = 3.0;
      Vector2d walkDirection = new Vector2d(1, 0);
      FramePose2d desiredMidFeetPose2d = createDesiredPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose2d);

      SysoutTool.println("Starting to Execute Behavior", DEBUG);
      double pausePercent = 20.0;
      double pauseDuration = 2.0;
      double stopPercent = Double.POSITIVE_INFINITY;

      ReferenceFrame frameToKeepTrackOf = drcBehaviorTestHelper.getReferenceFrames().getMidFeetZUpFrame();

      TrajectoryBasedStopThreadUpdatable stopThreadUpdatable = new TrajectoryBasedStopThreadUpdatable(drcBehaviorTestHelper.getRobotDataReceiver(),
            walkToLocationBehavior, pausePercent, pauseDuration, stopPercent, desiredMidFeetPose2d, frameToKeepTrackOf);

      success = drcBehaviorTestHelper.executeBehaviorPauseAndResumeOrStop(walkToLocationBehavior, stopThreadUpdatable);
      assertTrue(success);
      SysoutTool.println("Stop Simulating Behavior", DEBUG);

      FramePose2d midFeetPoseAtPause = stopThreadUpdatable.getTestFramePose2dAtTransition(HumanoidBehaviorControlModeEnum.PAUSE);
      FramePose2d midFeetPoseAtResume = stopThreadUpdatable.getTestFramePose2dAtTransition(HumanoidBehaviorControlModeEnum.RESUME);
      FramePose2d midFeetPoseFinal = stopThreadUpdatable.getCurrentTestFramePose2dCopy();

      // Position and orientation may change after pause command if the robot is currently in single support, 
      // since the robot will complete the current step (to get back into double support) before actually pausing
      double positionThreshold = getRobotModel().getWalkingControllerParameters().getMaxStepLength();
      double orientationThreshold = Math.PI;
      assertPosesAreWithinThresholds(midFeetPoseAtPause, midFeetPoseAtResume, positionThreshold, orientationThreshold);
      assertTrue(walkToLocationBehavior.isDone());
      assertPosesAreWithinThresholds(desiredMidFeetPose2d, midFeetPoseFinal);

      BambooTools.reportTestFinishedMessage();
   }

   @AverageDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testWalkPauseAndResumeOnLastStepBehavior() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      SysoutTool.println("Initializing Sim", DEBUG);
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      SysoutTool.println("Initializing Behavior", DEBUG);
      double walkDistance = 3.0;
      Vector2d walkDirection = new Vector2d(1, 0);
      FramePose2d desiredMidFeetPose2d = createDesiredPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose2d);

      SysoutTool.println("Starting to Execute Behavior", DEBUG);
      double pausePercent = 80.0;
      double pauseDuration = 2.0;
      double stopPercent = Double.POSITIVE_INFINITY;

      ReferenceFrame frameToKeepTrackOf = drcBehaviorTestHelper.getReferenceFrames().getMidFeetZUpFrame();

      TrajectoryBasedStopThreadUpdatable stopThreadUpdatable = new TrajectoryBasedStopThreadUpdatable(drcBehaviorTestHelper.getRobotDataReceiver(),
            walkToLocationBehavior, pausePercent, pauseDuration, stopPercent, desiredMidFeetPose2d, frameToKeepTrackOf);

      success = drcBehaviorTestHelper.executeBehaviorPauseAndResumeOrStop(walkToLocationBehavior, stopThreadUpdatable);
      assertTrue(success);
      SysoutTool.println("Stop Simulating Behavior", DEBUG);

      FramePose2d midFeetPoseAtPause = stopThreadUpdatable.getTestFramePose2dAtTransition(HumanoidBehaviorControlModeEnum.PAUSE);
      FramePose2d midFeetPoseAtResume = stopThreadUpdatable.getTestFramePose2dAtTransition(HumanoidBehaviorControlModeEnum.RESUME);
      FramePose2d midFeetPoseFinal = stopThreadUpdatable.getCurrentTestFramePose2dCopy();

      // Position and orientation may change after pause command if the robot is currently in single support, 
      // since the robot will complete the current step (to get back into double support) before actually pausing
      double positionThreshold = getRobotModel().getWalkingControllerParameters().getMaxStepLength();
      double orientationThreshold = Math.PI;
      assertPosesAreWithinThresholds(midFeetPoseAtPause, midFeetPoseAtResume, positionThreshold, orientationThreshold);
      assertTrue(walkToLocationBehavior.isDone());
      assertPosesAreWithinThresholds(desiredMidFeetPose2d, midFeetPoseFinal);

      BambooTools.reportTestFinishedMessage();
   }

   private FramePose2d createDesiredPose2d(double walkDistance, Vector2d walkDirection)
   {
      FramePose2d initialMidFeetPose = getCurrentMidFeetPose2d();
      SysoutTool.println(" initial Midfeet Pose :\n" + initialMidFeetPose + "\n", DEBUG);

      FramePose2d desiredMidFeetPose = new FramePose2d(initialMidFeetPose);

      walkDirection.normalize();
      desiredMidFeetPose.setX(initialMidFeetPose.getX() + walkDistance * walkDirection.getX());
      desiredMidFeetPose.setY(initialMidFeetPose.getY() + walkDistance * walkDirection.getY());

      return desiredMidFeetPose;
   }

   private WalkToLocationBehavior createAndSetupWalkToLocationBehavior(FramePose2d desiredMidFeetPose)
   {
      BehaviorCommunicationBridge communicationBridge = drcBehaviorTestHelper.getBehaviorCommunicationBridge();
      FullRobotModel fullRobotModel = drcBehaviorTestHelper.getFullRobotModel();
      ReferenceFrames referenceFrames = drcBehaviorTestHelper.getReferenceFrames();
      WalkingControllerParameters walkingControllerParams = getRobotModel().getWalkingControllerParameters();

      final WalkToLocationBehavior walkToLocationBehavior = new WalkToLocationBehavior(communicationBridge, fullRobotModel, referenceFrames,
            walkingControllerParams);

      walkToLocationBehavior.initialize();
      walkToLocationBehavior.setTarget(desiredMidFeetPose);
      assertTrue(walkToLocationBehavior.hasInputBeenSet());

      return walkToLocationBehavior;
   }

   private FramePose2d getCurrentMidFeetPose2d()
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
      FramePose2d currentMidFeetPose = getCurrentMidFeetPose2d();
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
         SysoutTool.println(" desired Midfeet Pose :\n" + desiredPose + "\n");
         SysoutTool.println(" actual Midfeet Pose :\n" + actualPose + "\n");

         SysoutTool.println(" positionDistance = " + positionDistance);
         SysoutTool.println(" orientationDistance = " + orientationDistance);
      }

      assertEquals(0.0, positionDistance, positionThreshold);
      assertEquals(0.0, orientationDistance, orientationThreshold);
   }
}
