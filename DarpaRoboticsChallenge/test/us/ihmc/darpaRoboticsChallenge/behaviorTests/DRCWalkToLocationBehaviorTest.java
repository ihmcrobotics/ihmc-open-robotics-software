package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;
import java.util.Random;

import javax.vecmath.Vector2d;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoPacketCommunicator;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.utilities.TrajectoryPercentCompletedUpdatable;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.AsyncContinuousExecutor;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.SysoutTool;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.TimerTaskScheduler;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FramePose2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
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
   private final double ORIENTATION_THRESHOLD = 0.05;

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
      FramePose2d desiredMidFeetPose = createDesiredPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose);

      SysoutTool.println("Starting to Execute Behavior", DEBUG);
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(walkToLocationBehavior);
      assertTrue(success);
      SysoutTool.println("Behavior Should be done", DEBUG);

      assertCurrentMidFeetPoseIsWithinThreshold(desiredMidFeetPose);
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
      FramePose2d desiredMidFeetPose = createDesiredPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose);

      SysoutTool.println("Starting to Execute Behavior", DEBUG);
      double percentDistanceToWalkBeforeStop = 50.0;
      TrajectoryPercentCompletedUpdatable percentTrajectoryCompletedUpdatable = createTrajectoryPercentCompletedUpdatable(desiredMidFeetPose,
            percentDistanceToWalkBeforeStop);
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(walkToLocationBehavior, percentTrajectoryCompletedUpdatable,
            percentDistanceToWalkBeforeStop);
      assertTrue(success);
      assertTrue(!walkToLocationBehavior.isDone());
      SysoutTool.println("Stopping Behavior", DEBUG);
      FramePose2d midFeetPoseAtStop = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly();
      walkToLocationBehavior.stop();

      double simulateForThisLongAfterStop = 2.0;
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(walkToLocationBehavior, simulateForThisLongAfterStop);
      assertTrue(success);
      assertTrue(!walkToLocationBehavior.isDone());
      SysoutTool.println("Done waiting in stopped mode", DEBUG);

      FramePose2d midFeetPoseFinal = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly();
      double maxStepLength = getRobotModel().getWalkingControllerParameters().getMaxStepLength();
      assertPosesAreWithinThresholds(midFeetPoseAtStop, midFeetPoseFinal, maxStepLength);
      assertEquals(percentDistanceToWalkBeforeStop, percentTrajectoryCompletedUpdatable.getPercentTrajectoryCompleted(), 30.0);

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
      FramePose2d desiredMidFeetPose = createDesiredPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose);

      SysoutTool.println("Starting to Execute Behavior", DEBUG);
      double percentDistanceToWalkBeforePause = 30.0;
      TrajectoryPercentCompletedUpdatable percentTrajectoryCompletedUpdatable = createTrajectoryPercentCompletedUpdatable(desiredMidFeetPose,
            percentDistanceToWalkBeforePause);
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(walkToLocationBehavior, percentTrajectoryCompletedUpdatable,
            percentDistanceToWalkBeforePause);
      assertTrue(success);
      assertTrue(!walkToLocationBehavior.isDone());
      SysoutTool.println("Pausing Behavior", DEBUG);
      FramePose2d midFeetPoseAtPause = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly();
      walkToLocationBehavior.pause();

      double simulateForThisLongAfterPause = 2.0;
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(walkToLocationBehavior, simulateForThisLongAfterPause);
      assertTrue(success);
      assertTrue(!walkToLocationBehavior.isDone());
      FramePose2d midFeetPoseAfterPauseAndWait = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly();
      SysoutTool.println("Done waiting in paused mode", DEBUG);

      double maxStepLength = getRobotModel().getWalkingControllerParameters().getMaxStepLength();
      assertPosesAreWithinThresholds(midFeetPoseAtPause, midFeetPoseAfterPauseAndWait, maxStepLength);
      assertEquals(percentDistanceToWalkBeforePause, percentTrajectoryCompletedUpdatable.getPercentTrajectoryCompleted(), 30.0);

      SysoutTool.println("Resuming Behavior", DEBUG);
      walkToLocationBehavior.resume();
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(walkToLocationBehavior);
      assertTrue(success);
      assertTrue(walkToLocationBehavior.isDone());
      assertCurrentMidFeetPoseIsWithinThreshold(desiredMidFeetPose);
      SysoutTool.println("Behavior Should be done", DEBUG);

      BambooTools.reportTestFinishedMessage();
   }

   @AverageDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testWalkPauseAndResumeOnLastStepBehavior() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double walkDistance = 3.0;
      Vector2d walkDirection = new Vector2d(1, 0);
      FramePose2d desiredMidFeetPose = createDesiredPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose);

      double percentDistanceToWalkBeforePause = 50.0;
      TrajectoryPercentCompletedUpdatable percentTrajectoryCompletedUpdatable = createTrajectoryPercentCompletedUpdatable(desiredMidFeetPose,
            percentDistanceToWalkBeforePause);
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(walkToLocationBehavior, percentTrajectoryCompletedUpdatable,
            percentDistanceToWalkBeforePause);
      assertTrue(success);
      assertTrue(!walkToLocationBehavior.isDone());
      SysoutTool.println("Pausing Behavior", DEBUG);
      FramePose2d midFeetPoseAtPause = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly();
      walkToLocationBehavior.pause();

      double simulateForThisLongAfterPause = 2.0;
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(walkToLocationBehavior, simulateForThisLongAfterPause);
      assertTrue(success);
      assertTrue(!walkToLocationBehavior.isDone());
      FramePose2d midFeetPoseAfterPauseAndWait = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly();

      double maxStepLength = getRobotModel().getWalkingControllerParameters().getMaxStepLength();
      assertPosesAreWithinThresholds(midFeetPoseAtPause, midFeetPoseAfterPauseAndWait, maxStepLength);
      assertEquals(percentDistanceToWalkBeforePause, percentTrajectoryCompletedUpdatable.getPercentTrajectoryCompleted(), 30.0);

      walkToLocationBehavior.resume();
      SysoutTool.println("Resuming Behavior", DEBUG);
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(walkToLocationBehavior);
      assertTrue(success);
      FramePose2d finalMidFeetPose = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly();
      assertTrue(walkToLocationBehavior.isDone());
      assertPosesAreWithinThresholds(desiredMidFeetPose, finalMidFeetPose);

      BambooTools.reportTestFinishedMessage();
   }

   private TrajectoryPercentCompletedUpdatable createTrajectoryPercentCompletedUpdatable(FramePose2d desiredMidFeetPose2d, double percentTrajectoryToComplete)
   {
      List<GroundContactPoint> gcPoints = drcBehaviorTestHelper.getRobot().getFootGroundContactPoints(RobotSide.LEFT);
      Joint ankleJoint = gcPoints.get(0).getParentJoint();

      RigidBodyTransform midFeetTransformToWorld = new RigidBodyTransform();
      desiredMidFeetPose2d.getPose(midFeetTransformToWorld);
      FramePose desiredMidFeetPose = new FramePose(ReferenceFrame.getWorldFrame(), midFeetTransformToWorld);

      TrajectoryPercentCompletedUpdatable percentTrajectoryCompleteUpdatable = new TrajectoryPercentCompletedUpdatable(desiredMidFeetPose, ankleJoint,
            percentTrajectoryToComplete);
      return percentTrajectoryCompleteUpdatable;
   }

   private FramePose2d createDesiredPose2d(double walkDistance, Vector2d walkDirection)
   {
      FramePose2d initialMidFeetPose = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly();
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

   private FramePose2d getCurrentMidFeetPose2d_THIS_DOES_NOT_WORK(ReferenceFrames referenceFrames)
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

   private FramePose2d getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly()
   {
      SDFRobot robot = drcBehaviorTestHelper.getRobot();
      FramePose midFeetPose = getRobotMidFeetPose(robot);

      FramePose2d ret = new FramePose2d();
      ret.setPoseIncludingFrame(ReferenceFrame.getWorldFrame(), midFeetPose.getX(), midFeetPose.getY(), midFeetPose.getYaw());

      return ret;
   }

   private FramePose getRobotMidFeetPose(SDFRobot robot)
   {
      FramePose leftFootPose = getRobotFootPose(robot, RobotSide.LEFT);
      FramePose rightFootPose = getRobotFootPose(robot, RobotSide.RIGHT);

      FramePose ret = new FramePose();
      ret.interpolate(leftFootPose, rightFootPose, 0.5);

      return ret;
   }

   private FramePose getRobotFootPose(SDFRobot robot, RobotSide robotSide)
   {
      List<GroundContactPoint> gcPoints = robot.getFootGroundContactPoints(robotSide);
      Joint ankleJoint = gcPoints.get(0).getParentJoint();
      RigidBodyTransform ankleTransformToWorld = new RigidBodyTransform();
      ankleJoint.getTransformToWorld(ankleTransformToWorld);

      FramePose ret = new FramePose();
      ret.setPose(ankleTransformToWorld);

      return ret;
   }

   private void assertCurrentMidFeetPoseIsWithinThreshold(FramePose2d desiredMidFeetPose)
   {
      FramePose2d currentMidFeetPose = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly();
      assertPosesAreWithinThresholds(desiredMidFeetPose, currentMidFeetPose);
   }

   private void assertPosesAreWithinThresholds(FramePose2d desiredPose, FramePose2d actualPose)
   {
      assertPosesAreWithinThresholds(desiredPose, actualPose, POSITION_THRESHOLD);
   }

   private void assertPosesAreWithinThresholds(FramePose2d desiredPose, FramePose2d actualPose, double positionThreshold)
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
      assertEquals(0.0, orientationDistance, ORIENTATION_THRESHOLD);
   }
}
