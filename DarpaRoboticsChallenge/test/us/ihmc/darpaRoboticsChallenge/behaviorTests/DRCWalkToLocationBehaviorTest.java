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

   private final double ASSUMED_WALKING_SPEED_mPerSec = 0.2;

   private final double POSITION_THRESHOLD = 0.1;
   private final double ORIENTATION_THRESHOLD = 0.05;
   private final double EXTRA_SIM_TIME_FOR_SETTLING = 1.0;

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

   @AverageDuration
   @Test(timeout = 300000)
   public void testWalkForwardsX() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double walkDistance = RandomTools.generateRandomDouble(new Random(), 1.0, 2.0);
      Vector2d walkDirection = new Vector2d(1, 0);

      FramePose2d desiredMidFeetPose = createDesiredPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose);

      success = drcBehaviorTestHelper.executeBehaviorUntilDone(walkToLocationBehavior);
      assertTrue(success);

      FramePose2d finalMidFeetPose = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly();
      assertPosesAreWithinThresholds(desiredMidFeetPose, finalMidFeetPose);
      assertTrue(walkToLocationBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   @AverageDuration
   @Test(timeout = 300000)
   public void testTurn90WalkTurnNeg90() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double walkDistance = RandomTools.generateRandomDouble(new Random(), 1.0, 2.0);
      Vector2d walkDirection = new Vector2d(0, 1);

      FramePose2d desiredMidFeetPose = createDesiredPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose);

      success = drcBehaviorTestHelper.executeBehaviorUntilDone(walkToLocationBehavior);
      assertTrue(success);

      FramePose2d finalMidFeetPose = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly();
      assertPosesAreWithinThresholds(desiredMidFeetPose, finalMidFeetPose);
      assertTrue(walkToLocationBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   @AverageDuration
   @Test(timeout = 300000)
   public void testWalkAndStopBehavior() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double walkDistance = 4.0;
      Vector2d walkDirection = new Vector2d(1, 0);

      FramePose2d desiredMidFeetPose = createDesiredPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose);

      TrajectoryPercentCompletedUpdatable percentTrajectoryCompletedUpdatable = createTrajectoryPercentCompletedUpdatable(desiredMidFeetPose);

      double percentDistanceToWalkBeforeStop = 50.0;
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(walkToLocationBehavior, percentTrajectoryCompletedUpdatable, percentDistanceToWalkBeforeStop);
      assertTrue(success);
      assertTrue(!walkToLocationBehavior.isDone());

      walkToLocationBehavior.stop();
      SysoutTool.println("Stopping Behavior", DEBUG);
      
      FramePose2d midFeetPoseAfterStop = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly();

      double simulateForThisLongAfterStop = 2.0;
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(walkToLocationBehavior, simulateForThisLongAfterStop);
      assertTrue(success);
      assertTrue(!walkToLocationBehavior.isDone());
      
      FramePose2d midFeetPoseFinal = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly();

      double maxStepLength = getRobotModel().getWalkingControllerParameters().getMaxStepLength();
      assertPosesAreWithinThresholds(midFeetPoseAfterStop, midFeetPoseFinal, maxStepLength);
      
      assertEquals(percentDistanceToWalkBeforeStop, percentTrajectoryCompletedUpdatable.getPercentTrajectoryCompleted(), 30.0);

      BambooTools.reportTestFinishedMessage();
   }

   @AverageDuration
   @Test(timeout = 300000)
   public void testWalkPauseAndResumeBehavior() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double walkDistance = 3.0;
      Vector2d walkDirection = new Vector2d(1, 0);

      FramePose2d desiredMidFeetPose = createDesiredPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose);

      TrajectoryPercentCompletedUpdatable percentTrajectoryCompletedUpdatable = createTrajectoryPercentCompletedUpdatable(desiredMidFeetPose);

      double percentDistanceToWalkBeforePause = 30.0;
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(walkToLocationBehavior, percentTrajectoryCompletedUpdatable, percentDistanceToWalkBeforePause);
      assertTrue(success);
      assertTrue(!walkToLocationBehavior.isDone());

      walkToLocationBehavior.pause();
      SysoutTool.println("Pausing Behavior", DEBUG);

      FramePose2d midFeetPoseAfterPause = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly();

      double simulateForThisLongAfterPause = 2.0;
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(walkToLocationBehavior, simulateForThisLongAfterPause);
      assertTrue(success);
      assertTrue(!walkToLocationBehavior.isDone());

      FramePose2d midFeetPoseAfterPauseAndWait = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly();

      double maxStepLength = getRobotModel().getWalkingControllerParameters().getMaxStepLength();
      assertPosesAreWithinThresholds(midFeetPoseAfterPause, midFeetPoseAfterPauseAndWait, maxStepLength); 
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
   
   @AverageDuration
   @Test(timeout = 300000)
   public void testWalkPauseLateAndResumeBehavior() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double walkDistance = 3.0;
      Vector2d walkDirection = new Vector2d(1, 0);

      FramePose2d desiredMidFeetPose = createDesiredPose2d(walkDistance, walkDirection);
      WalkToLocationBehavior walkToLocationBehavior = createAndSetupWalkToLocationBehavior(desiredMidFeetPose);

      TrajectoryPercentCompletedUpdatable percentTrajectoryCompletedUpdatable = createTrajectoryPercentCompletedUpdatable(desiredMidFeetPose);

      double percentDistanceToWalkBeforePause = 50.0;
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(walkToLocationBehavior, percentTrajectoryCompletedUpdatable, percentDistanceToWalkBeforePause);
      assertTrue(success);
      assertTrue(!walkToLocationBehavior.isDone());

      walkToLocationBehavior.pause();
      SysoutTool.println("Pausing Behavior", DEBUG);

      FramePose2d midFeetPoseAfterPause = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly();

      double simulateForThisLongAfterPause = 2.0;
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(walkToLocationBehavior, simulateForThisLongAfterPause);
      assertTrue(success);
      assertTrue(!walkToLocationBehavior.isDone());

      FramePose2d midFeetPoseAfterPauseAndWait = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly();

      double maxStepLength = getRobotModel().getWalkingControllerParameters().getMaxStepLength();
      assertPosesAreWithinThresholds(midFeetPoseAfterPause, midFeetPoseAfterPauseAndWait, maxStepLength); 
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

   private TrajectoryPercentCompletedUpdatable createTrajectoryPercentCompletedUpdatable(FramePose2d desiredMidFeetPose)
   {
      List<GroundContactPoint> gcPoints = drcBehaviorTestHelper.getRobot().getFootGroundContactPoints(RobotSide.LEFT);
      Joint ankleJoint = gcPoints.get(0).getParentJoint();

      FramePose poseAtTrajectoryEnd = new FramePose();
      RigidBodyTransform tempTransformToWorld = new RigidBodyTransform();
      desiredMidFeetPose.getPose(tempTransformToWorld);
      poseAtTrajectoryEnd.setPose(tempTransformToWorld);
      
      TrajectoryPercentCompletedUpdatable percentTrajectoryCompleteUpdatable = new TrajectoryPercentCompletedUpdatable(poseAtTrajectoryEnd, ankleJoint);
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

   private double getPercentDistanceWalked(FramePose2d initialMidFeetPose, FramePose2d finalDesiredMidFeetPose)
   {
      double totalDistanceToWalk = initialMidFeetPose.getPositionDistance(finalDesiredMidFeetPose);

      FramePose2d currentMidFeetPose = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly();

      double distanceWalked = initialMidFeetPose.getPositionDistance(currentMidFeetPose);

      double percentDistanceWalked = 100.0 * distanceWalked / totalDistanceToWalk;

      return percentDistanceWalked;
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

   private void assertPosesAreWithinThresholds(FramePose2d framePose1, FramePose2d framePose2)
   {
      assertPosesAreWithinThresholds(framePose1, framePose2, POSITION_THRESHOLD);
   }
   
   private void assertPosesAreWithinThresholds(FramePose2d framePose1, FramePose2d framePose2, double positionThreshold)
   {
      double positionDistance = framePose1.getPositionDistance(framePose2);
      double orientationDistance = framePose1.getOrientationDistance(framePose2);

      if (DEBUG)
      {
         SysoutTool.println(" desired Midfeet Pose :\n" + framePose1 + "\n");
         SysoutTool.println(" actual Midfeet Pose :\n" + framePose2 + "\n");

         SysoutTool.println(" positionDistance = " + positionDistance);
         SysoutTool.println(" orientationDistance = " + orientationDistance);
      }

      assertEquals(0.0, positionDistance, positionThreshold);
      assertEquals(0.0, orientationDistance, ORIENTATION_THRESHOLD);
   }
}
