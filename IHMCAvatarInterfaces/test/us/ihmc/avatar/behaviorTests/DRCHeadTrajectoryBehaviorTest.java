package us.ihmc.avatar.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HeadTrajectoryBehavior;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.time.GlobalTimer;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class DRCHeadTrajectoryBehaviorTest implements MultiRobotTestInterface
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
      
      

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @AfterClass
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCHeadTrajectoryBehaviorTest.class + " after class.");
   }

   private static final boolean DEBUG = false;

   private final double MAX_ANGLE_TO_TEST_RAD = 15.0 * Math.PI / 180.0;
   private final double POSITION_THRESHOLD = Double.NaN;
   private final double ORIENTATION_THRESHOLD = 0.007;
   private final double EXTRA_SIM_TIME_FOR_SETTLING = 1.0;

   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   @Before
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      DefaultCommonAvatarEnvironment testEnvironment = new DefaultCommonAvatarEnvironment();
      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, getSimpleRobotName(),
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel());
   }

   //TODO: Fix HeadOrienationManager() so that head actually tracks desired yaw and roll orientations.  Currently, only pitch orientation tracks properly.

	@ContinuousIntegrationTest(estimatedDuration = 37.6)
   @Test(timeout = 190000)
   public void testHeadPitch() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double trajectoryTime = 4.0;
      Vector3D axis = new Vector3D(0, 1, 0);
      double rotationAngle = MAX_ANGLE_TO_TEST_RAD * RandomTools.generateRandomDouble(new Random(), 0.3, 1.0);

      HeadTrajectoryMessage message = createHeadOrientationPacket(axis, rotationAngle, trajectoryTime);
      testHeadOrientationBehavior(message, trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   //   @Test(timeout = 300000)
   public void testHeadRoll() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double trajectoryTime = 4.0;
      Vector3D axis = new Vector3D(1, 0, 0);
      double rotationAngle = MAX_ANGLE_TO_TEST_RAD * RandomTools.generateRandomDouble(new Random(), 0.3, 1.0);

      HeadTrajectoryMessage message = createHeadOrientationPacket(axis, rotationAngle, trajectoryTime);
      testHeadOrientationBehavior(message, trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   //   @Test(timeout = 300000)
   public void testHeadYaw() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double trajectoryTime = 4.0;
      Vector3D axis = new Vector3D(0, 0, 1);
      double rotationAngle = MAX_ANGLE_TO_TEST_RAD * RandomTools.generateRandomDouble(new Random(), 0.3, 1.0);

      HeadTrajectoryMessage message = createHeadOrientationPacket(axis, rotationAngle, trajectoryTime);
      testHeadOrientationBehavior(message, trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   //   @Test(timeout = 300000)
   public void testRandomOrientation() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double trajectoryTime = 4.0;
      Quaternion desiredHeadQuat = new Quaternion(RandomTools.generateRandomQuaternion(new Random(), MAX_ANGLE_TO_TEST_RAD));
      HeadTrajectoryMessage message = new HeadTrajectoryMessage(trajectoryTime, desiredHeadQuat);
      testHeadOrientationBehavior(message, trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private HeadTrajectoryMessage createHeadOrientationPacket(Vector3D axis, double rotationAngle, double trajectoryTime)
   {
      AxisAngle desiredAxisAngle = new AxisAngle();
      desiredAxisAngle.set(axis, rotationAngle);
      Quaternion desiredHeadQuat = new Quaternion();
      desiredHeadQuat.set(desiredAxisAngle);

      HeadTrajectoryMessage message = new HeadTrajectoryMessage(trajectoryTime, desiredHeadQuat);
      return message;
   }

   private void testHeadOrientationBehavior(HeadTrajectoryMessage headTrajectoryMessage, double trajectoryTime) throws SimulationExceededMaximumTimeException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      final HeadTrajectoryBehavior headTrajectoryBehavior = new HeadTrajectoryBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            drcBehaviorTestHelper.getYoTime());

      headTrajectoryBehavior.initialize();
      headTrajectoryBehavior.setInput(headTrajectoryMessage);
      assertTrue(headTrajectoryBehavior.hasInputBeenSet());

      FramePose initialHeadPose = getCurrentHeadPose(drcBehaviorTestHelper.getSDFFullRobotModel());
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(headTrajectoryBehavior, trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING);
      assertTrue(success);
      FramePose finalHeadPose = getCurrentHeadPose(drcBehaviorTestHelper.getSDFFullRobotModel());

      if (DEBUG)
      {
         PrintTools.debug(this, " initial Head Pose :\n" + initialHeadPose);
      }

      FramePose desiredHeadPose = new FramePose();
      desiredHeadPose.setPose(initialHeadPose.getFramePointCopy().getPoint(), headTrajectoryMessage.getLastTrajectoryPoint().orientation);
      assertPosesAreWithinThresholds(desiredHeadPose, finalHeadPose);
      assertTrue(headTrajectoryBehavior.isDone());
   }

   private FramePose getCurrentHeadPose(FullRobotModel fullRobotModel)
   {
      FramePose ret = new FramePose();

      fullRobotModel.updateFrames();
      ReferenceFrame headFrame = fullRobotModel.getHead().getBodyFixedFrame();

      ret.setToZero(headFrame);
      ret.changeFrame(ReferenceFrame.getWorldFrame());

      return ret;
   }

   private void assertPosesAreWithinThresholds(FramePose framePose1, FramePose framePose2)
   {
      double positionDistance = framePose1.getPositionDistance(framePose2);
      double orientationDistance = framePose1.getOrientationDistance(framePose2);

      if (DEBUG)
      {
         PrintTools.debug(this, " desired Head Pose : \n" + framePose1);
         PrintTools.debug(this, " final Head Pose : \n" + framePose2);

         PrintTools.debug(this, " positionDistance = " + positionDistance);
         PrintTools.debug(this, " orientationDistance = " + orientationDistance);
      }

      if (!Double.isNaN(POSITION_THRESHOLD))
      {
         assertEquals("Pose position error :" + positionDistance + " exceeds threshold: " + POSITION_THRESHOLD, 0.0, positionDistance, POSITION_THRESHOLD);

         assertEquals(0.0, positionDistance, POSITION_THRESHOLD);
      }
      assertEquals("Pose orientation error :" + orientationDistance + " exceeds threshold: " + ORIENTATION_THRESHOLD, 0.0, orientationDistance, ORIENTATION_THRESHOLD);
   }
}
