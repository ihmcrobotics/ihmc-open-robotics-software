package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.communication.packets.walking.HeadOrientationPacket;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HeadOrientationBehavior;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.humanoidRobotics.model.FullRobotModel;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class DRCHeadOrientationBehaviorTest implements MultiRobotTestInterface
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
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCHeadOrientationBehaviorTest.class + " after class.");
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

      DRCDemo01NavigationEnvironment testEnvironment = new DRCDemo01NavigationEnvironment();
      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel());
   }

   //TODO: Fix HeadOrienationManager() so that head actually tracks desired yaw and roll orientations.  Currently, only pitch orientation tracks properly.

   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testHeadPitch() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double trajectoryTime = 4.0;
      Vector3d axis = new Vector3d(0, 1, 0);
      double rotationAngle = MAX_ANGLE_TO_TEST_RAD * RandomTools.generateRandomDouble(new Random(), 0.3, 1.0);

      HeadOrientationPacket headOrientationPacket = createHeadOrientationPacket(axis, rotationAngle, trajectoryTime);
      testHeadOrientationBehavior(headOrientationPacket, trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING);
      BambooTools.reportTestFinishedMessage();
   }

   //   @Test(timeout = 300000)
   public void testHeadRoll() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double trajectoryTime = 4.0;
      Vector3d axis = new Vector3d(1, 0, 0);
      double rotationAngle = MAX_ANGLE_TO_TEST_RAD * RandomTools.generateRandomDouble(new Random(), 0.3, 1.0);

      HeadOrientationPacket headOrientationPacket = createHeadOrientationPacket(axis, rotationAngle, trajectoryTime);
      testHeadOrientationBehavior(headOrientationPacket, trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING);
      BambooTools.reportTestFinishedMessage();
   }

   //   @Test(timeout = 300000)
   public void testHeadYaw() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double trajectoryTime = 4.0;
      Vector3d axis = new Vector3d(0, 0, 1);
      double rotationAngle = MAX_ANGLE_TO_TEST_RAD * RandomTools.generateRandomDouble(new Random(), 0.3, 1.0);

      HeadOrientationPacket headOrientationPacket = createHeadOrientationPacket(axis, rotationAngle, trajectoryTime);
      testHeadOrientationBehavior(headOrientationPacket, trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING);
      BambooTools.reportTestFinishedMessage();
   }

   //   @Test(timeout = 300000)
   public void testRandomOrientation() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double trajectoryTime = 4.0;
      Quat4d desiredHeadQuat = new Quat4d(RandomTools.generateRandomQuaternion(new Random(), MAX_ANGLE_TO_TEST_RAD));
      HeadOrientationPacket headOrientationPacket = new HeadOrientationPacket(desiredHeadQuat, trajectoryTime);

      testHeadOrientationBehavior(headOrientationPacket, trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING);
      BambooTools.reportTestFinishedMessage();
   }

   private HeadOrientationPacket createHeadOrientationPacket(Vector3d axis, double rotationAngle, double trajectoryTime)
   {
      AxisAngle4d desiredAxisAngle = new AxisAngle4d();
      desiredAxisAngle.set(axis, rotationAngle);
      Quat4d desiredHeadQuat = new Quat4d();
      desiredHeadQuat.set(desiredAxisAngle);

      HeadOrientationPacket headOrientationPacket = new HeadOrientationPacket(desiredHeadQuat, trajectoryTime);
      return headOrientationPacket;
   }

   private void testHeadOrientationBehavior(HeadOrientationPacket headOrientationPacket, double trajectoryTime) throws SimulationExceededMaximumTimeException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      final HeadOrientationBehavior headOrientBehavior = new HeadOrientationBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            drcBehaviorTestHelper.getYoTime());

      headOrientBehavior.initialize();
      headOrientBehavior.setInput(headOrientationPacket);
      assertTrue(headOrientBehavior.hasInputBeenSet());

      FramePose initialHeadPose = getCurrentHeadPose(drcBehaviorTestHelper.getSDFFullRobotModel());
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(headOrientBehavior, trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING);
      assertTrue(success);
      FramePose finalHeadPose = getCurrentHeadPose(drcBehaviorTestHelper.getSDFFullRobotModel());

      if (DEBUG)
      {
         PrintTools.debug(this, " initial Head Pose :\n" + initialHeadPose);
      }

      FramePose desiredHeadPose = new FramePose();
      desiredHeadPose.setPose(initialHeadPose.getFramePointCopy().getPoint(), headOrientationPacket.orientation);
      assertPosesAreWithinThresholds(desiredHeadPose, finalHeadPose);
      assertTrue(headOrientBehavior.isDone());
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
