package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.primitives.LookAtBehavior;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.humanoidRobotics.model.FullRobotModel;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class DRCLookAtBehaviorTest implements MultiRobotTestInterface
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
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCLookAtBehaviorTest.class + " after class.");
   }

   private static final boolean DEBUG = false;

   private final double MAX_ANGLE_TO_TEST_RAD = 15.0 * Math.PI / 180.0;
   private final double ORIENTATION_THRESHOLD = 0.007;
   private final double EXTRA_SIM_TIME_FOR_SETTLING = 1.0;

   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   @Before
   public void setUp()
   {
      DRCDemo01NavigationEnvironment testEnvironment = new DRCDemo01NavigationEnvironment();

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel());
   }

   //TODO: Fix HeadOrienationManager() so that head actually tracks desired yaw and roll orientations.  Currently, only pitch orientation tracks properly.

   @EstimatedDuration(duration = 27.5)
   @Test(timeout = 82410)
   public void testLookAtPitch() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double trajectoryTime = 4.0;
      Vector3d axis = new Vector3d(0, 1, 0);
      double rotationAngle = MAX_ANGLE_TO_TEST_RAD * RandomTools.generateRandomDouble(new Random(), 0.3, 1.0);
      Quat4d desiredHeadQuat = convertAxisAngleToQuat(axis, rotationAngle);

      LookAtBehavior lookAtBehavior = testLookAtBehavior(trajectoryTime, desiredHeadQuat);

      assertTrue("Behavior should be done, but it isn't.", lookAtBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   //   @Test(timeout = 300000)
   public void testLookAtYaw() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double trajectoryTime = 4.0;
      Vector3d axis = new Vector3d(0, 0, 1);
      double rotationAngle = MAX_ANGLE_TO_TEST_RAD * RandomTools.generateRandomDouble(new Random(), 0.3, 1.0);
      Quat4d desiredHeadQuat = convertAxisAngleToQuat(axis, rotationAngle);

      LookAtBehavior lookAtBehavior = testLookAtBehavior(trajectoryTime, desiredHeadQuat);

      assertTrue(lookAtBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   //   @Test(timeout = 300000)
   public void testLookAtRoll() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double trajectoryTime = 4.0;
      Vector3d axis = new Vector3d(1, 0, 0);
      double rotationAngle = MAX_ANGLE_TO_TEST_RAD * RandomTools.generateRandomDouble(new Random(), 0.3, 1.0);
      Quat4d desiredHeadQuat = convertAxisAngleToQuat(axis, rotationAngle);

      LookAtBehavior lookAtBehavior = testLookAtBehavior(trajectoryTime, desiredHeadQuat);

      assertTrue(lookAtBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   // @Test(timeout = 300000)
   public void testLookAtRandom() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double trajectoryTime = 4.0;
      Quat4d desiredHeadQuat = new Quat4d(RandomTools.generateRandomQuaternion(new Random(), MAX_ANGLE_TO_TEST_RAD));

      LookAtBehavior lookAtBehavior = testLookAtBehavior(trajectoryTime, desiredHeadQuat);

      assertTrue(lookAtBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   private LookAtBehavior testLookAtBehavior(double trajectoryTime, Quat4d desiredHeadQuat) throws SimulationExceededMaximumTimeException
   {
      FullRobotModel fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();

      FramePose initialHeadPose = getCurrentHeadPose(fullRobotModel);
      Point3d desiredLookAtPoint = computeDesiredLookAtPoint(initialHeadPose, desiredHeadQuat);

      LookAtBehavior lookAtBehavior = testLookAtBehavior(desiredLookAtPoint, trajectoryTime);

      FramePose finalHeadPose = getCurrentHeadPose(fullRobotModel);
      Quat4d finalHeadQuat = new Quat4d();
      finalHeadPose.getOrientation(finalHeadQuat);

      assertAxesParallel(desiredHeadQuat, finalHeadQuat);

      return lookAtBehavior;
   }

   private LookAtBehavior testLookAtBehavior(Point3d pointToLookAt, double trajectoryTime) throws SimulationExceededMaximumTimeException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      final LookAtBehavior lookAtBehavior = new LookAtBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(), getRobotModel()
            .getWalkingControllerParameters(), drcBehaviorTestHelper.getYoTime());

      lookAtBehavior.setLookAtLocation(pointToLookAt);
      assertTrue(lookAtBehavior.hasInputBeenSet());

      success = drcBehaviorTestHelper.executeBehaviorUntilDone(lookAtBehavior);

      assertTrue(success);

      return lookAtBehavior;
   }

   private void assertAxesParallel(Quat4d desiredHeadQuat, Quat4d finalHeadQuat)
   {
      Vector3d desiredLookAxis = getQuatAxis(desiredHeadQuat);
      Vector3d actualLookAxis = getQuatAxis(finalHeadQuat);

      desiredLookAxis.normalize();
      actualLookAxis.normalize();

      double angleBetweenAxes = Math.abs(Math.acos(desiredLookAxis.dot(actualLookAxis)));

      assertEquals("Angle between actual and desired lookAt directions," + angleBetweenAxes + " exceeds threshold: " + ORIENTATION_THRESHOLD, 0.0,
            angleBetweenAxes, ORIENTATION_THRESHOLD);
   }

   private Vector3d getQuatAxis(Quat4d quat)
   {
      AxisAngle4d axisAngle = new AxisAngle4d();
      axisAngle.set(quat);

      Vector3d ret = new Vector3d(axisAngle.x, axisAngle.y, axisAngle.z);

      return ret;
   }

   private Quat4d convertAxisAngleToQuat(Vector3d axis, double rotationAngle)
   {
      AxisAngle4d desiredAxisAngle = new AxisAngle4d();
      desiredAxisAngle.set(axis, rotationAngle);

      Quat4d desiredHeadQuat = new Quat4d();
      desiredHeadQuat.set(desiredAxisAngle);

      return desiredHeadQuat;
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

   private Point3d computeDesiredLookAtPoint(FramePose currentHeadPose, Quat4d desiredHeadQuat)
   {
      Vector3d desiredLookAtAxis = getLookAtAxis(desiredHeadQuat);

      Point3d desiredLookAtPt = new Point3d(currentHeadPose.getFramePointCopy().getPoint());
      desiredLookAtPt.add(desiredLookAtAxis);

      return desiredLookAtPt;
   }

   private Vector3d getLookAtAxis(Quat4d headQuat)
   {
      AxisAngle4d headAxisAngle = new AxisAngle4d();
      headAxisAngle.set(headQuat);

      Vector3d headToLookAtVec = new Vector3d(headAxisAngle.x, headAxisAngle.y, headAxisAngle.z);
      return headToLookAtVec;
   }
}
