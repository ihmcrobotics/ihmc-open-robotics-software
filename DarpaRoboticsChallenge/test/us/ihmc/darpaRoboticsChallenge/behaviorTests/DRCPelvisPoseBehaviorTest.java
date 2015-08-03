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

import us.ihmc.communication.packets.walking.PelvisPosePacket;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisPoseBehavior;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.robotics.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class DRCPelvisPoseBehaviorTest implements MultiRobotTestInterface
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
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCPelvisPoseBehaviorTest.class + " after class.");
   }

   private static final boolean DEBUG = false;

   private final double MAX_ANGLE_TO_TEST_RAD = Math.toRadians(15.0);
   private final double MAX_TRANSLATION_TO_TEST_M = 0.15;

   private final double POSITION_THRESHOLD = 0.05;
   private final double ORIENTATION_THRESHOLD = 0.07;
   private final double EXTRA_SIM_TIME_FOR_SETTLING = 1.0;

   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   @Before
   public void setUp()
   {
      DRCDemo01NavigationEnvironment testEnvironment = new DRCDemo01NavigationEnvironment();

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel());
   }

   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testSingleRandomPelvisRotationNoTranslation() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      Quat4d desiredPelvisQuat = new Quat4d(RandomTools.generateRandomQuaternion(new Random(), 0.8 * MAX_ANGLE_TO_TEST_RAD));
      PelvisPosePacket pelvisPosePacket = new PelvisPosePacket(desiredPelvisQuat);

      PelvisPoseBehavior pelvisPoseBehavior = testPelvisPoseBehavior(pelvisPosePacket);

      assertTrue(pelvisPoseBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testPelvisPitchRotationNoTranslation() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      Vector3d rotationAxis = new Vector3d(0, 1, 0);
      double rotationAngle = MAX_ANGLE_TO_TEST_RAD;
      PelvisPosePacket pelvisPosePacket = createRotationOnlyPelvisPosePacket(rotationAxis, rotationAngle);

      PelvisPoseBehavior pelvisPoseBehavior = testPelvisPoseBehavior(pelvisPosePacket);

      assertTrue(pelvisPoseBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testPelvisRollRotationNoTranslation() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      Vector3d rotationAxis = new Vector3d(1, 0, 0);
      double rotationAngle = MAX_ANGLE_TO_TEST_RAD;
      PelvisPosePacket pelvisPosePacket = createRotationOnlyPelvisPosePacket(rotationAxis, rotationAngle);

      PelvisPoseBehavior pelvisPoseBehavior = testPelvisPoseBehavior(pelvisPosePacket);

      assertTrue(pelvisPoseBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testPelvisYawRotationNoTranslation() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      Vector3d rotationAxis = new Vector3d(0, 0, 1);
      double rotationAngle = 0.8 * MAX_ANGLE_TO_TEST_RAD;
      PelvisPosePacket pelvisPosePacket = createRotationOnlyPelvisPosePacket(rotationAxis, rotationAngle);

      PelvisPoseBehavior pelvisPoseBehavior = testPelvisPoseBehavior(pelvisPosePacket);

      assertTrue(pelvisPoseBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testPelvisXTranslation() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      Vector3d desiredDirection = new Vector3d(1, 0, 0);
      FramePose currentPelvisPose = getCurrentPelvisPose();
      PelvisPosePacket pelvisPosePacket = createTranslationOnlyPelvisPosePacket(desiredDirection, currentPelvisPose);

      PelvisPoseBehavior pelvisPoseBehavior = testPelvisPoseBehavior(pelvisPosePacket);

      assertTrue(pelvisPoseBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testPelvisYTranslation() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      Vector3d desiredDirection = new Vector3d(0, 1, 0);
      FramePose currentPelvisPose = getCurrentPelvisPose();
      PelvisPosePacket pelvisPosePacket = createTranslationOnlyPelvisPosePacket(desiredDirection, currentPelvisPose);

      PelvisPoseBehavior pelvisPoseBehavior = testPelvisPoseBehavior(pelvisPosePacket);

      assertTrue(pelvisPoseBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   //      @Test(timeout = 300000)
   public void testPelvisZTranslation() throws SimulationExceededMaximumTimeException
   {
      //FIXME: This z-translation is not supported by PelvisPoseBehavior; already handled by ComHeightBehavior.  
      BambooTools.reportTestStartedMessage();
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      Vector3d desiredDirection = new Vector3d(0, 0, 1);
      FramePose currentPelvisPose = getCurrentPelvisPose();
      PelvisPosePacket pelvisPosePacket = createTranslationOnlyPelvisPosePacket(desiredDirection, currentPelvisPose);

      PelvisPoseBehavior pelvisPoseBehavior = testPelvisPoseBehavior(pelvisPosePacket);

      assertTrue(pelvisPoseBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   private PelvisPosePacket createRotationOnlyPelvisPosePacket(Vector3d rotationAxis, double rotationAngle)
   {
      AxisAngle4d desiredPelvisAxisAngle = new AxisAngle4d(rotationAxis, rotationAngle);
      Quat4d desiredPelvisQuat = new Quat4d();
      desiredPelvisQuat.set(desiredPelvisAxisAngle);

      PelvisPosePacket pelvisPosePacket = new PelvisPosePacket(desiredPelvisQuat);
      return pelvisPosePacket;
   }

   private PelvisPosePacket createTranslationOnlyPelvisPosePacket(Vector3d desiredDirection, FramePose currentPelvisPose)
   {
      desiredDirection.normalize();
      double distanceToTranslate = RandomTools.generateRandomDouble(new Random(), MAX_TRANSLATION_TO_TEST_M);
      desiredDirection.scale(distanceToTranslate);
      Point3d desiredPelvisPoint = new Point3d(currentPelvisPose.getFramePointCopy().getPoint());
      desiredPelvisPoint.add(desiredDirection);

      PelvisPosePacket pelvisPosePacket = new PelvisPosePacket(desiredPelvisPoint);
      return pelvisPosePacket;
   }

   private PelvisPoseBehavior testPelvisPoseBehavior(PelvisPosePacket pelvisPosePacket) throws SimulationExceededMaximumTimeException
   {
      final PelvisPoseBehavior pelvisPoseBehavior = new PelvisPoseBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            drcBehaviorTestHelper.getYoTime());

      pelvisPoseBehavior.initialize();
      pelvisPoseBehavior.setInput(pelvisPosePacket);
      assertTrue(pelvisPoseBehavior.hasInputBeenSet());

      double simulationTime = pelvisPosePacket.getTrajectoryTime() + EXTRA_SIM_TIME_FOR_SETTLING;
      FramePose initialPelvisPose = getCurrentPelvisPose();
      boolean success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(pelvisPoseBehavior, simulationTime);
      FramePose finalPelvisPose = getCurrentPelvisPose();

      if (DEBUG)
      {
         PrintTools.debug(this, " initial Pelvis Pose :\n" + initialPelvisPose + "\n");
      }
      FramePose desiredPelvisPose = new FramePose();

      if (pelvisPosePacket.position == null)
      {
         desiredPelvisPose.setOrientation(pelvisPosePacket.orientation);
         assertOrientationsAreWithinThresholds(desiredPelvisPose, finalPelvisPose);
      }
      else if (pelvisPosePacket.orientation == null)
      {
         desiredPelvisPose.setPosition(pelvisPosePacket.position);
         assertPositionsAreWithinThresholds(desiredPelvisPose, finalPelvisPose);
      }
      else
      {
         desiredPelvisPose.setPose(pelvisPosePacket.position, pelvisPosePacket.orientation);
         assertPosesAreWithinThresholds(desiredPelvisPose, finalPelvisPose);
      }

      assertTrue(success);
      return pelvisPoseBehavior;
   }

   private FramePose getCurrentPelvisPose()
   {
      FullRobotModel fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();
      drcBehaviorTestHelper.updateRobotModel();

      FramePose ret = new FramePose();
      ret.setToZero(fullRobotModel.getPelvis().getBodyFixedFrame());
      ret.changeFrame(ReferenceFrame.getWorldFrame());

      return ret;
   }

   private void assertPosesAreWithinThresholds(FramePose framePose1, FramePose framePose2)
   {
      double positionDistance = framePose1.getPositionDistance(framePose2);
      double orientationDistance = framePose1.getOrientationDistance(framePose2);

      if (DEBUG)
      {
         PrintTools.debug(this, " desired Pelvis Pose :\n" + framePose1 + "\n");
         PrintTools.debug(this, " actual Pelvis Pose :\n" + framePose2 + "\n");

         PrintTools.debug(this, " positionDistance = " + positionDistance);
         PrintTools.debug(this, " orientationDistance = " + orientationDistance);
      }

      if (!Double.isNaN(POSITION_THRESHOLD))
      {
         assertEquals("Pose position error :" + positionDistance + " exceeds threshold: " + POSITION_THRESHOLD, 0.0, positionDistance, POSITION_THRESHOLD);
      }
      assertEquals("Pose orientation error :" + orientationDistance + " exceeds threshold: " + ORIENTATION_THRESHOLD, 0.0, orientationDistance, ORIENTATION_THRESHOLD);
   }

   private void assertPositionsAreWithinThresholds(FramePose framePose1, FramePose framePose2)
   {
      double positionDistance = framePose1.getPositionDistance(framePose2);

      if (DEBUG)
      {
         PrintTools.debug(this, " desired Pelvis Pose :\n" + framePose1 + "\n");
         PrintTools.debug(this, " actual Pelvis Pose :\n" + framePose2 + "\n");

         PrintTools.debug(this, " positionDistance = " + positionDistance);
      }

      if (!Double.isNaN(POSITION_THRESHOLD))
      {
         assertEquals("Pose position error :" + positionDistance + " exceeds threshold: " + POSITION_THRESHOLD, 0.0, positionDistance, POSITION_THRESHOLD);
      }
   }

   private void assertOrientationsAreWithinThresholds(FramePose framePose1, FramePose framePose2)
   {
      double orientationDistance = framePose1.getOrientationDistance(framePose2);

      if (DEBUG)
      {
         PrintTools.debug(this, " desired Pelvis Pose :\n" + framePose1 + "\n");
         PrintTools.debug(this, " actual Pelvis Pose :\n" + framePose2 + "\n");

         PrintTools.debug(this, " orientationDistance = " + orientationDistance);
      }

      assertEquals("Pose orientation error :" + orientationDistance + " exceeds threshold: " + ORIENTATION_THRESHOLD, 0.0, orientationDistance, ORIENTATION_THRESHOLD);
   }
}
