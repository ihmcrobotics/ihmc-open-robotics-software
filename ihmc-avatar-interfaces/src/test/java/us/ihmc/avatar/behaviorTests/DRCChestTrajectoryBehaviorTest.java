package us.ihmc.avatar.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ChestTrajectoryBehavior;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.commons.thread.ThreadTools;

public abstract class DRCChestTrajectoryBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

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
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCChestTrajectoryBehaviorTest.class + " after class.");
   }

   private static final boolean DEBUG = false;

   private final double MAX_ANGLE_TO_TEST_RAD = 30.0 * Math.PI / 180.0;
   private final double POSITION_THRESHOLD = Double.NaN;
   private final double ORIENTATION_THRESHOLD = 0.007;

   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   @Before
   public void setUp()
   {
      DefaultCommonAvatarEnvironment testEnvironment = new DefaultCommonAvatarEnvironment();
      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, getSimpleRobotName(),
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel());
   }

   public void testSingleRandomChestOrientationMove() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      ReferenceFrame pelvisZUpFrame = drcBehaviorTestHelper.getReferenceFrames().getPelvisZUpFrame();

      Quaternion desiredChestQuat = new Quaternion(RandomGeometry.nextQuaternion(new Random(), 0.8 * MAX_ANGLE_TO_TEST_RAD));
      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(1.0, desiredChestQuat, ReferenceFrame.getWorldFrame(), pelvisZUpFrame);

      ChestTrajectoryBehavior chestOrientationBehavior = testChestOrientationBehavior(chestTrajectoryMessage);

      assertTrue(chestOrientationBehavior.isDone());

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private ChestTrajectoryBehavior testChestOrientationBehavior(ChestTrajectoryMessage chestTrajectoryMessage) throws SimulationExceededMaximumTimeException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      final ChestTrajectoryBehavior chestTrajectoryBehavior = new ChestTrajectoryBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            drcBehaviorTestHelper.getYoTime());

      chestTrajectoryBehavior.initialize();
      chestTrajectoryBehavior.setInput(chestTrajectoryMessage);
      assertTrue(chestTrajectoryBehavior.hasInputBeenSet());

      double totalSimTime = chestTrajectoryMessage.getSO3Trajectory().getTrajectoryTime();
      totalSimTime += 1.0;

      FramePose3D initialChestPose = getCurrentChestPose();
      success = success && drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(chestTrajectoryBehavior, totalSimTime);
      FramePose3D finalChestPose = getCurrentChestPose();

      PrintTools.debug(this, " initial Chest Pose :\n" + initialChestPose);
      PrintTools.debug(this, " final Chest Pose :\n" + finalChestPose);
      FramePose3D desiredChestPose = new FramePose3D();

      desiredChestPose.set(initialChestPose.getPosition(), chestTrajectoryMessage.getSO3Trajectory().getLastTrajectoryPoint().orientation);
      assertPosesAreWithinThresholds(desiredChestPose, finalChestPose);

      assertTrue(success);

      return chestTrajectoryBehavior;
   }

   private FramePose3D getCurrentChestPose()
   {
      drcBehaviorTestHelper.updateRobotModel();

      FramePose3D ret = new FramePose3D();
      ret.setToZero(drcBehaviorTestHelper.getSDFFullRobotModel().getChest().getBodyFixedFrame());
      ret.changeFrame(ReferenceFrame.getWorldFrame());

      return ret;
   }

   private void assertPosesAreWithinThresholds(FramePose3D framePose1, FramePose3D framePose2)
   {
      double positionDistance = framePose1.getPositionDistance(framePose2);
      double orientationDistance = framePose1.getOrientationDistance(framePose2);

      PrintTools.debug(this, " desired Chest Pose :\n" + framePose1);
      PrintTools.debug(this, " actual Chest Pose :\n" + framePose2);

      PrintTools.debug(this, " positionDistance = " + positionDistance);
      PrintTools.debug(this, " orientationDistance = " + orientationDistance);

      if (!Double.isNaN(POSITION_THRESHOLD))
      {
         assertEquals("Pose position error :" + positionDistance + " exceeds threshold: " + POSITION_THRESHOLD, 0.0, positionDistance, POSITION_THRESHOLD);
      }
      assertEquals("Pose orientation error :" + orientationDistance + " exceeds threshold: " + ORIENTATION_THRESHOLD, 0.0, orientationDistance, ORIENTATION_THRESHOLD);
   }
}
