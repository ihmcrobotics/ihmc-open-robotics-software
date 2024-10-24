package us.ihmc.avatar.behaviorTests;

import java.util.Random;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.HeadTrajectoryMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.avatar.testTools.scs2.SCS2BehaviorTestHelper;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HeadTrajectoryBehavior;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

import static org.junit.jupiter.api.Assertions.*;

@Disabled
@Deprecated
public abstract class DRCHeadTrajectoryBehaviorTest implements MultiRobotTestInterface
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
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCHeadTrajectoryBehaviorTest.class + " after class.");
   }

   private static final boolean DEBUG = false;

   private final double MAX_ANGLE_TO_TEST_RAD = 15.0 * Math.PI / 180.0;
   private final double POSITION_THRESHOLD = Double.NaN;
   private final double ORIENTATION_THRESHOLD = 0.009;
   private final double EXTRA_SIM_TIME_FOR_SETTLING = 1.0;

   private SCS2BehaviorTestHelper behaviorTestHelper;

   @BeforeEach
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      DefaultCommonAvatarEnvironment testEnvironment = new DefaultCommonAvatarEnvironment();
      SCS2AvatarTestingSimulation simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), testEnvironment, simulationTestingParameters);
      simulationTestHelper.start();
      behaviorTestHelper = new SCS2BehaviorTestHelper(simulationTestHelper);
   }

   //TODO: Fix HeadOrienationManager() so that head actually tracks desired yaw and roll orientations.  Currently, only pitch orientation tracks properly.

   @Test
   public void testHeadPitch()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double trajectoryTime = 4.0;
      Vector3D axis = new Vector3D(0, 1, 0);
      double rotationAngle = MAX_ANGLE_TO_TEST_RAD * RandomNumbers.nextDouble(new Random(), 0.3, 1.0);

      HeadTrajectoryMessage message = createHeadOrientationPacket(axis, rotationAngle, trajectoryTime);
      testHeadOrientationBehavior(message, trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING);
      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testHeadRoll()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double trajectoryTime = 4.0;
      Vector3D axis = new Vector3D(1, 0, 0);
      double rotationAngle = MAX_ANGLE_TO_TEST_RAD * RandomNumbers.nextDouble(new Random(), 0.3, 1.0);

      HeadTrajectoryMessage message = createHeadOrientationPacket(axis, rotationAngle, trajectoryTime);
      testHeadOrientationBehavior(message, trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING);
      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testHeadYaw()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double trajectoryTime = 4.0;
      Vector3D axis = new Vector3D(0, 0, 1);
      double rotationAngle = MAX_ANGLE_TO_TEST_RAD * RandomNumbers.nextDouble(new Random(), 0.3, 1.0);

      HeadTrajectoryMessage message = createHeadOrientationPacket(axis, rotationAngle, trajectoryTime);
      testHeadOrientationBehavior(message, trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING);
      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testRandomOrientation()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FullHumanoidRobotModel controllerFullRobotModel = behaviorTestHelper.getControllerFullRobotModel();
      ReferenceFrame chestCoMFrame = controllerFullRobotModel.getChest().getBodyFixedFrame();
      double trajectoryTime = 4.0;
      Quaternion desiredHeadQuat = new Quaternion(EuclidCoreRandomTools.nextQuaternion(new Random(), MAX_ANGLE_TO_TEST_RAD));
      HeadTrajectoryMessage message = HumanoidMessageTools.createHeadTrajectoryMessage(trajectoryTime, desiredHeadQuat, chestCoMFrame);
      message.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
      testHeadOrientationBehavior(message, trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING);
      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private HeadTrajectoryMessage createHeadOrientationPacket(Vector3D axis, double rotationAngle, double trajectoryTime)
   {
      AxisAngle desiredAxisAngle = new AxisAngle();
      desiredAxisAngle.set(axis, rotationAngle);
      Quaternion desiredHeadQuat = new Quaternion();
      desiredHeadQuat.set(desiredAxisAngle);

      FullHumanoidRobotModel controllerFullRobotModel = behaviorTestHelper.getControllerFullRobotModel();
      ReferenceFrame chestCoMFrame = controllerFullRobotModel.getChest().getBodyFixedFrame();

      HeadTrajectoryMessage message = HumanoidMessageTools.createHeadTrajectoryMessage(trajectoryTime, desiredHeadQuat, chestCoMFrame);
      message.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
      return message;
   }

   private void testHeadOrientationBehavior(HeadTrajectoryMessage headTrajectoryMessage, double trajectoryTime)
   {
      boolean success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);

      final HeadTrajectoryBehavior headTrajectoryBehavior = new HeadTrajectoryBehavior(behaviorTestHelper.getRobotName(),
                                                                                       behaviorTestHelper.getROS2Node(), behaviorTestHelper.getYoTime());

      headTrajectoryBehavior.initialize();
      headTrajectoryBehavior.setInput(headTrajectoryMessage);
      assertTrue(headTrajectoryBehavior.hasInputBeenSet());

      FramePose3D initialHeadPose = getCurrentHeadPose(behaviorTestHelper.getSDFFullRobotModel());
      success = behaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(headTrajectoryBehavior, trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING);
      assertTrue(success);
      FramePose3D finalHeadPose = getCurrentHeadPose(behaviorTestHelper.getSDFFullRobotModel());

      if (DEBUG)
      {
         LogTools.debug(" initial Head Pose :\n" + initialHeadPose);
      }

      FramePose3D desiredHeadPose = new FramePose3D();
      desiredHeadPose.set(initialHeadPose.getPosition(), headTrajectoryMessage.getSo3Trajectory().getTaskspaceTrajectoryPoints().getLast().getOrientation());
      assertPosesAreWithinThresholds(desiredHeadPose, finalHeadPose);
      assertTrue(headTrajectoryBehavior.isDone());
   }

   private FramePose3D getCurrentHeadPose(FullRobotModel fullRobotModel)
   {
      FramePose3D ret = new FramePose3D();

      fullRobotModel.updateFrames();
      ReferenceFrame headFrame = fullRobotModel.getHead().getBodyFixedFrame();

      ret.setToZero(headFrame);
      ret.changeFrame(ReferenceFrame.getWorldFrame());

      return ret;
   }

   private void assertPosesAreWithinThresholds(FramePose3D framePose1, FramePose3D framePose2)
   {
      double positionDistance = framePose1.getPositionDistance(framePose2);
      double orientationDistance = framePose1.getOrientationDistance(framePose2);

      if (DEBUG)
      {
         LogTools.info(" desired Head Pose : \n" + framePose1);
         LogTools.info(" final Head Pose : \n" + framePose2);

         LogTools.info(" positionDistance = " + positionDistance);
         LogTools.info(" orientationDistance = " + orientationDistance);
      }

      if (!Double.isNaN(POSITION_THRESHOLD))
      {
         assertEquals(0.0, positionDistance, POSITION_THRESHOLD, "Pose position error :" + positionDistance + " exceeds threshold: " + POSITION_THRESHOLD);

         assertEquals(0.0, positionDistance, POSITION_THRESHOLD);
      }
      assertEquals( 0.0, orientationDistance, ORIENTATION_THRESHOLD, "Pose orientation error :" + orientationDistance + " exceeds threshold: " + ORIENTATION_THRESHOLD);
   }
}
