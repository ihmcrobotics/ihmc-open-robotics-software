package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.LinkedHashMap;
import java.util.Random;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoPacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorControlModePacket.HumanoidBehaviorControlModeEnum;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.communication.util.PacketControllerTools;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.utilities.StopThreadUpdatable;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.io.printing.SysoutTool;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class DRCHandPoseBehaviorTest implements MultiRobotTestInterface
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
         armJointNames = null;
         armJointIndices = null;
      }

      GlobalTimer.clearTimers();

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @AfterClass
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCHandPoseBehaviorTest.class + " after class.");
   }

   private static final boolean DEBUG = false;

   private final double POSITION_THRESHOLD = 0.01;
   private final double ORIENTATION_THRESHOLD = 0.03;
   public static final double JOINT_POSITION_THRESHOLD = 0.05;
   private final double EXTRA_SIM_TIME_FOR_SETTLING = 1.0;

   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   private ArmJointName[] armJointNames;
   private int numberOfArmJoints;
   private LinkedHashMap<ArmJointName, Integer> armJointIndices = new LinkedHashMap<ArmJointName, Integer>();

   @Before
   public void setUp()
   {
      if (NetworkConfigParameters.USE_BEHAVIORS_MODULE)
      {
         throw new RuntimeException("Must set NetworkConfigParameters.USE_BEHAVIORS_MODULE = false in order to perform this test!");
      }

      DRCDemo01NavigationEnvironment testEnvironment = new DRCDemo01NavigationEnvironment();

      KryoPacketCommunicator controllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
            PacketDestination.CONTROLLER.ordinal(), "DRCControllerCommunicator");
      KryoPacketCommunicator networkObjectCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
            PacketDestination.NETWORK_PROCESSOR.ordinal(), "MockNetworkProcessorCommunicator");

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, networkObjectCommunicator, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel(), controllerCommunicator);

      armJointNames = drcBehaviorTestHelper.getSDFFullRobotModel().getRobotSpecificJointNames().getArmJointNames();
      numberOfArmJoints = armJointNames.length;

      for (int i = 0; i < numberOfArmJoints; i++)
      {
         armJointIndices.put(armJointNames[i], i);
      }
   }

   @AverageDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testJointSpaceHandPoseMove() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      SysoutTool.println("Initializing Sim", DEBUG);
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      SysoutTool.println("Initializing Behavior", DEBUG);
      RobotSide robotSide = RobotSide.LEFT;
      double trajectoryTime = 2.0;
      double[] desiredArmPose = createRandomArmPose(robotSide);
      final HandPoseBehavior handPoseBehavior = createNewHandPoseBehavior(robotSide, trajectoryTime, desiredArmPose);

      SysoutTool.println("Starting Behavior", DEBUG);
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(handPoseBehavior);
      SysoutTool.println("Behavior Should Be Done", DEBUG);

      assertTrue(success);
      assertCurrentHandPoseIsWithinThresholds(robotSide, desiredArmPose);
      assertTrue(handPoseBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   @AverageDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testTaskSpaceMoveToPoseAchievedInJointSpace() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      SysoutTool.println("Initializing Sim", DEBUG);
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      SysoutTool.println("Setting Hand Pose Behavior Input in Joint Space", DEBUG);
      RobotSide robotSide = RobotSide.LEFT;
      double trajectoryTime = 2.0;
      double[] desiredJointSpaceHandPose = createRandomArmPose(robotSide);
      final HandPoseBehavior handPoseBehavior = createNewHandPoseBehavior(robotSide, trajectoryTime, desiredJointSpaceHandPose);

      SysoutTool.println("Starting Joint Space Hand Pose Behavior", DEBUG);
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(handPoseBehavior);
      SysoutTool.println("Joint Space Hand Pose Behavior Should Be Done", DEBUG);

      assertTrue(success);
      assertCurrentHandPoseIsWithinThresholds(robotSide, desiredJointSpaceHandPose);
      assertTrue(handPoseBehavior.isDone());

      SysoutTool.println("Recording HandPose Acheived in Joint Space", DEBUG);
      FramePose handPoseAcheivedInJointSpace = getCurrentHandPose(robotSide);

      HandPosePacket goToHomePacket = PacketControllerTools.createGoToHomeHandPosePacket(robotSide, trajectoryTime);
      handPoseBehavior.initialize();
      handPoseBehavior.setInput(goToHomePacket);
      SysoutTool.println("Moving arm back to home pose", DEBUG);
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(handPoseBehavior);
      SysoutTool.println("Arm should be back to home pose", DEBUG);
      assertTrue(success);

      SysoutTool.println("Setting Hand Pose Behavior Input in Task Space", DEBUG);
      RigidBodyTransform handPoseTargetTransform = new RigidBodyTransform();
      handPoseAcheivedInJointSpace.getPose(handPoseTargetTransform);
      handPoseBehavior.initialize();
      handPoseBehavior.setInput(Frame.WORLD, handPoseTargetTransform, robotSide, trajectoryTime);
      assertTrue(handPoseBehavior.hasInputBeenSet());

      SysoutTool.println("Starting Task Space Hand Pose Behavior", DEBUG);
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(handPoseBehavior);
      SysoutTool.println("Task Space Hand Pose Behavior Should Be Done", DEBUG);

      assertTrue(success);
      assertCurrentHandPoseIsWithinThresholds(robotSide, handPoseAcheivedInJointSpace);
      assertTrue(handPoseBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   @AverageDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testUnreachableHandPoseMove() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      SysoutTool.println("Initializing Sim", DEBUG);
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      SysoutTool.println("Initializing Behavior", DEBUG);
      RobotSide robotSide = RobotSide.LEFT;
      FramePose handPoseStart = getCurrentHandPose(robotSide);
      FramePose handPoseTarget = new FramePose(handPoseStart);
      handPoseTarget.setX(handPoseTarget.getX() + 1.5);
      handPoseTarget.setOrientation(new double[] { 0.0, 0.0, 0.6 });
      double swingTrajectoryTime = 2.0;
      final HandPoseBehavior handPoseBehavior = createNewHandPoseBehavior(robotSide, swingTrajectoryTime, handPoseTarget);

      SysoutTool.println("Starting Behavior", DEBUG);
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(handPoseBehavior);
      SysoutTool.println("Behavior Should Be Done", DEBUG);

      assertTrue(success);
      double positionDistance = handPoseStart.getPositionDistance(handPoseTarget);
      double orientationDistance = handPoseStart.getOrientationDistance(handPoseTarget);
      boolean desiredHandPoseWasNotReached = positionDistance > POSITION_THRESHOLD || orientationDistance > ORIENTATION_THRESHOLD;
      assertTrue(desiredHandPoseWasNotReached);
      assertTrue(handPoseBehavior.isDone()); // hand pose should be done if elapsedTime > swingTrajectoryTime, even if desired pose was not reached

      BambooTools.reportTestFinishedMessage();
   }

   @AverageDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testTwoSimultaneousHandPoseBehaviors() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      SysoutTool.println("Initializing Sim", DEBUG);
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      SysoutTool.println("Initializing Behaviors", DEBUG);
      double trajectoryTime = 3.0;
      SideDependentList<double[]> desiredArmPoses = new SideDependentList<double[]>();
      SideDependentList<BehaviorInterface> handPoseBehaviors = new SideDependentList<BehaviorInterface>();

      for (RobotSide robotSide : RobotSide.values)
      {
         double[] desiredArmPose = createRandomArmPose(robotSide);
         desiredArmPoses.put(robotSide, desiredArmPose);

         HandPoseBehavior handPoseBehavior = createNewHandPoseBehavior(robotSide, trajectoryTime, desiredArmPose);
         handPoseBehaviors.put(robotSide, handPoseBehavior);
      }

      SysoutTool.println("Starting Behaviors", DEBUG);
      success = drcBehaviorTestHelper.executeBehaviorsSimulateAndBlockAndCatchExceptions(handPoseBehaviors, trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING);
      SysoutTool.println("Behaviors Should Be Done", DEBUG);

      assertTrue(success);
      for (RobotSide robotSide : RobotSide.values)
      {
         double[] desiredArmPose = desiredArmPoses.get(robotSide);
         BehaviorInterface handPoseBehavior = handPoseBehaviors.get(robotSide);

         assertCurrentHandPoseIsWithinThresholds(robotSide, desiredArmPose);
         assertTrue(handPoseBehavior.isDone());
      }

      BambooTools.reportTestFinishedMessage();
   }

   @AverageDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testPauseAndResume() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      SysoutTool.println("Initializing Sim", DEBUG);
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      SysoutTool.println("Initializing Behavior", DEBUG);
      final RobotSide robotSide = RobotSide.LEFT;
      double[] handPoseTarget = createRandomArmPose(robotSide);
      double trajectoryTime = 4.0;
      final HandPoseBehavior handPoseBehavior = createNewHandPoseBehavior(robotSide, trajectoryTime, handPoseTarget);

      ReferenceFrame frameToKeepTrackOf = drcBehaviorTestHelper.getSDFFullRobotModel().getHandControlFrame(robotSide);
      double pauseTime = 1.0;
      double resumeTime = 2.0;
      double stopTime = Double.POSITIVE_INFINITY;

      SysoutTool.println("Starting Behavior", DEBUG);
      StopThreadUpdatable stopThreadUpdatable = drcBehaviorTestHelper.executeBehaviorPauseAndResumeOrStop(handPoseBehavior, pauseTime, resumeTime, stopTime,
            frameToKeepTrackOf);
      SysoutTool.println("Behavior Should Be Done", DEBUG);

      FramePose handPoseAtPauseStart = stopThreadUpdatable.getTestFramePoseCopyAtTransition(HumanoidBehaviorControlModeEnum.PAUSE);
      FramePose handPoseAtPauseEnd = stopThreadUpdatable.getTestFramePoseCopyAtTransition(HumanoidBehaviorControlModeEnum.RESUME);
      assertPosesAreWithinThresholds(handPoseAtPauseStart, handPoseAtPauseEnd, 5.0 * POSITION_THRESHOLD, 5.0 * ORIENTATION_THRESHOLD);
      assertCurrentHandPoseIsWithinThresholds(robotSide, handPoseTarget);
      assertTrue(handPoseBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   @AverageDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testStop() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      SysoutTool.println("Initializing Sim", DEBUG);
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      SysoutTool.println("Initializing Behavior", DEBUG);
      final RobotSide robotSide = RobotSide.LEFT;
      double[] handPoseTarget = createRandomArmPose(robotSide);
      double trajectoryTime = 4.0;
      final HandPoseBehavior handPoseBehavior = createNewHandPoseBehavior(robotSide, trajectoryTime, handPoseTarget);

      final double simTimeBeforeStop = trajectoryTime / 2.0;

      ReferenceFrame frameToKeepTrackOf = drcBehaviorTestHelper.getSDFFullRobotModel().getHandControlFrame(robotSide);
      StopThreadUpdatable stopThreadUpdatable = drcBehaviorTestHelper.executeBehaviorPauseAndResumeOrStop(handPoseBehavior, Double.POSITIVE_INFINITY,
            Double.POSITIVE_INFINITY, simTimeBeforeStop, frameToKeepTrackOf);

      FramePose handPoseJustAfterStop = stopThreadUpdatable.getTestFramePoseCopyAtTransition(HumanoidBehaviorControlModeEnum.STOP);
      FramePose handPoseAfterResting = stopThreadUpdatable.getCurrentTestFramePoseCopy();
      assertPosesAreWithinThresholds(handPoseJustAfterStop, handPoseAfterResting, 5.0 * POSITION_THRESHOLD, 5.0 * ORIENTATION_THRESHOLD);
      assertTrue(!handPoseBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   private double[] getCurrentArmPose(RobotSide robotSide)
   {
      double[] armPose = new double[numberOfArmJoints];

      for (int jointNum = 0; jointNum < numberOfArmJoints; jointNum++)
      {
         ArmJointName jointName = armJointNames[jointNum];
         double currentAngle = drcBehaviorTestHelper.getSDFFullRobotModel().getArmJoint(robotSide, jointName).getQ();
         armPose[jointNum] = currentAngle;
      }

      return armPose;
   }

   private double[] createRandomArmPose(RobotSide robotSide)
   {
      double[] armPose = new double[numberOfArmJoints];

      for (int jointNum = 0; jointNum < numberOfArmJoints; jointNum++)
      {
         double qDesired = clipDesiredJointQToJointLimits(robotSide, armJointNames[jointNum], RandomTools.generateRandomDouble(new Random(), 1.5));
         armPose[jointNum] = qDesired;
      }

      return armPose;
   }

   private double clipDesiredJointQToJointLimits(RobotSide robotSide, ArmJointName armJointName, double desiredJointAngle)
   {
      FullRobotModel fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();

      double q;
      double qMin = fullRobotModel.getArmJoint(robotSide, armJointName).getJointLimitLower();
      double qMax = fullRobotModel.getArmJoint(robotSide, armJointName).getJointLimitUpper();

      if (qMin > qMax)
      {
         double temp = qMax;
         qMax = qMin;
         qMin = temp;
      }

      q = MathTools.clipToMinMax(desiredJointAngle, qMin, qMax);
      return q;
   }

   private HandPoseBehavior createNewHandPoseBehavior(RobotSide robotSide, double trajectoryTime, FramePose handPoseTarget)
   {
      HandPoseBehavior ret = new HandPoseBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(), drcBehaviorTestHelper.getYoTime());

      RigidBodyTransform handPoseTargetTransform = new RigidBodyTransform();
      handPoseTarget.getPose(handPoseTargetTransform);

      ret.initialize();
      ret.setInput(Frame.WORLD, handPoseTargetTransform, robotSide, trajectoryTime);

      return ret;
   }

   private HandPoseBehavior createNewHandPoseBehavior(RobotSide robotSide, double trajectoryTime, double[] desiredArmJointAngles)
   {
      final HandPoseBehavior ret = new HandPoseBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(), drcBehaviorTestHelper.getYoTime());

      HandPosePacket desiredHandPosePacket = new HandPosePacket(robotSide, trajectoryTime, desiredArmJointAngles);
      ret.setInput(desiredHandPosePacket);
      assertTrue(ret.hasInputBeenSet());

      return ret;
   }

   private FramePose offsetCurrentHandPose(RobotSide robotSide, double deltaX, double deltaY, double deltaZ)
   {
      FramePose handPoseStart = getCurrentHandPose(robotSide);

      FramePose ret = new FramePose(handPoseStart);
      ret.setX(ret.getX() + deltaX);
      ret.setY(ret.getY() + deltaY);
      ret.setZ(ret.getZ() + deltaZ);

      return ret;
   }

   private FramePose getCurrentHandPose(RobotSide robotSideToTest)
   {
      FramePose ret = new FramePose();
      drcBehaviorTestHelper.updateRobotModel();
      ret.setToZero(drcBehaviorTestHelper.getSDFFullRobotModel().getHandControlFrame(robotSideToTest));
      ret.changeFrame(ReferenceFrame.getWorldFrame());
      return ret;
   }

   private void assertCurrentHandPoseIsWithinThresholds(RobotSide robotSide, FramePose desiredPose)
   {
      FramePose currentPose = getCurrentHandPose(robotSide);
      assertPosesAreWithinThresholds(desiredPose, currentPose);
   }

   private void assertPosesAreWithinThresholds(FramePose desiredPose, FramePose actualPose)
   {
      assertPosesAreWithinThresholds(desiredPose, actualPose, POSITION_THRESHOLD, ORIENTATION_THRESHOLD);
   }

   private void assertPosesAreWithinThresholds(FramePose desiredPose, FramePose actualPose, double positionThreshold, double orientationThreshold)
   {
      double positionDistance = desiredPose.getPositionDistance(actualPose);
      double orientationDistance = desiredPose.getOrientationDistance(actualPose);

      if (DEBUG)
      {
         System.out.println("testSimpleHandPoseMove: positionDistance=" + positionDistance);
         System.out.println("testSimpleHandPoseMove: orientationDistance=" + orientationDistance);
      }

      assertEquals("Pose position error :" + positionDistance + " exceeds threshold: " + positionThreshold, 0.0, positionDistance, positionThreshold);
      assertEquals("Pose orientation error :" + orientationDistance + " exceeds threshold: " + orientationThreshold, 0.0, orientationDistance,
            orientationThreshold);
   }

   private void assertCurrentHandPoseIsWithinThresholds(RobotSide robotSide, double[] desiredArmPose)
   {
      double[] currentArmPose = getCurrentArmPose(robotSide);
      assertPosesAreWithinThresholds(desiredArmPose, currentArmPose, robotSide);
   }

   private void assertPosesAreWithinThresholds(double[] desiredArmPose, double[] actualArmPose, RobotSide robotSide)
   {
      for (int i = 0; i < numberOfArmJoints; i++)
      {
         ArmJointName armJointName = armJointNames[i];

         double q_desired = desiredArmPose[i];
         double q_actual = actualArmPose[i];
         double error = Math.abs(q_actual - q_desired);

         if (DEBUG)
         {
            SysoutTool.println(armJointName + " qDesired = " + q_desired + ".  qActual = " + q_actual + ".");
         }
         assertEquals(armJointName + " position error (" + Math.toDegrees(error) + " degrees) exceeds threshold of " + Math.toDegrees(JOINT_POSITION_THRESHOLD)
               + " degrees.", q_desired, q_actual, JOINT_POSITION_THRESHOLD);
      }
   }
}
