package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Random;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoPacketCommunicator;
import us.ihmc.communication.packets.manipulation.HandPoseListPacket;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseListBehavior;
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
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class DRCHandPoseListBehaviorTest implements MultiRobotTestInterface
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
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCHandPoseListBehaviorTest.class + " after class.");
   }

   private static final boolean DEBUG = false;

   private final double JOINT_POSITION_THRESHOLD = 0.007;
   private final double EXTRA_SIM_TIME_FOR_SETTLING = 2.0;

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

      KryoPacketCommunicator controllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10, "DRCControllerCommunicator");
      KryoPacketCommunicator networkObjectCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10, "DRCJunkyCommunicator");

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, networkObjectCommunicator, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, false, getRobotModel(), controllerCommunicator);

      FullRobotModel fullRobotModel = drcBehaviorTestHelper.getFullRobotModel();

      armJointNames = fullRobotModel.getRobotSpecificJointNames().getArmJointNames();
      numberOfArmJoints = armJointNames.length;

      for (int i = 0; i < numberOfArmJoints; i++)
      {
         armJointIndices.put(armJointNames[i], i);
      }
   }

   @AverageDuration
   @Test(timeout = 300000)
   public void testMoveOneRandomJoint90Deg() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      final HandPoseListBehavior handPoseListBehavior = new HandPoseListBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            drcBehaviorTestHelper.getYoTime());

      double swingTrajectoryTime = 2.0;
      int numberOfArmPoses = 2;
      RobotSide robotSide = RobotSide.LEFT;

      double[][] armPoses = createArmPosesInitializedToRobot(numberOfArmPoses, robotSide);

      int randomArmJointIndex = RandomTools.generateRandomInt(new Random(), 0, numberOfArmJoints - 1);
      ArmJointName randomArmJoint = armJointNames[randomArmJointIndex];
      int poseNumber = 1;
      setSingleJoint(armPoses, poseNumber, robotSide, randomArmJoint, Math.PI / 2, true);

      HandPoseListPacket handPoseListPacket = new HandPoseListPacket(robotSide, armPoses, swingTrajectoryTime);
      handPoseListBehavior.initialize();
      handPoseListBehavior.setInput(handPoseListPacket);
      assertTrue(handPoseListBehavior.hasInputBeenSet());

      success = drcBehaviorTestHelper
            .executeBehaviorSimulateAndBlockAndCatchExceptions(handPoseListBehavior, swingTrajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING);
      assertTrue(success);

      assertRobotAchievedFinalDesiredArmPose(armPoses, robotSide);
      assertTrue(success);
      assertTrue(handPoseListBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   @AverageDuration
   @Test(timeout = 300000)
   public void testWackyInflatableArmFlailingTubeManDance() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      SideDependentList<double[][]> armPosesLeftAndRightSide = new SideDependentList<double[][]>();
      ArrayList<BehaviorInterface> behaviors = new ArrayList<BehaviorInterface>();
      double swingTrajectoryTime = 20.0;
      int numberOfArmPoses = 3;

      for (RobotSide robotSide : RobotSide.values)
      {
         final HandPoseListBehavior handPoseListBehavior = new HandPoseListBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
               drcBehaviorTestHelper.getYoTime());

         double[][] armPoses = createRandomArmPoses(numberOfArmPoses, robotSide);
         HandPoseListPacket handPoseListPacket = new HandPoseListPacket(robotSide, armPoses, swingTrajectoryTime);
         handPoseListBehavior.initialize();
         handPoseListBehavior.setInput(handPoseListPacket);
         assertTrue(handPoseListBehavior.hasInputBeenSet());

         armPosesLeftAndRightSide.put(robotSide, armPoses);
         behaviors.add(handPoseListBehavior);
      }

      //TODO: Check that each pose executes properly, not just the last pose (tricky, unless motion is very very slow)
      
      for (int poseNumber = 0; poseNumber < numberOfArmPoses; poseNumber ++)
      {
         success = drcBehaviorTestHelper.executeBehaviorsSimulateAndBlockAndCatchExceptions(behaviors, swingTrajectoryTime / numberOfArmPoses);
         assertTrue(success);
         
         for (RobotSide robotSide : RobotSide.values)
         {
            assertRobotAchievedDesiredArmPose(armPosesLeftAndRightSide.get(robotSide), robotSide, poseNumber, 10.0 * JOINT_POSITION_THRESHOLD);           
         }
      }


      for (RobotSide robotSide : RobotSide.values)
      {
         assertRobotAchievedFinalDesiredArmPose(armPosesLeftAndRightSide.get(robotSide), robotSide);

         assertTrue(behaviors.get(robotSide.ordinal()).isDone());
      }

      assertTrue(success);

      BambooTools.reportTestFinishedMessage();
   }

   private void setSingleJoint(double[][] armPosesToPack, int poseNumber, RobotSide robotSide, ArmJointName armJointName, double desiredJointAngle,
         boolean clipDesiredQToJointLimits)
   {
      int armJointIndex = armJointIndices.get(armJointName);
      setSingleJoint(armPosesToPack, poseNumber, robotSide, armJointIndex, desiredJointAngle, clipDesiredQToJointLimits);
   }

   private void setSingleJoint(double[][] armPosesToPack, int poseNumber, RobotSide robotSide, int armJointIndex, double desiredJointAngle,
         boolean clipDesiredQToJointLimits)
   {
      double q = desiredJointAngle;
      ArmJointName armJointName = armJointNames[armJointIndex];

      if (clipDesiredQToJointLimits)
      {
         q = clipDesiredToJointLimits(robotSide, armJointName, desiredJointAngle);
      }

      armPosesToPack[armJointIndex][poseNumber] = q;
   }

   private double clipDesiredToJointLimits(RobotSide robotSide, ArmJointName armJointName, double desiredJointAngle)
   {
      FullRobotModel fullRobotModel = drcBehaviorTestHelper.getFullRobotModel();
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

   private double[] getCurrentArmPose(RobotSide robotSide)
   {
      double[] armPose = new double[numberOfArmJoints];

      for (int jointNum = 0; jointNum < numberOfArmJoints; jointNum++)
      {
         ArmJointName jointName = armJointNames[jointNum];
         double currentAngle = drcBehaviorTestHelper.getFullRobotModel().getArmJoint(robotSide, jointName).getQ();
         armPose[jointNum] = currentAngle;
      }

      return armPose;
   }

   private double[] getDesiredArmPose(double[][] armPoses, int poseNumber)
   {
      double[] desiredPose = new double[numberOfArmJoints];

      for (int jointNum = 0; jointNum < numberOfArmJoints; jointNum++)
      {
         desiredPose[jointNum] = armPoses[jointNum][poseNumber];
      }

      return desiredPose;
   }

   private double[] getFinalDesiredArmPose(double[][] armPoses)
   {
      int lastPoseIndex = armPoses[0].length - 1;

      double[] desiredPose = getDesiredArmPose(armPoses, lastPoseIndex);

      return desiredPose;
   }

   private double[][] createRandomArmPoses(int numberOfPoses, RobotSide robotSide)
   {
      double[][] armPoses = new double[numberOfArmJoints][numberOfPoses];

      for (int poseNumber = 0; poseNumber < numberOfPoses; poseNumber++)
      {
         double[] desiredArmPose = createRandomArmPose(robotSide);

         setArmPose(armPoses, poseNumber, desiredArmPose);
      }

      return armPoses;
   }

   private double[] createRandomArmPose(RobotSide robotSide)
   {
      double[] armPose = new double[numberOfArmJoints];

      for (int jointNum = 0; jointNum < numberOfArmJoints; jointNum++)
      {
         double qDesired = clipDesiredToJointLimits(robotSide, armJointNames[jointNum], RandomTools.generateRandomDouble(new Random(), 1.5));
         armPose[jointNum] = qDesired;
      }

      return armPose;
   }

   private double[][] createArmPosesInitializedToRobot(int numberOfPoses, RobotSide robotSide)
   {
      double[][] armPoses = new double[numberOfArmJoints][numberOfPoses];

      double[] currentArmPose = getCurrentArmPose(robotSide);

      for (int i = 0; i < numberOfPoses; i++)
      {
         setArmPose(armPoses, i, currentArmPose);
      }

      return armPoses;
   }

   private void setArmPose(double[][] armPosesToPack, int poseNumber, double[] armPose)
   {
      for (ArmJointName jointName : armJointNames)
      {
         int jointNum = armJointIndices.get(jointName);
         armPosesToPack[jointNum][poseNumber] = armPose[jointNum];
      }
   }

   private void assertRobotAchievedDesiredArmPose(double[][] armPoses, RobotSide robotSide, int poseNumber, double jointPositionThreshold)
   {
      double[] desiredArmPose = getDesiredArmPose(armPoses, poseNumber);
      double[] actualArmPose = getCurrentArmPose(robotSide);

      for (int i = 0; i < numberOfArmJoints; i++)
      {
         ArmJointName armJointName = armJointNames[i];

         double q_desired = desiredArmPose[i];
         double q_actual = actualArmPose[i];

         if (DEBUG)
         {
            SysoutTool.println(armJointName + " qDesired = " + q_desired + ".  qActual = " + q_actual + ".");
         }

         assertEquals(q_desired, q_actual, jointPositionThreshold);
      }
   }
   
   private void assertRobotAchievedFinalDesiredArmPose(double[][] armPoses, RobotSide robotSide)
   {
      int lastPoseIndex = armPoses[0].length - 1;
      assertRobotAchievedDesiredArmPose(armPoses, robotSide, lastPoseIndex, JOINT_POSITION_THRESHOLD);
   }
}
