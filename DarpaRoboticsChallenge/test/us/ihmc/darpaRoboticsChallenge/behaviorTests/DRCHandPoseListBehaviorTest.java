package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.SdfLoader.partNames.ArmJointName;
import us.ihmc.communication.packets.manipulation.HandPoseListPacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.DataType;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseListBehavior;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.dataProcessors.RobotAllJointsDataChecker;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.utilities.io.printing.PrintTools;
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
      
      

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @AfterClass
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCHandPoseListBehaviorTest.class + " after class.");
   }

   private static final boolean DEBUG = false;

   private final double maximumJointVelocity = 1.5;
   private final double maximumJointAcceleration = 15.0;
   private final double JOINT_POSITION_THRESHOLD_AFTER_SETTLING = Math.toRadians(5.0);
   private final double JOINT_POSITION_THRESHOLD_BETWEEN_POSES = 25.0 * JOINT_POSITION_THRESHOLD_AFTER_SETTLING;
   private final double EXTRA_SIM_TIME_FOR_SETTLING = 1.0;

   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   private ArmJointName[] armJointNames;
   private int numberOfArmJoints;
   private final LinkedHashMap<ArmJointName, Integer> armJointIndices = new LinkedHashMap<ArmJointName, Integer>();

   @Before
   public void setUp()
   {
      DRCDemo01NavigationEnvironment testEnvironment = new DRCDemo01NavigationEnvironment();

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel());

      FullRobotModel fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();

      armJointNames = fullRobotModel.getRobotSpecificJointNames().getArmJointNames();
      numberOfArmJoints = armJointNames.length;

      for (int i = 0; i < numberOfArmJoints; i++)
      {
         armJointIndices.put(armJointNames[i], i);
      }
   }

	@DeployableTestMethod(estimatedDuration = 93.3)
   @Test(timeout = 470000)
   public void testRandomJointSpaceMoveAndTaskSpaceMoveBackToHome() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      PrintTools.debug(this, "Initializing Simulation");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      
      final HandPoseListBehavior handPoseListBehavior = new HandPoseListBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            drcBehaviorTestHelper.getYoTime());

      double swingTrajectoryTime = 3.0;
      int numberOfArmPoses = 2;
      RobotSide robotSide = RobotSide.LEFT;
      FramePose initialHandPose = getCurrentHandPose(robotSide);

      double[] currentArmPose = getCurrentArmPose(robotSide);
      double[] randomArmPose = createRandomArmPose(robotSide);
      double[] intermediateArmPose = new double[numberOfArmJoints];
      for (int i=0; i<numberOfArmJoints; i++)
      {
         intermediateArmPose[i] = 0.5*randomArmPose[i] + 0.5*currentArmPose[i]; 
      }
      
      double[][] armPoses = new double[numberOfArmJoints][numberOfArmPoses];
      setArmPose(armPoses, 0, intermediateArmPose);
      setArmPose(armPoses, 1, randomArmPose);
      
      PrintTools.debug(this, "Initializing Joint Space HandPoseListBehavior");
      HandPoseListPacket handPoseListPacket = new HandPoseListPacket(robotSide, armPoses, swingTrajectoryTime);

      handPoseListBehavior.initialize();
      handPoseListBehavior.setInput(handPoseListPacket);
      assertTrue(handPoseListBehavior.hasInputBeenSet());

      PrintTools.debug(this, "Executing Joint Space HandPoseListBehavior");
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(handPoseListBehavior);
      assertTrue(success);
      PrintTools.debug(this, "Joint Space HandPoseListBehavior should be done");

      assertRobotAchievedFinalDesiredArmPose(armPoses, robotSide);
      assertTrue(success);
      assertTrue(handPoseListBehavior.isDone());

      FramePose finalHandPose = getCurrentHandPose(robotSide);
      
      PrintTools.debug(this, "Pausing behavior for half a second");
      success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);
      
      int numberOfTaskSpacePoses = 5;
      
      FramePose[] desiredPosesInWorld = new FramePose[numberOfTaskSpacePoses];
      desiredPosesInWorld[0] = new FramePose();
      desiredPosesInWorld[0].interpolate(initialHandPose, finalHandPose, 0.8);
      desiredPosesInWorld[1] = new FramePose();
      desiredPosesInWorld[1].interpolate(initialHandPose, finalHandPose, 0.6);
      desiredPosesInWorld[2] = new FramePose();
      desiredPosesInWorld[2].interpolate(initialHandPose, finalHandPose, 0.4);
      desiredPosesInWorld[3] = new FramePose();
      desiredPosesInWorld[3].interpolate(initialHandPose, finalHandPose, 0.2);
      desiredPosesInWorld[4] = new FramePose();
      desiredPosesInWorld[4].interpolate(initialHandPose, finalHandPose, 0.0);

      assertHandPosesAreWithinThresholds(initialHandPose, desiredPosesInWorld[desiredPosesInWorld.length-1], 0.0, 0.0);
      
      Point3d[] positions = new Point3d[numberOfTaskSpacePoses];
      Quat4d[] orientations = new Quat4d[numberOfTaskSpacePoses];
      
      for (int i=0; i<numberOfTaskSpacePoses; i++)
      {
         positions[i] = new Point3d();
         orientations[i] = new Quat4d();
         desiredPosesInWorld[i].getPose(positions[i], orientations[i]);
      }
      
      PrintTools.debug(this, "Initializing HandPoseListBehavior using Task-Space Hand Poses");
      handPoseListPacket = new HandPoseListPacket(robotSide, Frame.WORLD, DataType.HAND_POSE, positions, orientations, false, swingTrajectoryTime, null);

      handPoseListBehavior.initialize();
      handPoseListBehavior.setInput(handPoseListPacket);
      assertTrue(handPoseListBehavior.hasInputBeenSet());

      PrintTools.debug(this, "Executing Task Space HandPoseListBehavior");
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(handPoseListBehavior, swingTrajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING);
      assertTrue(success);
      PrintTools.debug(this, "Task Space HandPoseListBehavior should be done");

      assertHandPosesAreWithinThresholds(initialHandPose, getCurrentHandPose(robotSide), 8.0 * DRCHandPoseBehaviorTest.POSITION_THRESHOLD, 8.0 * DRCHandPoseBehaviorTest.ORIENTATION_THRESHOLD);
      assertTrue(success);
      assertTrue(handPoseListBehavior.isDone());
      
      assertAllRobotJointsMovedSmoothlyWithinKinematicAndDynamicLimits();

      BambooTools.reportTestFinishedMessage();
   }
   
	@DeployableTestMethod(estimatedDuration = 52.5)
   @Test(timeout = 260000)
   public void testMoveOneRandomJoint20Deg() throws SimulationExceededMaximumTimeException
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
      double currentJointAngle = drcBehaviorTestHelper.getSDFFullRobotModel().getArmJoint(robotSide, randomArmJoint).getQ();
      int poseNumber = 1;
      setSingleJoint(armPoses, poseNumber, robotSide, randomArmJoint, currentJointAngle + Math.toRadians(20.0), true);

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
      
      assertAllRobotJointsMovedSmoothlyWithinKinematicAndDynamicLimits();

      BambooTools.reportTestFinishedMessage();
   }

   
	@DeployableTestMethod(estimatedDuration = 53.1)
   @Test(timeout = 270000)
   public void testWristRoll() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      final HandPoseListBehavior handPoseListBehavior = new HandPoseListBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            drcBehaviorTestHelper.getYoTime());

      double swingTrajectoryTime = 2.0;
      int numberOfArmPoses = 2;
      RobotSide robotSide = RobotSide.LEFT;

      double[][] armPoses = createArmPosesInitializedToRobot(numberOfArmPoses, robotSide);

      ArmJointName randomArmJoint = ArmJointName.WRIST_ROLL;
      double currentJointAngle = drcBehaviorTestHelper.getSDFFullRobotModel().getArmJoint(robotSide, randomArmJoint).getQ();
      int poseNumber = 1;
      setSingleJoint(armPoses, poseNumber, robotSide, randomArmJoint, currentJointAngle + Math.toRadians(20.0), true);

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
      
      assertAllRobotJointsMovedSmoothlyWithinKinematicAndDynamicLimits();

      BambooTools.reportTestFinishedMessage();
   }
      
	@DeployableTestMethod(estimatedDuration = 82.1)
   @Test(timeout = 410000)
   public void testMultipleArmPosesOnBothArms() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      SideDependentList<double[][]> armPosesLeftAndRightSide = new SideDependentList<double[][]>();
      ArrayList<BehaviorInterface> behaviors = new ArrayList<BehaviorInterface>();
      double swingTrajectoryTime = 5.0;
      int numberOfArmPoses = 4;

      for (RobotSide robotSide : RobotSide.values)
      {
         final HandPoseListBehavior handPoseListBehavior = new HandPoseListBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
               drcBehaviorTestHelper.getYoTime());

         double[][] armPoses = createRandomArmPoseWithWayPointsFromCurrentArmPose(numberOfArmPoses, robotSide);
         HandPoseListPacket handPoseListPacket = new HandPoseListPacket(robotSide, armPoses, swingTrajectoryTime);
         handPoseListBehavior.initialize();
         handPoseListBehavior.setInput(handPoseListPacket);
         assertTrue(handPoseListBehavior.hasInputBeenSet());

         armPosesLeftAndRightSide.put(robotSide, armPoses);
         behaviors.add(handPoseListBehavior);
      }

      for (int poseNumber = 0; poseNumber < numberOfArmPoses; poseNumber++)
      {
         success = drcBehaviorTestHelper.executeBehaviorsSimulateAndBlockAndCatchExceptions(behaviors, swingTrajectoryTime / numberOfArmPoses);
         assertTrue(success);

         for (RobotSide robotSide : RobotSide.values)
         {
            assertRobotAchievedDesiredArmPose(armPosesLeftAndRightSide.get(robotSide), robotSide, poseNumber, JOINT_POSITION_THRESHOLD_BETWEEN_POSES);
         }
      }

      success = drcBehaviorTestHelper.executeBehaviorsSimulateAndBlockAndCatchExceptions(behaviors, EXTRA_SIM_TIME_FOR_SETTLING);
      assertTrue(success);
      
      for (RobotSide robotSide : RobotSide.values)
      {
         assertRobotAchievedFinalDesiredArmPose(armPosesLeftAndRightSide.get(robotSide), robotSide);
         assertTrue(behaviors.get(robotSide.ordinal()).isDone());
      }

      assertTrue(success);
      
      assertAllRobotJointsMovedSmoothlyWithinKinematicAndDynamicLimits();

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
      FullHumanoidRobotModel fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();
      double q;
      double qMin = fullRobotModel.getArmJoint(robotSide, armJointName).getJointLimitLower();
      double qMax = fullRobotModel.getArmJoint(robotSide, armJointName).getJointLimitUpper();

      if (qMin > qMax)
      {
         double temp = qMax;
         qMax = qMin;
         qMin = temp;
      }
      
      double qTotal = Math.abs(qMax-qMin);
      double safetyBuffer = 0.02*qTotal;

      q = MathTools.clipToMinMax(desiredJointAngle, qMin - safetyBuffer, qMax + safetyBuffer);
      return q;
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

   private double[][] createRandomArmPoseWithWayPointsFromCurrentArmPose(int numberOfPoses, RobotSide robotSide)
   {
      double[] currentArmPose = getCurrentArmPose(robotSide);
      double[] randomArmPose = createRandomArmPose(robotSide);

      double[][] armPoses = new double[numberOfArmJoints][numberOfPoses];
      double[] intermediateArmPose = new double[numberOfArmJoints];

      for (int poseNumber = 0; poseNumber < numberOfPoses; poseNumber++)
      {
         double alpha = ((double) poseNumber) / (numberOfPoses-1);

         for (int i = 0; i < numberOfArmJoints; i++)
         {
            intermediateArmPose[i] = (alpha) * randomArmPose[i] + (1.0 - alpha) * currentArmPose[i];
         }

         setArmPose(armPoses, poseNumber, intermediateArmPose);
      }

      return armPoses;
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

      LinkedHashMap<Double, ArmJointName> jointAngleErrors = new LinkedHashMap<Double, ArmJointName>(numberOfArmJoints);
      double maxJointAngleError = 0.0;
      
      if (DEBUG)
         System.out.println("\n");
      
      for (int i = 0; i < numberOfArmJoints; i++)
      {
         ArmJointName armJointName = armJointNames[i];

         double q_desired = desiredArmPose[i];
         double q_actual = actualArmPose[i];
         double error = Math.abs(q_actual - q_desired);
         jointAngleErrors.put(error, armJointName);
         
         assertEquals(armJointName + " position error (" + Math.toDegrees(error) + " degrees) exceeds threshold of " + Math.toDegrees(jointPositionThreshold) + " degrees.", q_desired, q_actual, jointPositionThreshold);
      
         if (DEBUG)
            PrintTools.debug(this, armJointName + " qDesired = " + q_desired + ".  qActual = " + q_actual + ".  Error = " + error);
         
         if (error > maxJointAngleError)
            maxJointAngleError = error;
      }

      if (DEBUG)
         PrintTools.debug(this, "Maximum Position Error: " + Math.toDegrees(maxJointAngleError) + " degrees in " + jointAngleErrors.get(maxJointAngleError) );
   }

   private void assertRobotAchievedFinalDesiredArmPose(double[][] armPoses, RobotSide robotSide)
   {
      int lastPoseIndex = armPoses[0].length - 1;
      assertRobotAchievedDesiredArmPose(armPoses, robotSide, lastPoseIndex, JOINT_POSITION_THRESHOLD_AFTER_SETTLING);
   }
   
   private FramePose getCurrentHandPose(RobotSide robotSideToTest)
   {
      FramePose ret = new FramePose();
      drcBehaviorTestHelper.updateRobotModel();
      ret.setToZero(drcBehaviorTestHelper.getSDFFullRobotModel().getHandControlFrame(robotSideToTest));
      ret.changeFrame(ReferenceFrame.getWorldFrame());
      return ret;
   }
   
   private void assertHandPosesAreWithinThresholds(FramePose desiredPose, FramePose actualPose, double positionThreshold, double orientationThreshold)
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
   
   private void assertAllRobotJointsMovedSmoothlyWithinKinematicAndDynamicLimits()
   {
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();
      RobotAllJointsDataChecker checker = new RobotAllJointsDataChecker(scs, drcBehaviorTestHelper.getRobot());
      
      checker.setMaximumDerivativeForAllJoints(10.2 * maximumJointVelocity);
      checker.setMaximumSecondDerivativeForAllJoints(1.2 * maximumJointAcceleration);
      
      checker.cropInitialSimPoints(10);

      scs.applyDataProcessingFunction(checker);
      
      assertFalse(checker.getDerivativeCompError(), checker.hasDerivativeComparisonErrorOccurredAnyJoint());
      assertFalse(checker.getMaxDerivativeExceededError(), checker.hasMaxDerivativeExeededAnyJoint());
//      assertFalse(checker.getMaxSecondDerivativeExceededError(), checker.hasMaxSecondDerivativeExeededAnyJoint());
      assertFalse(checker.getMaxValueExceededError(), checker.hasMaxValueExeededAnyJoint());
      assertFalse(checker.getMinValueExceededError(), checker.hasMinValueExeededAnyJoint());
   }
}
