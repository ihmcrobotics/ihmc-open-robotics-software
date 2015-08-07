package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.LinkedHashMap;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.util.PacketControllerTools;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyInverseKinematicBehavior;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.humanoidRobotics.model.FullRobotModel;
import us.ihmc.humanoidRobotics.partNames.ArmJointName;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ControlledDoF;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class DRCWholeBodyInverseKinematicBehaviorTest implements MultiRobotTestInterface
{
   private final boolean DEBUG = false;

   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   private void showMemoryUsageBeforeTest()
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
         drcBehaviorTestHelper.destroySimulation();
         drcBehaviorTestHelper = null;
      }

      GlobalTimer.clearTimers();

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private final double EXTRA_SIM_TIME_FOR_SETTLING = 1.5;

   private final double POSITION_ERROR_MARGIN = 0.03;
   private final double ANGLE_ERROR_MARGIN = 0.1;

   private final DRCDemo01NavigationEnvironment testEnvironment = new DRCDemo01NavigationEnvironment();

   private DoubleYoVariable yoTime;

   private SDFFullRobotModel fullRobotModel;
   private WholeBodyControllerParameters wholeBodyControllerParameters;

   private ArmJointName[] armJointNames;
   private int numberOfArmJoints;
   private LinkedHashMap<ArmJointName, Integer> armJointIndices = new LinkedHashMap<ArmJointName, Integer>();
   
   private final double DELTA_YAW = 0.2;//0.05;
   private final double DELTA_PITCH = 0.5;//0.08;
   private final double DELTA_ROLL = Math.PI;

   @Before
   public void setUp()
   {
      showMemoryUsageBeforeTest();

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel());

      yoTime = drcBehaviorTestHelper.getRobot().getYoTime();

      wholeBodyControllerParameters = getRobotModel();
      fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();
      
      armJointNames = drcBehaviorTestHelper.getSDFFullRobotModel().getRobotSpecificJointNames().getArmJointNames();
      numberOfArmJoints = armJointNames.length;

      for (int i = 0; i < numberOfArmJoints; i++)
      {
         armJointIndices.put(armJointNames[i], i);
      }
   }

   @EstimatedDuration(duration = 90.0)
   @Test(timeout = 300000)
   public void testWholeBodyInverseKinematicsMoveToPoseAcheivedInJointSpace() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      PrintTools.debug(this, "Initializing Sim");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      PrintTools.debug(this, "Setting Hand Pose Behavior Input in Joint Space");
      RobotSide robotSide = RobotSide.LEFT;
      double trajectoryTime = 2.0;
      double[] desiredJointSpaceHandPose = createRandomArmPose(robotSide);
      
      final HandPoseBehavior handPoseBehavior = new HandPoseBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(), drcBehaviorTestHelper.getYoTime());
      handPoseBehavior.initialize();
      HandPosePacket jointSpaceHandPosePacket = new HandPosePacket(robotSide, trajectoryTime, desiredJointSpaceHandPose);
      handPoseBehavior.setInput(jointSpaceHandPosePacket);

      PrintTools.debug(this, "Starting Joint Space Hand Pose Behavior");
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(handPoseBehavior);
      PrintTools.debug(this, "Joint Space Hand Pose Behavior Should Be Done");

      assertTrue(success);
      assertCurrentArmPoseIsWithinThresholds(robotSide, desiredJointSpaceHandPose);
      assertTrue(handPoseBehavior.isDone());

      PrintTools.debug(this, "Recording HandPose Acheived in Joint Space");
      FramePose handPoseAcheivedInJointSpace = getCurrentHandPose(robotSide);

      HandPosePacket goToHomePacket = PacketControllerTools.createGoToHomeHandPosePacket(robotSide, trajectoryTime);
      handPoseBehavior.initialize();
      handPoseBehavior.setInput(goToHomePacket);
      PrintTools.debug(this, "Moving arm back to home pose");
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(handPoseBehavior);
      PrintTools.debug(this, "Arm should be back to home pose");
      assertTrue(success);
      
      
      PrintTools.debug(this, "Setting Hand Pose Behavior Input in Task Space");
      RigidBodyTransform handPoseTargetTransform = new RigidBodyTransform();
      handPoseAcheivedInJointSpace.getPose(handPoseTargetTransform);
      handPoseBehavior.initialize();
      handPoseBehavior.setInput(Frame.WORLD, handPoseTargetTransform, robotSide, trajectoryTime);
      assertTrue(handPoseBehavior.hasInputBeenSet());

      PrintTools.debug(this, "Starting Task Space Hand Pose Behavior");
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(handPoseBehavior);
      PrintTools.debug(this, "Task Space Hand Pose Behavior Should Be Done");

      assertTrue(success);
      assertCurrentHandPoseIsWithinThresholds(robotSide, handPoseAcheivedInJointSpace);
      assertTrue(handPoseBehavior.isDone());

      FramePose finalTaskSpaceHandPose = getCurrentHandPose(robotSide);
      double handPoseBehaviorPositionError = finalTaskSpaceHandPose.getPositionDistance(handPoseAcheivedInJointSpace);
      double handPoseBehaviorOrientationError = finalTaskSpaceHandPose.getOrientationDistance(handPoseAcheivedInJointSpace);
      
      PrintTools.debug(this, "Initializing Whole Body Inverse Kinematic Behavior");
      final WholeBodyInverseKinematicBehavior wholeBodyIKBehavior = new WholeBodyInverseKinematicBehavior(
            drcBehaviorTestHelper.getBehaviorCommunicationBridge(), wholeBodyControllerParameters, fullRobotModel, yoTime);
      wholeBodyIKBehavior.initialize();
      wholeBodyIKBehavior.setPositionAndOrientationErrorTolerance(handPoseBehaviorPositionError, handPoseBehaviorOrientationError);
         
      wholeBodyIKBehavior.setInputs(robotSide, handPoseAcheivedInJointSpace, trajectoryTime, 5, ControlledDoF.DOF_3P3R);
      assertTrue(wholeBodyIKBehavior.hasInputBeenSet());

      wholeBodyIKBehavior.computeSolution();
      assertTrue(wholeBodyIKBehavior.hasSolutionBeenFound());
      drcBehaviorTestHelper.executeBehaviorUntilDone(wholeBodyIKBehavior);

      assertTrue(wholeBodyIKBehavior.isDone());
      assertPosesAreWithinThresholds(handPoseAcheivedInJointSpace, getCurrentHandPose(robotSide), 2.0 * handPoseBehaviorPositionError, 2.0 * handPoseBehaviorOrientationError);

      BambooTools.reportTestFinishedMessage();
   }

   @EstimatedDuration(duration = 90.0)
   @Test(timeout = 300000)
   public void testRandomRightHandPose() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      assertTrue(drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(EXTRA_SIM_TIME_FOR_SETTLING));

      drcBehaviorTestHelper.updateRobotModel();

      final WholeBodyInverseKinematicBehavior wholeBodyIKBehavior = new WholeBodyInverseKinematicBehavior(
            drcBehaviorTestHelper.getBehaviorCommunicationBridge(), wholeBodyControllerParameters, fullRobotModel, yoTime);

      wholeBodyIKBehavior.initialize();

      wholeBodyIKBehavior.setPositionAndOrientationErrorTolerance(POSITION_ERROR_MARGIN, ANGLE_ERROR_MARGIN);

      RobotSide side = RobotSide.RIGHT;

      double trajectoryDuration = 4.0;

      double[] xyzMin = new double[3];
      double[] xyzMax = new double[3];
      double[] yawPitchRollMin = new double[3];
      double[] yawPitchRollMax = new double[3];

      generatePositionAndAngleLimits(side, xyzMin, xyzMax, yawPitchRollMin, yawPitchRollMax);

      Random random = new Random();
      FramePose desiredHandPose = FramePose.generateRandomFramePose(random, worldFrame, xyzMin, xyzMax, yawPitchRollMin, yawPitchRollMax);

      if (DEBUG)
      {
         System.out.println("desired hand Pose");
         System.out.println(desiredHandPose);
         System.out.println("  ");
      }
      wholeBodyIKBehavior.setInputs(side, desiredHandPose, trajectoryDuration, 5, ControlledDoF.DOF_3P3R);
      assertTrue(wholeBodyIKBehavior.hasInputBeenSet());

      wholeBodyIKBehavior.computeSolution();
      assertTrue(wholeBodyIKBehavior.hasSolutionBeenFound());
      drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(wholeBodyIKBehavior, trajectoryDuration);

      assertTrue(wholeBodyIKBehavior.isDone());

      //Right wrist Position
      ReferenceFrame rightHandReferenceFrame = fullRobotModel.getHandControlFrame(side);
      FramePose rightHandPose = new FramePose(rightHandReferenceFrame);
      rightHandPose.changeFrame(worldFrame);
      if (DEBUG)
      {
         System.out.println("!!!!!!!!!!!!!!!!!!! MOVEMENT HAPPENED !!!!!!!!!!!!!!!!!!!");
         System.out.println("desired right hand pose");
         System.out.println(desiredHandPose);
         System.out.println("    ");

         System.out.println("right hand pose");
         System.out.println(rightHandPose);
      }
      assertPosesAreWithinThresholds(desiredHandPose, rightHandPose, POSITION_ERROR_MARGIN, ANGLE_ERROR_MARGIN);

      BambooTools.reportTestFinishedMessage();
   }

   private void generatePositionAndAngleLimits(RobotSide side, double[] xyzMin, double[] xyzMax, double[] yawPitchRollMin, double[] yawPitchRollMax)
   {
      xyzMin[0] = 0.39;
      xyzMax[0] = 0.39 + 0.1;

      xyzMin[1] = side.negateIfRightSide(0.34);
      xyzMax[1] = side.negateIfRightSide(0.34);

      xyzMin[2] = 0.74 - 0.1;
      xyzMax[2] = 0.74 + 0.1;

      yawPitchRollMin[0] = 0.125 - DELTA_YAW;
      yawPitchRollMax[0] = 0.125 + DELTA_YAW;

      yawPitchRollMin[1] = 0.543 - DELTA_PITCH;
      yawPitchRollMax[1] = 0.543 + DELTA_PITCH;

      yawPitchRollMin[2] = side.negateIfRightSide(0.295);
      yawPitchRollMax[2] = side.negateIfRightSide(0.295 + DELTA_ROLL);
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
      assertPosesAreWithinThresholds(desiredPose, actualPose, POSITION_ERROR_MARGIN, ANGLE_ERROR_MARGIN);
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
   
   private void assertCurrentArmPoseIsWithinThresholds(RobotSide robotSide, double[] desiredArmPose)
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
            PrintTools.debug(this, armJointName + " qDesired = " + q_desired + ".  qActual = " + q_actual + ".");
         }
         assertEquals(armJointName + " position error (" + Math.toDegrees(error) + " degrees) exceeds threshold of " + Math.toDegrees(DRCHandPoseBehaviorTest.JOINT_POSITION_THRESHOLD)
               + " degrees.", q_desired, q_actual, DRCHandPoseBehaviorTest.JOINT_POSITION_THRESHOLD);
      }
   }
}
