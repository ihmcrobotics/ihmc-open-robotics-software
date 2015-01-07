package us.ihmc.darpaRoboticsChallenge;

//~--- non-JDK imports --------------------------------------------------------

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.Random;

import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.kinematics.DdoglegInverseKinematicsCalculator;
import us.ihmc.utilities.kinematics.InverseKinematicsCalculator;
import us.ihmc.utilities.kinematics.KinematicSolver;
import us.ihmc.utilities.kinematics.NumericalInverseKinematicsCalculator;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.test.JUnitTools;

public abstract class NumericalInverseKinematicsCalculatorWithRobotTest implements MultiRobotTestInterface
{
   private static final long seed = 1391092L;
   private static final Random randomNumberGenerator = new Random(seed);
   private static final ArrayList<Double> shoulderRollLimits = new ArrayList<Double>();
   private static final ArrayList<Double> elbowRollLimits = new ArrayList<Double>();
   private static final ArrayList<Double> wristRollLimits = new ArrayList<Double>();
   private static final ArrayList<Double> shoulderYawLimits = new ArrayList<Double>();
   private static final ArrayList<Double> elbowPitchLimits = new ArrayList<Double>();
   private static final ArrayList<Double> wristPitchLimits = new ArrayList<Double>();
   private final EnumMap<JointNames, OneDoFJoint> oneDoFJoints = new EnumMap<JointNames, OneDoFJoint>(JointNames.class);
   private final EnumMap<JointNames, ArrayList<Double>> jointLimits = new EnumMap<JointNames, ArrayList<Double>>(JointNames.class);
   private final EnumMap<JointNames, Double> jointAngles = new EnumMap<JointNames, Double>(JointNames.class);
   private final EnumMap<JointNames, Double> jointAnglesInitial = new EnumMap<JointNames, Double>(JointNames.class);
   private boolean successfulSolve = true;
   private final ArrayList<Long> solvingTime = new ArrayList<Long>();
   private final ArrayList<Integer> numbIteration = new ArrayList<Integer>();
   private final SDFFullRobotModel fullRobotModel;
   private final ReferenceFrame worldFrame;
   private final double tolerance, maxStepSize, minRandomSearchScalar, maxRandomSearchScalar;
   private int maxIterations;
   private GeometricJacobian leftHandJacobian;
   private InverseKinematicsCalculator inverseKinematicsCalculator;
   private double bruteForceResolution;
   
   private enum InverseKinematicsSolver
   {
      TWAN_SOLVER, PETER_SOLVER, MAARTEN_SOLVER;
   }

   private enum JointNames
   {
      SHOULDER_YAW, SHOULDER_ROLL, ELBOW_PITCH, ELBOW_ROLL, WRIST_PITCH, WRIST_ROLL;
   }

   ;;

   public NumericalInverseKinematicsCalculatorWithRobotTest()
   {
      InverseKinematicsSolver inverseKinameticSolverToUse = InverseKinematicsSolver.PETER_SOLVER;
      fullRobotModel = getRobotModel().createFullRobotModel();
      
      worldFrame = ReferenceFrame.getWorldFrame();
      tolerance = 1e-5;
      maxStepSize = 1.0;
      minRandomSearchScalar = -0.05;
      maxRandomSearchScalar = 1.0;
      maxIterations = 140000;

      populateEnumMaps();

      leftHandJacobian = new GeometricJacobian(fullRobotModel.getChest(), fullRobotModel.getHand(RobotSide.LEFT), fullRobotModel.getHand(RobotSide.LEFT)
            .getBodyFixedFrame());

      switch (inverseKinameticSolverToUse)
      {
      case PETER_SOLVER:
         inverseKinematicsCalculator = new DdoglegInverseKinematicsCalculator(leftHandJacobian, 1, maxIterations, true, 0.02, 0.02, 0.02);
         break;

      case TWAN_SOLVER:
         inverseKinematicsCalculator = new NumericalInverseKinematicsCalculator(leftHandJacobian, tolerance, maxIterations, maxStepSize, minRandomSearchScalar,
               maxRandomSearchScalar);
         break;
      case MAARTEN_SOLVER:
         inverseKinematicsCalculator = new KinematicSolver(leftHandJacobian, tolerance, maxIterations);
      }
   }

   @Test(timeout=300000)
   public void generateRandomFeasibleRobotPoses()
   {
      int nTests = 10;

      for (int i = 0; i < nTests; i++)
      {
         System.out.println("iteration number " + i);
         randomArmPoseWithForwardKinematics();
         FramePoint handEndEffectorPositionFK = getHandEndEffectorPosition();
         FrameOrientation handEndEffectorOrientationFK = getHandEndEffectorOrientation();

         armPoseWithInverseKinematics(handEndEffectorOrientationFK, handEndEffectorPositionFK);

         FramePoint handEndEffectorPositionIK = getHandEndEffectorPosition();
         FrameOrientation handEndEffectorOrientationIK = getHandEndEffectorOrientation();

         JUnitTools.assertFramePointEquals(handEndEffectorPositionFK, handEndEffectorPositionIK, 0.25);
         JUnitTools.assertFrameOrientationEquals(handEndEffectorOrientationFK, handEndEffectorOrientationIK, 0.2);
      }

      System.out.println("Average Solving Time: " + getAvarageArray(solvingTime) + " ms");
      System.out.println("Maximal Solving Time: " + getMaximumArray(solvingTime) + " ms");
   }

   private boolean checkIfPotionsIsEqual(FramePoint pos1, FramePoint pos2, double d)
   {
      boolean equal = true;
      if (Math.abs(pos1.getX() - pos2.getX()) > d)
         equal = false;
      if (Math.abs(pos1.getY() - pos2.getY()) > d)
         equal = false;
      if (Math.abs(pos1.getZ() - pos2.getZ()) > d)
         equal = false;
      return equal;
   }

   private boolean checkIfRotationisEqual(FrameOrientation orien1, FrameOrientation orien2, double d)
   {
      boolean equal = true;
      for (int i = 0; i < orien1.getYawPitchRoll().length; i++)
      {
         if (Math.abs(orien1.getYawPitchRoll()[i] - orien2.getYawPitchRoll()[i]) > d)
            equal = false;

      }
      return equal;
   }

   @Test(timeout=300000)
   public void generateRandomDoubleInRangeTest()
   {
      double min = -randomNumberGenerator.nextDouble();
      double max = randomNumberGenerator.nextDouble();

      for (int i = 0; i < 100000; i++)
      {
         double randomNumber = generateRandomDoubleInRange(max, min);

         assertTrue((randomNumber <= max) && (randomNumber >= min));
      }
   }

   private void armPoseWithInverseKinematics(FrameOrientation desiredOrientation, FramePoint desiredPosition)
   {
      FramePose handPose = new FramePose(worldFrame);

      handPose.setOrientation(desiredOrientation);
      handPose.setPosition(desiredPosition);
      handPose.changeFrame(fullRobotModel.getChest().getBodyFixedFrame());

      RigidBodyTransform transform = new RigidBodyTransform();

      handPose.getPose(transform);
      randomArmPoseWithForwardKinematics();

      long start = System.nanoTime();

      solve(transform);

      long end = System.nanoTime();

      solvingTime.add((long) ((end - start) / 10E6));
   }

   private void solve(RigidBodyTransform transform)
   {
      successfulSolve = inverseKinematicsCalculator.solve(transform);

//      if (successfulSolve)
//      {
//         numbIteration.add(inverseKinematicsCalculator.getNumberOfIterations());
//
//         OneDoFJoint[] oneDoFJoints = ScrewTools.filterJoints(leftHandJacobian.getJointsInOrder(), OneDoFJoint.class);
//
//         for (int i = 0; i < oneDoFJoints.length; i++)
//         {
//            OneDoFJoint oneDoFJoint = oneDoFJoints[i];
//            double q = oneDoFJoint.getQ();
//
//            assertTrue("Failed joint limit", q >= oneDoFJoint.getJointLimitLower());
//            assertTrue("Failed joint limit", q <= oneDoFJoint.getJointLimitUpper());
//         }
//      }
   }

   private FrameOrientation getHandEndEffectorOrientation()
   {
      ReferenceFrame handEndEffectorFrame = fullRobotModel.getHand(RobotSide.LEFT).getBodyFixedFrame();
      FrameOrientation handEndEffectorOrientation = new FrameOrientation(handEndEffectorFrame);

      handEndEffectorOrientation.changeFrame(worldFrame);

      return handEndEffectorOrientation;
   }

   private FramePoint getHandEndEffectorPosition()
   {
      ReferenceFrame handEndEffectorFrame = fullRobotModel.getHand(RobotSide.LEFT).getBodyFixedFrame();
      FramePoint handEndEffectorPosition = new FramePoint(handEndEffectorFrame);

      handEndEffectorPosition.changeFrame(worldFrame);

      return handEndEffectorPosition;
   }

   private void initialArmPose()
   {
      for (JointNames name : JointNames.values())
      {
         oneDoFJoints.get(name).setQ(jointAnglesInitial.get(name));

         // oneDoFJoints.get(name).setQ(randomNumberGenerator.nextDouble()/10);
      }
   }

   private void randomArmPoseWithForwardKinematics()
   {
      for (JointNames name : JointNames.values())
      {
         double def = 0.0;

         jointAngles.put(name, generateRandomDoubleInRange(jointLimits.get(name).get(0) - def, jointLimits.get(name).get(1) + def));
         oneDoFJoints.get(name).setQ(jointAngles.get(name));
      }
   }

   private double generateRandomDoubleInRange(double max, double min)
   {
      return min + (max - min) * randomNumberGenerator.nextDouble();
   }

   private void populateEnumMaps()
   {
      oneDoFJoints.put(JointNames.SHOULDER_YAW, fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.SHOULDER_YAW));
      oneDoFJoints.put(JointNames.SHOULDER_ROLL, fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.SHOULDER_ROLL));
      oneDoFJoints.put(JointNames.ELBOW_PITCH, fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.ELBOW_PITCH));
      oneDoFJoints.put(JointNames.ELBOW_ROLL, fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.ELBOW_ROLL));
      oneDoFJoints.put(JointNames.WRIST_PITCH, fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.WRIST_PITCH));
      oneDoFJoints.put(JointNames.WRIST_ROLL, fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.WRIST_ROLL));
      jointLimits.put(JointNames.SHOULDER_YAW, shoulderYawLimits);
      jointLimits.put(JointNames.SHOULDER_ROLL, shoulderRollLimits);
      jointLimits.put(JointNames.ELBOW_PITCH, elbowPitchLimits);
      jointLimits.put(JointNames.ELBOW_ROLL, elbowRollLimits);
      jointLimits.put(JointNames.WRIST_PITCH, wristPitchLimits);
      jointLimits.put(JointNames.WRIST_ROLL, wristRollLimits);

      // *_*_Limits(*_*_Max, *_*_Min);
      shoulderRollLimits.add(oneDoFJoints.get(JointNames.SHOULDER_ROLL).getJointLimitUpper());
      shoulderRollLimits.add(oneDoFJoints.get(JointNames.SHOULDER_ROLL).getJointLimitLower());
      shoulderYawLimits.add(oneDoFJoints.get(JointNames.SHOULDER_YAW).getJointLimitUpper());
      shoulderYawLimits.add(oneDoFJoints.get(JointNames.SHOULDER_YAW).getJointLimitLower());
      elbowRollLimits.add(oneDoFJoints.get(JointNames.ELBOW_ROLL).getJointLimitUpper());
      elbowRollLimits.add(oneDoFJoints.get(JointNames.ELBOW_ROLL).getJointLimitLower());
      elbowPitchLimits.add(oneDoFJoints.get(JointNames.ELBOW_PITCH).getJointLimitUpper());
      elbowPitchLimits.add(oneDoFJoints.get(JointNames.ELBOW_PITCH).getJointLimitLower());
      wristRollLimits.add(oneDoFJoints.get(JointNames.WRIST_ROLL).getJointLimitUpper());
      wristRollLimits.add(oneDoFJoints.get(JointNames.WRIST_ROLL).getJointLimitLower());
      wristPitchLimits.add(oneDoFJoints.get(JointNames.WRIST_PITCH).getJointLimitUpper());
      wristPitchLimits.add(oneDoFJoints.get(JointNames.WRIST_PITCH).getJointLimitLower());
      jointAnglesInitial.put(JointNames.SHOULDER_YAW, 0.346);
      jointAnglesInitial.put(JointNames.SHOULDER_ROLL, -1.3141);
      jointAnglesInitial.put(JointNames.ELBOW_PITCH, 1.9195);
      jointAnglesInitial.put(JointNames.ELBOW_ROLL, 1.1749);
      jointAnglesInitial.put(JointNames.WRIST_PITCH, -0.0068);
      jointAnglesInitial.put(JointNames.WRIST_ROLL, -0.0447);
   }

   private double getMaximumArray(ArrayList<Long> array)
   {
      double max = 0;

      for (int i = 0; i < array.size(); i++)
      {
         if (max < array.get(i).doubleValue())
         {
            max = array.get(i).doubleValue();
         }
      }

      return max;
   }

   public double getAvarageArray(ArrayList<Long> array)
   {
      double sum = 0;

      for (int i = 0; i < array.size(); i++)
      {
         sum += array.get(i).doubleValue();
      }

      return sum / array.size();
   }
}
