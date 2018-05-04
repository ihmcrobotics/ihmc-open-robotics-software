package us.ihmc.avatar;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.Random;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.kinematics.DdoglegInverseKinematicsCalculator;
import us.ihmc.robotics.kinematics.InverseKinematicsCalculator;
import us.ihmc.robotics.kinematics.InverseKinematicsStepListener;
import us.ihmc.robotics.kinematics.KinematicSolver;
import us.ihmc.robotics.kinematics.NumericalInverseKinematicsCalculator;
import us.ihmc.robotics.kinematics.RandomRestartInverseKinematicsCalculator;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;

public abstract class NumericalInverseKinematicsCalculatorWithRobotTest implements MultiRobotTestInterface
{
   private boolean PRINT_OUT_TROUBLESOME = false;
   private boolean VISUALIZE = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final ArrayList<Double> shoulderRollLimits = new ArrayList<Double>();
   private static final ArrayList<Double> elbowRollLimits = new ArrayList<Double>();
   private static final ArrayList<Double> wristRollLimits = new ArrayList<Double>();
   private static final ArrayList<Double> shoulderYawLimits = new ArrayList<Double>();
   private static final ArrayList<Double> elbowPitchLimits = new ArrayList<Double>();
   private static final ArrayList<Double> wristPitchLimits = new ArrayList<Double>();
   private static final boolean DEBUG = true;

   private final EnumMap<JointNames, OneDoFJoint> oneDoFJoints = new EnumMap<JointNames, OneDoFJoint>(JointNames.class);
   private final EnumMap<JointNames, ArrayList<Double>> jointLimits = new EnumMap<JointNames, ArrayList<Double>>(JointNames.class);
   private final EnumMap<JointNames, Double> jointAngles = new EnumMap<JointNames, Double>(JointNames.class);
   private final EnumMap<JointNames, Double> jointAnglesInitial = new EnumMap<JointNames, Double>(JointNames.class);

   private final ArrayList<Long> solvingTime = new ArrayList<Long>();
   private final FullHumanoidRobotModel fullRobotModel;
   private final ReferenceFrame worldFrame;
   private GeometricJacobian leftHandJacobian;
   private InverseKinematicsCalculator inverseKinematicsCalculator;
   private InverseKinematicsStepListener inverseKinematicsStepListener;

   private SimulationConstructionSet scs;
   private FloatingRootJointRobot sdfRobot;
   private JointAnglesWriter jointAnglesWriter;

   private final YoFramePoint3D testPositionForwardKinematics = new YoFramePoint3D("testPositionForwardKinematics", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameYawPitchRoll testOrientationForwardKinematics = new YoFrameYawPitchRoll("testOrientationForwardKinematics",
                                                                          ReferenceFrame.getWorldFrame(), registry);
   private final YoDouble yoErrorScalar = new YoDouble("errorScalar", registry);
   private final YoDouble positionError = new YoDouble("positionError", registry);
   private final YoDouble orientationError = new YoDouble("orientationError", registry);

   private final YoDouble numberOfIterations = new YoDouble("numberOfIterations", registry);

   private final YoFramePoint3D testPositionInverseKinematics = new YoFramePoint3D("testPositionInverseKinematics", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameYawPitchRoll testOrientationInverseKinematics = new YoFrameYawPitchRoll("testOrientationInverseKinematics",
                                                                          ReferenceFrame.getWorldFrame(), registry);

   private enum InverseKinematicsSolver {TWAN_SOLVER, PETER_SOLVER, MAARTEN_SOLVER;}

   private enum JointNames
   {
      SHOULDER_YAW, SHOULDER_ROLL, ELBOW_PITCH, ELBOW_ROLL, WRIST_PITCH, WRIST_ROLL;
   }

   public NumericalInverseKinematicsCalculatorWithRobotTest()
   {
//      InverseKinematicsSolver inverseKinameticSolverToUse = InverseKinematicsSolver.TWAN_SOLVER;
      InverseKinematicsSolver inverseKinameticSolverToUse = InverseKinematicsSolver.PETER_SOLVER;
      DRCRobotModel robotModel = getRobotModel();

      sdfRobot = robotModel.createHumanoidFloatingRootJointRobot(false);

      if (VISUALIZE)
      {
         scs = new SimulationConstructionSet(sdfRobot);
         scs.addYoVariableRegistry(registry);

         YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

         double positionVizSize = 0.06;
         YoGraphicPosition testPositionForwardKinematicsViz = new YoGraphicPosition("testPositionForwardKinematics", testPositionForwardKinematics,
                                                                 positionVizSize, YoAppearance.Green());
         yoGraphicsListRegistry.registerYoGraphic("InverseKinematicsCalculatorTest", testPositionForwardKinematicsViz);

         YoGraphicPosition testPositionInverseKinematicsViz = new YoGraphicPosition("testPositionInverseKinematicsViz", testPositionInverseKinematics,
                                                                 positionVizSize, YoAppearance.Red());
         yoGraphicsListRegistry.registerYoGraphic("InverseKinematicsCalculatorTest", testPositionInverseKinematicsViz);


         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
         scs.startOnAThread();
      }

      fullRobotModel = robotModel.createFullRobotModel();
      jointAnglesWriter = new JointAnglesWriter(sdfRobot, fullRobotModel);

      worldFrame = ReferenceFrame.getWorldFrame();
      populateEnumMaps();

      leftHandJacobian = new GeometricJacobian(fullRobotModel.getChest(), fullRobotModel.getHand(RobotSide.LEFT),
              fullRobotModel.getHand(RobotSide.LEFT).getBodyFixedFrame());

      switch (inverseKinameticSolverToUse)
      {
         case PETER_SOLVER :
         {
            double orientationDiscount = 0.2;
            int maxIterations = 5000;
            boolean solveOrientation = true;
            double convergeTolerance = 4.0e-7; //1e-12;
            double acceptTolLoc = 0.005;
            double acceptTolAngle = 0.02;
            double parameterChangePenalty = 1.0e-4; //0.1;
            inverseKinematicsCalculator = new DdoglegInverseKinematicsCalculator(leftHandJacobian, orientationDiscount, maxIterations, solveOrientation,
                    convergeTolerance, acceptTolLoc, acceptTolAngle, parameterChangePenalty);

            break;
         }

         case TWAN_SOLVER :
         {
            // These values were tuned by Jerry Pratt on February 24, 2015 to match Atlas the best.
            int maxIterations = 60;
            double lambdaLeastSquares = 0.0009;
            double tolerance = 0.0025;
            double maxStepSize = 0.2;
            double minRandomSearchScalar = 0.01;
            double maxRandomSearchScalar = 0.8;

            NumericalInverseKinematicsCalculator numericalInverseKinematicsCalculator = new NumericalInverseKinematicsCalculator(leftHandJacobian,
                                                                                           lambdaLeastSquares, tolerance, maxIterations, maxStepSize,
                                                                                           minRandomSearchScalar, maxRandomSearchScalar);

            int maxRestarts = 10;
            double restartTolerance = 0.006;

            inverseKinematicsCalculator = new RandomRestartInverseKinematicsCalculator(maxRestarts, restartTolerance, leftHandJacobian,
                    numericalInverseKinematicsCalculator);

            break;
         }

         case MAARTEN_SOLVER :
         {
            int maxIterations = 60;
            double tolerance = 1e-5;
            inverseKinematicsCalculator = new KinematicSolver(leftHandJacobian, tolerance, maxIterations);
         }
      }

      if (VISUALIZE)
      {
         inverseKinematicsStepListener = new InverseKinematicsStepListener()
         {
            @Override
            public void didAnInverseKinemticsStep(double errorScalar)
            {
               yoErrorScalar.set(errorScalar);
               jointAnglesWriter.updateRobotConfigurationBasedOnFullRobotModel();
               scs.tickAndUpdate();
            }
         };

         inverseKinematicsCalculator.attachInverseKinematicsStepListener(inverseKinematicsStepListener);
      }
   }

   public void testSimpleCase()
   {
      Random random = new Random(1984L);

      generateArmPoseSlightlyOffOfMidRangeWithForwardKinematics(random, 0.5);

      FramePoint3D handEndEffectorPositionFK = getHandEndEffectorPosition();
      FrameQuaternion handEndEffectorOrientationFK = getHandEndEffectorOrientation();

      InitialGuessForTests initialGuessForTests = InitialGuessForTests.MIDRANGE;
      boolean updateListenersEachStep = false;
      double errorThreshold = 0.01;
      boolean success = testAPose(random, handEndEffectorPositionFK, handEndEffectorOrientationFK, initialGuessForTests, errorThreshold, updateListenersEachStep);
      assertTrue(success);
   }

   public void testRandomFeasibleRobotPoses()
   {
      Random random = new Random(1776L);

      int numberOfTests = 5000;

      InitialGuessForTests initialGuessForTests = InitialGuessForTests.MIDRANGE;
      boolean updateListenersEachStep = false;

      int numberPassed = 0;

      for (int i = 0; i < numberOfTests; i++)
      {
         generateRandomArmPoseWithForwardKinematics(random);
         fullRobotModel.updateFrames();

         FramePoint3D handEndEffectorPositionFK = getHandEndEffectorPosition();
         FrameQuaternion handEndEffectorOrientationFK = getHandEndEffectorOrientation();

         if (VISUALIZE)
         {
            inverseKinematicsStepListener.didAnInverseKinemticsStep(-1.0);
         }

         double errorThreshold = 0.01;
         boolean success = testAPose(random, handEndEffectorPositionFK, handEndEffectorOrientationFK, initialGuessForTests, errorThreshold, updateListenersEachStep);
         if (success)
            numberPassed++;
      }

      double percentPassed = ((double) numberPassed) / ((double) numberOfTests);
      double averageTimeMillis = getAvarageArray(solvingTime);
      double maximumTimeMillis = getMaximumArray(solvingTime);

      if (DEBUG)
      {
         System.out.println("numberPassed = " + numberPassed);
         System.out.println("nTests = " + numberOfTests);
         System.out.println("percentPassed = " + percentPassed);

         System.out.println("Average Solving Time: " + averageTimeMillis + " ms");
         System.out.println("Maximal Solving Time: " + maximumTimeMillis + " ms");
      }

      final double maximumTimeMillisMax = 600.0;
      final double averageTimeMillisMax = 8.0;
      assertTrue("Average Solving Time > " + averageTimeMillisMax + " ms", averageTimeMillis < averageTimeMillisMax);
      assertTrue("Maximal Solving Time > " + maximumTimeMillisMax + " ms", maximumTimeMillis < maximumTimeMillisMax);

      //NumericalInverseKinematicCalculator is much faster than the DDogLegOne, so use the following when running it...
//      assertTrue(averageTimeMillis < 0.04);
//      assertTrue(maximumTimeMillis < 16.0);

      assertTrue(percentPassed > 0.96);
   }

   public boolean testAPose(Random random, FramePoint3D handEndEffectorPositionFK, FrameQuaternion handEndEffectorOrientationFK, InitialGuessForTests initialGuessForTests,
                            double errorThreshold, boolean updateListenersEachStep)
   {
      testPositionForwardKinematics.set(handEndEffectorPositionFK);
      testOrientationForwardKinematics.set(handEndEffectorOrientationFK);

      solveForArmPoseWithInverseKinematics(random, handEndEffectorOrientationFK, handEndEffectorPositionFK, initialGuessForTests, updateListenersEachStep);

      FramePoint3D handEndEffectorPositionIK = getHandEndEffectorPosition();
      FrameQuaternion handEndEffectorOrientationIK = getHandEndEffectorOrientation();

      testPositionInverseKinematics.set(handEndEffectorPositionIK);
      testOrientationInverseKinematics.set(handEndEffectorOrientationIK);

      positionError.set(handEndEffectorPositionFK.distance(handEndEffectorPositionIK));

      FrameQuaternion errorOrientation = new FrameQuaternion();
      errorOrientation.difference(handEndEffectorOrientationIK, handEndEffectorOrientationFK);

      AxisAngle axisAngle = new AxisAngle(errorOrientation);
      orientationError.set(axisAngle.getAngle());

      if (PRINT_OUT_TROUBLESOME && (positionError.getDoubleValue() > errorThreshold))
      {
         System.err.println("Troublesome position: " + handEndEffectorPositionFK + ", orientation = " + handEndEffectorOrientationFK);
      }

      jointAnglesWriter.updateRobotConfigurationBasedOnFullRobotModel();
      if (scs != null)
         scs.tickAndUpdate();

      boolean positionErrorAcceptable = (positionError.getDoubleValue() < errorThreshold);

      if (!positionErrorAcceptable)
      {
         PrintTools.error("Position error not acceptable: positionError: " + positionError.getDoubleValue() + "  maxAllowed: " + errorThreshold);
      }

      boolean orientationErrorAcceptable = (orientationError.getDoubleValue() < errorThreshold);

      if (!orientationErrorAcceptable)
      {
         PrintTools.error("Orientation error not acceptable: orientationError: " + orientationError.getDoubleValue() + "  maxAllowed: " + errorThreshold);
      }

      return positionErrorAcceptable && orientationErrorAcceptable;
   }

   private void solveForArmPoseWithInverseKinematics(Random random, FrameQuaternion desiredOrientation, FramePoint3D desiredPosition,
           InitialGuessForTests initialGuessForTests, boolean updateListenersEachStep)
   {
      FramePose3D handPose = new FramePose3D(worldFrame);

      handPose.setOrientation(desiredOrientation);
      handPose.setPosition(desiredPosition);
      handPose.changeFrame(fullRobotModel.getChest().getBodyFixedFrame());

      RigidBodyTransform transform = new RigidBodyTransform();

      handPose.get(transform);

      createInitialGuess(random, initialGuessForTests);

      if (VISUALIZE && updateListenersEachStep)
      {
         inverseKinematicsStepListener.didAnInverseKinemticsStep(0.0);
      }

      long start = System.nanoTime();

      solve(transform);

      long end = System.nanoTime();

      solvingTime.add((long) ((end - start) * 1e-6));
   }

   private void createInitialGuess(Random random, InitialGuessForTests initialGuessForTests)
   {
      switch (initialGuessForTests)
      {
         case PREVIOUS :
         {
            break;
         }

         case MIDRANGE :
         {
            generateArmPoseAtMidRangeWithForwardKinematics();

            break;
         }

         case RANDOM :
         {
            generateRandomArmPoseWithForwardKinematics(random);

            break;
         }

         default :
         {
            throw new RuntimeException("Shouldn't get here!");
         }
      }
   }

   private boolean solve(RigidBodyTransform transform)
   {
      boolean successfulSolve = inverseKinematicsCalculator.solve(transform);
      numberOfIterations.set(inverseKinematicsCalculator.getNumberOfIterations());

      return successfulSolve;
   }

   private FrameQuaternion getHandEndEffectorOrientation()
   {
      ReferenceFrame handEndEffectorFrame = fullRobotModel.getHand(RobotSide.LEFT).getBodyFixedFrame();
      FrameQuaternion handEndEffectorOrientation = new FrameQuaternion(handEndEffectorFrame);

      handEndEffectorOrientation.changeFrame(worldFrame);

      return handEndEffectorOrientation;
   }

   private FramePoint3D getHandEndEffectorPosition()
   {
      ReferenceFrame handEndEffectorFrame = fullRobotModel.getHand(RobotSide.LEFT).getBodyFixedFrame();
      FramePoint3D handEndEffectorPosition = new FramePoint3D(handEndEffectorFrame);

      handEndEffectorPosition.changeFrame(worldFrame);

      return handEndEffectorPosition;
   }

   private void generateRandomArmPoseWithForwardKinematics(Random random)
   {
      for (JointNames name : JointNames.values())
      {
         double bufferAwayFromJointLimits = 0.05;    // 0.2; //0.0;

         double minRange = jointLimits.get(name).get(0) - bufferAwayFromJointLimits;
         double maxRange = jointLimits.get(name).get(1) + bufferAwayFromJointLimits;

         double randomJointAngle = RandomNumbers.nextDouble(random, minRange, maxRange);
         jointAngles.put(name, randomJointAngle);
         oneDoFJoints.get(name).setQ(jointAngles.get(name));
      }
   }

   private void generateArmPoseSlightlyOffOfMidRangeWithForwardKinematics(Random random, double maxAngleDeviationFromMidRange)
   {
      for (JointNames name : JointNames.values())
      {
         double bufferAwayFromJointLimits = 0.2;    // 0.0;

         double minRange = jointLimits.get(name).get(0) - bufferAwayFromJointLimits;
         double maxRange = jointLimits.get(name).get(1) + bufferAwayFromJointLimits;

         double middleRangeJointAngle = (minRange + maxRange) / 2.0;

         double deviation = RandomNumbers.nextDouble(random, maxAngleDeviationFromMidRange);

         jointAngles.put(name, middleRangeJointAngle + deviation);
         oneDoFJoints.get(name).setQ(jointAngles.get(name));
      }
   }

   private void generateArmPoseAtMidRangeWithForwardKinematics()
   {
      for (JointNames name : JointNames.values())
      {
         double bufferAwayFromJointLimits = 0.2;    // 0.0;

         double minRange = jointLimits.get(name).get(0) - bufferAwayFromJointLimits;
         double maxRange = jointLimits.get(name).get(1) + bufferAwayFromJointLimits;

         double middleRangeJointAngle = (minRange + maxRange) / 2.0;
         jointAngles.put(name, middleRangeJointAngle);
         oneDoFJoints.get(name).setQ(jointAngles.get(name));
      }
   }


   private void populateEnumMaps()
   {
      oneDoFJoints.put(JointNames.SHOULDER_YAW, fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.SHOULDER_YAW));
      oneDoFJoints.put(JointNames.SHOULDER_ROLL, fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.SHOULDER_ROLL));
      oneDoFJoints.put(JointNames.ELBOW_PITCH, fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.ELBOW_PITCH));
      oneDoFJoints.put(JointNames.ELBOW_ROLL, fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.ELBOW_ROLL));
      oneDoFJoints.put(JointNames.WRIST_PITCH, fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.FIRST_WRIST_PITCH));
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

   protected enum InitialGuessForTests {RANDOM, PREVIOUS, MIDRANGE;}
}
