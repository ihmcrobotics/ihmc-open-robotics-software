package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.CubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.CubicMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing.SwingStateEndMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.transfer.TransferStateEndMatrix;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.InterpolationTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class StateEndCurrentMultiplierTest
{
   private static final double blendingFraction = 0.5;
   private static final double minimumBlendingTime = 0.05;
   private static final double epsilon = 0.0001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationTwoCMPTransfer()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble startOfSplineTime = new YoDouble("startOfSplineTime", registry);
      YoDouble endOfSplineTime = new YoDouble("endOfSplineTime", registry);
      YoDouble totalTrajectoryTime = new YoDouble("totalTrajectoryTime", registry);

      double omega = 3.0;
      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      List<YoDouble> transferSplitFractions = new ArrayList<>();
      List<YoDouble> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         YoDouble transferSplitRatio = new YoDouble("transferSplitRatio" + i, registry);
         YoDouble swingSplitRatio = new YoDouble("swingSplitRatio" + i, registry);
         transferSplitRatio.setToNaN();
         swingSplitRatio.setToNaN();
         transferSplitFractions.add(transferSplitRatio);
         swingSplitFractions.add(swingSplitRatio);
      }

      StateEndCurrentMultiplier stateEndCurrentMultiplier = new StateEndCurrentMultiplier(swingSplitFractions, transferSplitFractions,
            startOfSplineTime, endOfSplineTime, "", true, false, blendingFraction, minimumBlendingTime, registry);

      TransferStateEndMatrix stateEndMatrix = new TransferStateEndMatrix(swingSplitFractions, transferSplitFractions);
      CubicMatrix cubicMatrix = new CubicMatrix();
      CubicDerivativeMatrix cubicDerivativeMatrix = new CubicDerivativeMatrix();
      DenseMatrix64F positionMatrixOut = new DenseMatrix64F(1, 1);
      DenseMatrix64F velocityMatrixOut = new DenseMatrix64F(1, 1);

      for (int iter = 0; iter < iters; iter++)
      {
         double currentTransferRatio = 0.7 * random.nextDouble();
         double currentSwingRatio = 0.7 * random.nextDouble();
         double nextTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         swingSplitFractions.get(0).set(currentSwingRatio);
         transferSplitFractions.get(1).set(nextTransferRatio);

         doubleSupportDurations.get(0).set(2.0 * random.nextDouble());
         singleSupportDurations.get(0).set(5.0 * random.nextDouble());
         doubleSupportDurations.get(1).set(2.0 * random.nextDouble());

         double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();

         totalTrajectoryTime.set(singleSupportDuration);

         boolean isInTransfer = true;
         boolean useTwoCMPs = true;

         double doubleSupportDuration = doubleSupportDurations.get(0).getDoubleValue();

         double timeInCurrentState = random.nextDouble() * doubleSupportDuration;

         stateEndCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         cubicMatrix.setSegmentDuration(doubleSupportDuration);
         cubicDerivativeMatrix.setSegmentDuration(doubleSupportDuration);
         cubicMatrix.update(timeInCurrentState);
         cubicDerivativeMatrix.update(timeInCurrentState);

         stateEndMatrix.compute(1, singleSupportDurations, doubleSupportDurations, useTwoCMPs, omega);

         CommonOps.mult(cubicMatrix, stateEndMatrix, positionMatrixOut);
         CommonOps.mult(cubicDerivativeMatrix, stateEndMatrix, velocityMatrixOut);

         Assert.assertEquals(positionMatrixOut.get(0, 0), stateEndCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(velocityMatrixOut.get(0, 0), stateEndCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(stateEndCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(stateEndCurrentMultiplier.getVelocityMultiplier()));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationTwoCMPTransferWithBlending()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble startOfSplineTime = new YoDouble("startOfSplineTime", registry);
      YoDouble endOfSplineTime = new YoDouble("endOfSplineTime", registry);
      YoDouble totalTrajectoryTime = new YoDouble("totalTrajectoryTime", registry);

      double omega = 3.0;
      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      List<YoDouble> transferSplitFractions = new ArrayList<>();
      List<YoDouble> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         YoDouble transferSplitRatio = new YoDouble("transferSplitRatio" + i, registry);
         YoDouble swingSplitRatio = new YoDouble("swingSplitRatio" + i, registry);
         transferSplitRatio.setToNaN();
         swingSplitRatio.setToNaN();
         transferSplitFractions.add(transferSplitRatio);
         swingSplitFractions.add(swingSplitRatio);
      }

      StateEndCurrentMultiplier stateEndCurrentMultiplier = new StateEndCurrentMultiplier(swingSplitFractions, transferSplitFractions,
            startOfSplineTime, endOfSplineTime, "", true, true, blendingFraction, minimumBlendingTime, registry);

      TransferStateEndMatrix stateEndMatrix = new TransferStateEndMatrix(swingSplitFractions, transferSplitFractions);
      CubicMatrix cubicMatrix = new CubicMatrix();
      CubicDerivativeMatrix cubicDerivativeMatrix = new CubicDerivativeMatrix();
      DenseMatrix64F positionMatrixOut = new DenseMatrix64F(1, 1);
      DenseMatrix64F velocityMatrixOut = new DenseMatrix64F(1, 1);

      for (int iter = 0; iter < iters; iter++)
      {
         double currentTransferRatio = 0.7 * random.nextDouble();
         double currentSwingRatio = 0.7 * random.nextDouble();
         double nextTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         swingSplitFractions.get(0).set(currentSwingRatio);
         transferSplitFractions.get(1).set(nextTransferRatio);

         doubleSupportDurations.get(0).set(2.0 * random.nextDouble());
         singleSupportDurations.get(0).set(5.0 * random.nextDouble());
         doubleSupportDurations.get(1).set(2.0 * random.nextDouble());

         double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();

         totalTrajectoryTime.set(singleSupportDuration);

         boolean isInTransfer = true;
         boolean useTwoCMPs = true;

         double doubleSupportDuration = doubleSupportDurations.get(0).getDoubleValue();

         double timeInCurrentState = random.nextDouble() * doubleSupportDuration;

         stateEndCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         cubicMatrix.setSegmentDuration(doubleSupportDuration);
         cubicDerivativeMatrix.setSegmentDuration(doubleSupportDuration);
         cubicMatrix.update(timeInCurrentState);
         cubicDerivativeMatrix.update(timeInCurrentState);

         stateEndMatrix.compute(1, singleSupportDurations, doubleSupportDurations, useTwoCMPs, omega);

         CommonOps.mult(cubicMatrix, stateEndMatrix, positionMatrixOut);
         CommonOps.mult(cubicDerivativeMatrix, stateEndMatrix, velocityMatrixOut);

         Assert.assertEquals(positionMatrixOut.get(0, 0), stateEndCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(velocityMatrixOut.get(0, 0), stateEndCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(stateEndCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(stateEndCurrentMultiplier.getVelocityMultiplier()));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationOneCMPTransfer()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble startOfSplineTime = new YoDouble("startOfSplineTime", registry);
      YoDouble endOfSplineTime = new YoDouble("endOfSplineTime", registry);
      YoDouble totalTrajectoryTime = new YoDouble("totalTrajectoryTime", registry);

      double omega = 3.0;
      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      List<YoDouble> transferSplitFractions = new ArrayList<>();
      List<YoDouble> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         YoDouble transferSplitRatio = new YoDouble("transferSplitRatio" + i, registry);
         transferSplitRatio.setToNaN();
         transferSplitFractions.add(transferSplitRatio);
      }

      StateEndCurrentMultiplier stateEndCurrentMultiplier = new StateEndCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, "", true, false, blendingFraction, minimumBlendingTime, registry);

      TransferStateEndMatrix stateEndMatrix = new TransferStateEndMatrix(swingSplitFractions, transferSplitFractions);
      CubicMatrix cubicMatrix = new CubicMatrix();
      CubicDerivativeMatrix cubicDerivativeMatrix = new CubicDerivativeMatrix();
      DenseMatrix64F positionMatrixOut = new DenseMatrix64F(1, 1);
      DenseMatrix64F velocityMatrixOut = new DenseMatrix64F(1, 1);

      for (int iter = 0; iter < iters; iter++)
      {
         double currentTransferRatio = 0.7 * random.nextDouble();
         double nextTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         transferSplitFractions.get(1).set(nextTransferRatio);

         doubleSupportDurations.get(0).set(2.0 * random.nextDouble());
         singleSupportDurations.get(0).set(5.0 * random.nextDouble());
         doubleSupportDurations.get(1).set(2.0 * random.nextDouble());

         double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();

         totalTrajectoryTime.set(singleSupportDuration);

         boolean isInTransfer = true;
         boolean useTwoCMPs = false;

         double doubleSupportDuration = doubleSupportDurations.get(0).getDoubleValue();

         double timeInCurrentState = random.nextDouble() * doubleSupportDuration;

         stateEndCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         cubicMatrix.setSegmentDuration(doubleSupportDuration);
         cubicDerivativeMatrix.setSegmentDuration(doubleSupportDuration);
         cubicMatrix.update(timeInCurrentState);
         cubicDerivativeMatrix.update(timeInCurrentState);

         stateEndMatrix.compute(1, singleSupportDurations, doubleSupportDurations, false, omega);

         CommonOps.mult(cubicMatrix, stateEndMatrix, positionMatrixOut);
         CommonOps.mult(cubicDerivativeMatrix, stateEndMatrix, velocityMatrixOut);

         Assert.assertEquals(positionMatrixOut.get(0, 0), stateEndCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(velocityMatrixOut.get(0, 0), stateEndCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(stateEndCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(stateEndCurrentMultiplier.getVelocityMultiplier()));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationOneCMPTransferWithBlending()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble startOfSplineTime = new YoDouble("startOfSplineTime", registry);
      YoDouble endOfSplineTime = new YoDouble("endOfSplineTime", registry);
      YoDouble totalTrajectoryTime = new YoDouble("totalTrajectoryTime", registry);

      double omega = 3.0;
      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      List<YoDouble> transferSplitFractions = new ArrayList<>();
      List<YoDouble> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         YoDouble transferSplitRatio = new YoDouble("transferSplitRatio" + i, registry);
         transferSplitRatio.setToNaN();
         transferSplitFractions.add(transferSplitRatio);
      }

      StateEndCurrentMultiplier stateEndCurrentMultiplier = new StateEndCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, "", true, true, blendingFraction, minimumBlendingTime, registry);

      TransferStateEndMatrix stateEndMatrix = new TransferStateEndMatrix(swingSplitFractions, transferSplitFractions);
      CubicMatrix cubicMatrix = new CubicMatrix();
      CubicDerivativeMatrix cubicDerivativeMatrix = new CubicDerivativeMatrix();
      DenseMatrix64F positionMatrixOut = new DenseMatrix64F(1, 1);
      DenseMatrix64F velocityMatrixOut = new DenseMatrix64F(1, 1);

      for (int iter = 0; iter < iters; iter++)
      {
         double currentTransferRatio = 0.7 * random.nextDouble();
         double nextTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         transferSplitFractions.get(1).set(nextTransferRatio);

         doubleSupportDurations.get(0).set(2.0 * random.nextDouble());
         singleSupportDurations.get(0).set(5.0 * random.nextDouble());
         doubleSupportDurations.get(1).set(2.0 * random.nextDouble());

         double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();

         totalTrajectoryTime.set(singleSupportDuration);

         boolean isInTransfer = true;
         boolean useTwoCMPs = false;

         double doubleSupportDuration = doubleSupportDurations.get(0).getDoubleValue();

         double timeInCurrentState = random.nextDouble() * doubleSupportDuration;

         stateEndCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         cubicMatrix.setSegmentDuration(doubleSupportDuration);
         cubicDerivativeMatrix.setSegmentDuration(doubleSupportDuration);
         cubicMatrix.update(timeInCurrentState);
         cubicDerivativeMatrix.update(timeInCurrentState);

         stateEndMatrix.compute(1, singleSupportDurations, doubleSupportDurations, false, omega);

         CommonOps.mult(cubicMatrix, stateEndMatrix, positionMatrixOut);
         CommonOps.mult(cubicDerivativeMatrix, stateEndMatrix, velocityMatrixOut);

         Assert.assertEquals(positionMatrixOut.get(0, 0), stateEndCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(velocityMatrixOut.get(0, 0), stateEndCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(stateEndCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(stateEndCurrentMultiplier.getVelocityMultiplier()));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationTwoCMPFirstSegment()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble startOfSplineTime = new YoDouble("startOfSplineTime", registry);
      YoDouble endOfSplineTime = new YoDouble("endOfSplineTime", registry);
      YoDouble totalTrajectoryTime = new YoDouble("totalTrajectoryTime", registry);

      double omega = 3.0;
      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      List<YoDouble> transferSplitFractions = new ArrayList<>();
      List<YoDouble> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         YoDouble transferSplitRatio = new YoDouble("transferSplitRatio" + i, registry);
         YoDouble swingSplitRatio = new YoDouble("swingSplitRatio" + i, registry);
         transferSplitRatio.setToNaN();
         swingSplitRatio.setToNaN();
         transferSplitFractions.add(transferSplitRatio);
         swingSplitFractions.add(swingSplitRatio);
      }

      StateEndCurrentMultiplier stateEndCurrentMultiplier = new StateEndCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, "", true, false, blendingFraction, minimumBlendingTime, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double currentTransferRatio = 0.7 * random.nextDouble();
         double currentSwingRatio = 0.7 * random.nextDouble();
         double nextTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         swingSplitFractions.get(0).set(currentSwingRatio);
         transferSplitFractions.get(1).set(nextTransferRatio);

         doubleSupportDurations.get(0).set(2.0 * random.nextDouble());
         singleSupportDurations.get(0).set(5.0 * random.nextDouble());
         doubleSupportDurations.get(1).set(2.0 * random.nextDouble());

         double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();
         double minimumSplineTime = Math.min(singleSupportDuration, 0.5);
         double startOfSpline = 0.2 * random.nextDouble();
         double endOfSpline = singleSupportDuration - 0.2 * random.nextDouble();
         if (minimumSplineTime > endOfSpline - startOfSpline)
            startOfSpline = 0.0;
         if (minimumSplineTime > endOfSpline - startOfSpline)
            endOfSpline = singleSupportDuration;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupportDuration);

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         double timeInCurrentState = random.nextDouble() * startOfSpline;

         stateEndCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(0.0, stateEndCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(0.0, stateEndCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(stateEndCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(stateEndCurrentMultiplier.getVelocityMultiplier()));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationTwoCMPFirstSegmentWithBlending()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble startOfSplineTime = new YoDouble("startOfSplineTime", registry);
      YoDouble endOfSplineTime = new YoDouble("endOfSplineTime", registry);
      YoDouble totalTrajectoryTime = new YoDouble("totalTrajectoryTime", registry);

      double omega = 3.0;
      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      List<YoDouble> transferSplitFractions = new ArrayList<>();
      List<YoDouble> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         YoDouble transferSplitRatio = new YoDouble("transferSplitRatio" + i, registry);
         YoDouble swingSplitRatio = new YoDouble("swingSplitRatio" + i, registry);
         transferSplitRatio.setToNaN();
         swingSplitRatio.setToNaN();
         transferSplitFractions.add(transferSplitRatio);
         swingSplitFractions.add(swingSplitRatio);
      }

      StateEndCurrentMultiplier stateEndCurrentMultiplier = new StateEndCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, "", true, true, blendingFraction, minimumBlendingTime, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double currentTransferRatio = 0.7 * random.nextDouble();
         double currentSwingRatio = 0.7 * random.nextDouble();
         double nextTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         swingSplitFractions.get(0).set(currentSwingRatio);
         transferSplitFractions.get(1).set(nextTransferRatio);

         doubleSupportDurations.get(0).set(2.0 * random.nextDouble());
         singleSupportDurations.get(0).set(5.0 * random.nextDouble());
         doubleSupportDurations.get(1).set(2.0 * random.nextDouble());

         double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();
         double minimumSplineTime = Math.min(singleSupportDuration, 0.5);
         double startOfSpline = 0.2 * random.nextDouble();
         double endOfSpline = singleSupportDuration - 0.2 * random.nextDouble();
         if (minimumSplineTime > endOfSpline - startOfSpline)
            startOfSpline = 0.0;
         if (minimumSplineTime > endOfSpline - startOfSpline)
            endOfSpline = singleSupportDuration;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupportDuration);

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         double timeInCurrentState = random.nextDouble() * startOfSpline;

         stateEndCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         double nextTransferOnExitCMP = nextTransferRatio * doubleSupportDurations.get(1).getDoubleValue();
         double currentSwingOnExit = (1.0 - currentSwingRatio) * singleSupportDuration;
         double timeOnExit = currentSwingOnExit + nextTransferOnExitCMP;

         double currentSwingOnEntry = currentSwingRatio * singleSupportDuration;

         double recursion = Math.exp(omega * (timeInCurrentState - timeOnExit - currentSwingOnEntry));

         double blendingTime = Math.max(blendingFraction * startOfSpline, minimumBlendingTime);
         double alpha = MathTools.clamp(timeInCurrentState / blendingTime, 0.0, 1.0);

         double positionMultiplier = InterpolationTools.linearInterpolate(0.0, recursion, alpha);

         Assert.assertEquals(positionMultiplier, stateEndCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(omega * positionMultiplier, stateEndCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(stateEndCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(stateEndCurrentMultiplier.getVelocityMultiplier()));
      }
   }


   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationTwoCMPSecondSegment()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble startOfSplineTime = new YoDouble("startOfSplineTime", registry);
      YoDouble endOfSplineTime = new YoDouble("endOfSplineTime", registry);
      YoDouble totalTrajectoryTime = new YoDouble("totalTrajectoryTime", registry);

      double omega = 3.0;
      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      List<YoDouble> transferSplitFractions = new ArrayList<>();
      List<YoDouble> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         YoDouble transferSplitRatio = new YoDouble("transferSplitRatio" + i, registry);
         YoDouble swingSplitRatio = new YoDouble("swingSplitRatio" + i, registry);
         transferSplitRatio.setToNaN();
         swingSplitRatio.setToNaN();
         transferSplitFractions.add(transferSplitRatio);
         swingSplitFractions.add(swingSplitRatio);
      }

      StateEndCurrentMultiplier stateEndCurrentMultiplier = new StateEndCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, "", true, false, blendingFraction, minimumBlendingTime, registry);

      SwingStateEndMatrix stateEndMatrix = new SwingStateEndMatrix(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime, false, minimumBlendingTime);
      CubicMatrix cubicMatrix = new CubicMatrix();
      CubicDerivativeMatrix cubicDerivativeMatrix = new CubicDerivativeMatrix();
      DenseMatrix64F positionMatrixOut = new DenseMatrix64F(1, 1);
      DenseMatrix64F velocityMatrixOut = new DenseMatrix64F(1, 1);

      for (int iter = 0; iter < iters; iter++)
      {
         double currentTransferRatio = 0.7 * random.nextDouble();
         double currentSwingRatio = 0.7 * random.nextDouble();
         double nextTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         swingSplitFractions.get(0).set(currentSwingRatio);
         transferSplitFractions.get(1).set(nextTransferRatio);

         doubleSupportDurations.get(0).set(2.0 * random.nextDouble());
         singleSupportDurations.get(0).set(5.0 * random.nextDouble());
         doubleSupportDurations.get(1).set(2.0 * random.nextDouble());

         double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();
         double minimumSplineTime = Math.min(singleSupportDuration, 0.5);
         double startOfSpline = 0.2 * random.nextDouble();
         double endOfSpline = singleSupportDuration - 0.2 * random.nextDouble();
         if (minimumSplineTime > endOfSpline - startOfSpline)
            startOfSpline = 0.0;
         if (minimumSplineTime > endOfSpline - startOfSpline)
            endOfSpline = singleSupportDuration;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupportDuration);

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         double timeInCurrentState = random.nextDouble() * (endOfSpline - startOfSpline) + startOfSpline;

         double splineDuration = endOfSpline - startOfSpline;
         cubicMatrix.setSegmentDuration(splineDuration);
         cubicDerivativeMatrix.setSegmentDuration(splineDuration);

         double timeInSpline = timeInCurrentState - startOfSpline;
         cubicMatrix.update(timeInSpline);
         cubicDerivativeMatrix.update(timeInSpline);

         stateEndMatrix.compute(singleSupportDurations, doubleSupportDurations, omega);
         CommonOps.mult(cubicMatrix, stateEndMatrix, positionMatrixOut);
         CommonOps.mult(cubicDerivativeMatrix, stateEndMatrix, velocityMatrixOut);

         stateEndCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(positionMatrixOut.get(0, 0), stateEndCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(velocityMatrixOut.get(0, 0), stateEndCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(stateEndCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(stateEndCurrentMultiplier.getVelocityMultiplier()));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationTwoCMPSecondSegmentWithBlending()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble startOfSplineTime = new YoDouble("startOfSplineTime", registry);
      YoDouble endOfSplineTime = new YoDouble("endOfSplineTime", registry);
      YoDouble totalTrajectoryTime = new YoDouble("totalTrajectoryTime", registry);

      double omega = 3.0;
      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      List<YoDouble> transferSplitFractions = new ArrayList<>();
      List<YoDouble> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         YoDouble transferSplitRatio = new YoDouble("transferSplitRatio" + i, registry);
         YoDouble swingSplitRatio = new YoDouble("swingSplitRatio" + i, registry);
         transferSplitRatio.setToNaN();
         swingSplitRatio.setToNaN();
         transferSplitFractions.add(transferSplitRatio);
         swingSplitFractions.add(swingSplitRatio);
      }

      StateEndCurrentMultiplier stateEndCurrentMultiplier = new StateEndCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, "", true, true, blendingFraction, minimumBlendingTime, registry);

      SwingStateEndMatrix stateEndMatrix = new SwingStateEndMatrix(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime, true, minimumBlendingTime);
      CubicMatrix cubicMatrix = new CubicMatrix();
      CubicDerivativeMatrix cubicDerivativeMatrix = new CubicDerivativeMatrix();
      DenseMatrix64F positionMatrixOut = new DenseMatrix64F(1, 1);
      DenseMatrix64F velocityMatrixOut = new DenseMatrix64F(1, 1);

      for (int iter = 0; iter < iters; iter++)
      {
         double currentTransferRatio = 0.7 * random.nextDouble();
         double currentSwingRatio = 0.7 * random.nextDouble();
         double nextTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         swingSplitFractions.get(0).set(currentSwingRatio);
         transferSplitFractions.get(1).set(nextTransferRatio);

         doubleSupportDurations.get(0).set(2.0 * random.nextDouble());
         singleSupportDurations.get(0).set(5.0 * random.nextDouble());
         doubleSupportDurations.get(1).set(2.0 * random.nextDouble());

         double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();
         double minimumSplineTime = Math.min(singleSupportDuration, 0.5);
         double startOfSpline = 0.2 * random.nextDouble();
         double endOfSpline = singleSupportDuration - 0.2 * random.nextDouble();
         if (minimumSplineTime > endOfSpline - startOfSpline)
            startOfSpline = 0.0;
         if (minimumSplineTime > endOfSpline - startOfSpline)
            endOfSpline = singleSupportDuration;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupportDuration);

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         double timeInCurrentState = random.nextDouble() * (endOfSpline - startOfSpline) + startOfSpline;

         double splineDuration = endOfSpline - startOfSpline;
         cubicMatrix.setSegmentDuration(splineDuration);
         cubicDerivativeMatrix.setSegmentDuration(splineDuration);

         double timeInSpline = timeInCurrentState - startOfSpline;
         cubicMatrix.update(timeInSpline);
         cubicDerivativeMatrix.update(timeInSpline);

         stateEndMatrix.compute(singleSupportDurations, doubleSupportDurations, omega);
         CommonOps.mult(cubicMatrix, stateEndMatrix, positionMatrixOut);
         CommonOps.mult(cubicDerivativeMatrix, stateEndMatrix, velocityMatrixOut);

         stateEndCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(positionMatrixOut.get(0, 0), stateEndCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(velocityMatrixOut.get(0, 0), stateEndCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(stateEndCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(stateEndCurrentMultiplier.getVelocityMultiplier()));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationTwoCMPThirdSegment()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble startOfSplineTime = new YoDouble("startOfSplineTime", registry);
      YoDouble endOfSplineTime = new YoDouble("endOfSplineTime", registry);
      YoDouble totalTrajectoryTime = new YoDouble("totalTrajectoryTime", registry);

      double omega = 3.0;
      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      List<YoDouble> transferSplitFractions = new ArrayList<>();
      List<YoDouble> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         YoDouble transferSplitRatio = new YoDouble("transferSplitRatio" + i, registry);
         YoDouble swingSplitRatio = new YoDouble("swingSplitRatio" + i, registry);
         transferSplitRatio.setToNaN();
         swingSplitRatio.setToNaN();
         transferSplitFractions.add(transferSplitRatio);
         swingSplitFractions.add(swingSplitRatio);
      }

      StateEndCurrentMultiplier stateEndCurrentMultiplier = new StateEndCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, "", true, false, blendingFraction, minimumBlendingTime, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double currentTransferRatio = 0.7 * random.nextDouble();
         double currentSwingRatio = 0.7 * random.nextDouble();
         double nextTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         swingSplitFractions.get(0).set(currentSwingRatio);
         transferSplitFractions.get(1).set(nextTransferRatio);

         doubleSupportDurations.get(0).set(2.0 * random.nextDouble());
         singleSupportDurations.get(0).set(5.0 * random.nextDouble());
         doubleSupportDurations.get(1).set(2.0 * random.nextDouble());

         double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();
         double minimumSplineTime = Math.min(singleSupportDuration, 0.5);
         double startOfSpline = 0.2 * random.nextDouble();
         double endOfSpline = singleSupportDuration - 0.2 * random.nextDouble();
         if (minimumSplineTime > endOfSpline - startOfSpline)
            startOfSpline = 0.0;
         if (minimumSplineTime > endOfSpline - startOfSpline)
            endOfSpline = singleSupportDuration;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupportDuration);

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         double timeInCurrentState = random.nextDouble() * (singleSupportDuration - endOfSpline) + endOfSpline;

         stateEndCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         double timeRemaining = singleSupportDuration - timeInCurrentState;
         double nextTransferOnExit = transferSplitFractions.get(1).getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();
         double projectionTime = timeRemaining + nextTransferOnExit;
         double projection = Math.exp(-omega * projectionTime);

         Assert.assertEquals(projection, stateEndCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(omega * projection, stateEndCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(stateEndCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(stateEndCurrentMultiplier.getVelocityMultiplier()));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationTwoCMPThirdSegmentWithBlending()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble startOfSplineTime = new YoDouble("startOfSplineTime", registry);
      YoDouble endOfSplineTime = new YoDouble("endOfSplineTime", registry);
      YoDouble totalTrajectoryTime = new YoDouble("totalTrajectoryTime", registry);

      double omega = 3.0;
      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      List<YoDouble> transferSplitFractions = new ArrayList<>();
      List<YoDouble> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         YoDouble transferSplitRatio = new YoDouble("transferSplitRatio" + i, registry);
         YoDouble swingSplitRatio = new YoDouble("swingSplitRatio" + i, registry);
         transferSplitRatio.setToNaN();
         swingSplitRatio.setToNaN();
         transferSplitFractions.add(transferSplitRatio);
         swingSplitFractions.add(swingSplitRatio);
      }

      StateEndCurrentMultiplier stateEndCurrentMultiplier = new StateEndCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, "", true, true, blendingFraction, minimumBlendingTime, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double currentTransferRatio = 0.7 * random.nextDouble();
         double currentSwingRatio = 0.7 * random.nextDouble();
         double nextTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         swingSplitFractions.get(0).set(currentSwingRatio);
         transferSplitFractions.get(1).set(nextTransferRatio);

         doubleSupportDurations.get(0).set(2.0 * random.nextDouble());
         singleSupportDurations.get(0).set(5.0 * random.nextDouble());
         doubleSupportDurations.get(1).set(2.0 * random.nextDouble());

         double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();
         double minimumSplineTime = Math.min(singleSupportDuration, 0.5);
         double startOfSpline = 0.2 * random.nextDouble();
         double endOfSpline = singleSupportDuration - 0.2 * random.nextDouble();
         if (minimumSplineTime > endOfSpline - startOfSpline)
            startOfSpline = 0.0;
         if (minimumSplineTime > endOfSpline - startOfSpline)
            endOfSpline = singleSupportDuration;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupportDuration);

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         double timeInCurrentState = random.nextDouble() * (singleSupportDuration - endOfSpline) + endOfSpline;

         stateEndCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         double timeRemaining = singleSupportDuration - timeInCurrentState;
         double nextTransferOnExit = transferSplitFractions.get(1).getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();
         double projectionTime = timeRemaining + nextTransferOnExit;
         double projection = Math.exp(-omega * projectionTime);

         Assert.assertEquals(projection, stateEndCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(omega * projection, stateEndCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(stateEndCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(stateEndCurrentMultiplier.getVelocityMultiplier()));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationOneCMPSwing()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble startOfSplineTime = new YoDouble("startOfSplineTime", registry);
      YoDouble endOfSplineTime = new YoDouble("endOfSplineTime", registry);
      YoDouble totalTrajectoryTime = new YoDouble("totalTrajectoryTime", registry);

      double omega = 3.0;
      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      List<YoDouble> transferSplitFractions = new ArrayList<>();
      List<YoDouble> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         YoDouble transferSplitRatio = new YoDouble("transferSplitRatio" + i, registry);
         transferSplitRatio.setToNaN();
         transferSplitFractions.add(transferSplitRatio);
      }

      StateEndCurrentMultiplier stateEndCurrentMultiplier = new StateEndCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, "", true, false, blendingFraction, minimumBlendingTime, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double currentTransferRatio = 0.7 * random.nextDouble();
         double nextTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         transferSplitFractions.get(1).set(nextTransferRatio);

         doubleSupportDurations.get(0).set(2.0 * random.nextDouble());
         singleSupportDurations.get(0).set(5.0 * random.nextDouble());
         doubleSupportDurations.get(1).set(2.0 * random.nextDouble());

         double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();
         double minimumSplineTime = Math.min(singleSupportDuration, 0.5);
         double startOfSpline = 0.2 * random.nextDouble();
         double endOfSpline = singleSupportDuration - 0.2 * random.nextDouble();
         if (minimumSplineTime > endOfSpline - startOfSpline)
            startOfSpline = 0.0;
         if (minimumSplineTime > endOfSpline - startOfSpline)
            endOfSpline = singleSupportDuration;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupportDuration);

         boolean isInTransfer = false;

         double timeInCurrentState = random.nextDouble() * singleSupportDuration;

         stateEndCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, false, isInTransfer, omega);

         double nextTransferOnCurrentCMP = transferSplitFractions.get(1).getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();
         double timeRemaining = singleSupportDuration - timeInCurrentState;
         double projectionTime = timeRemaining + nextTransferOnCurrentCMP;

         double projection = Math.exp(-omega * projectionTime);

         Assert.assertEquals(projection, stateEndCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(omega * projection, stateEndCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(stateEndCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(stateEndCurrentMultiplier.getVelocityMultiplier()));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationOneCMPSwingWithBlending()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble startOfSplineTime = new YoDouble("startOfSplineTime", registry);
      YoDouble endOfSplineTime = new YoDouble("endOfSplineTime", registry);
      YoDouble totalTrajectoryTime = new YoDouble("totalTrajectoryTime", registry);

      double omega = 3.0;
      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      List<YoDouble> transferSplitFractions = new ArrayList<>();
      List<YoDouble> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         YoDouble transferSplitRatio = new YoDouble("transferSplitRatio" + i, registry);
         transferSplitRatio.setToNaN();
         transferSplitFractions.add(transferSplitRatio);
      }

      StateEndCurrentMultiplier stateEndCurrentMultiplier = new StateEndCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, "", true, true, blendingFraction, minimumBlendingTime, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double currentTransferRatio = 0.7 * random.nextDouble();
         double nextTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         transferSplitFractions.get(1).set(nextTransferRatio);

         doubleSupportDurations.get(0).set(2.0 * random.nextDouble());
         singleSupportDurations.get(0).set(5.0 * random.nextDouble());
         doubleSupportDurations.get(1).set(2.0 * random.nextDouble());

         double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();
         double minimumSplineTime = Math.min(singleSupportDuration, 0.5);
         double startOfSpline = 0.2 * random.nextDouble();
         double endOfSpline = singleSupportDuration - 0.2 * random.nextDouble();
         if (minimumSplineTime > endOfSpline - startOfSpline)
            startOfSpline = 0.0;
         if (minimumSplineTime > endOfSpline - startOfSpline)
            endOfSpline = singleSupportDuration;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupportDuration);

         boolean isInTransfer = false;

         double timeInCurrentState = random.nextDouble() * singleSupportDuration;

         stateEndCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, false, isInTransfer, omega);

         double nextTransferOnCurrentCMP = transferSplitFractions.get(1).getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();
         double timeRemaining = singleSupportDuration - timeInCurrentState;
         double projectionTime = timeRemaining + nextTransferOnCurrentCMP;

         double projection = 0.0;
         double recursion = Math.exp(-omega * projectionTime);

         double blendingTime = Math.max(blendingFraction * singleSupportDuration, minimumBlendingTime);
         double alpha = MathTools.clamp(timeInCurrentState / blendingTime, 0.0, 1.0);

         double positionMultiplier = InterpolationTools.linearInterpolate(projection, recursion, alpha);

         Assert.assertEquals(positionMultiplier, stateEndCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(omega * positionMultiplier, stateEndCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(stateEndCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(stateEndCurrentMultiplier.getVelocityMultiplier()));
      }
   }
}
