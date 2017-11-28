package us.ihmc.commonWalkingControlModules.capturePoint.optimization.recursiveController.multipliers.current;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.current.EntryCMPCurrentMultiplier;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.interpolation.CubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.interpolation.CubicMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.stateMatrices.swing.SwingEntryCMPMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.stateMatrices.transfer.TransferEntryCMPMatrix;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class EntryCMPCurrentMultiplierTest
{
   private static final double minimumBlendingTime = 0.05;
   private static final double blendingFraction = 0.5;
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

      EntryCMPCurrentMultiplier entryCMPCurrentMultiplier = new EntryCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, totalTrajectoryTime, "", true, false, blendingFraction, minimumBlendingTime, registry);

      TransferEntryCMPMatrix entryCMPMatrix = new TransferEntryCMPMatrix(swingSplitFractions, transferSplitFractions);

      CubicMatrix positionMatrix = new CubicMatrix();
      CubicDerivativeMatrix velocityMatrix = new CubicDerivativeMatrix();
      DenseMatrix64F position = new DenseMatrix64F(1, 1);
      DenseMatrix64F velocity = new DenseMatrix64F(1, 1);

      for (int iter = 0; iter < iters; iter++)
      {
         double currentTransferRatio = 0.7 * random.nextDouble();
         double currentSwingRatio = 0.7 * random.nextDouble();
         double nextTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         swingSplitFractions.get(0).set(currentSwingRatio);
         transferSplitFractions.get(1).set(nextTransferRatio);

         double currentDoubleSupportDuration = 2.0 * random.nextDouble();
         double currentSingleSupportDuration = 5.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(currentDoubleSupportDuration);
         singleSupportDurations.get(0).set(currentSingleSupportDuration);
         doubleSupportDurations.get(1).set(2.0 * random.nextDouble());

         totalTrajectoryTime.set(currentSingleSupportDuration);

         boolean isInTransfer = true;
         boolean useTwoCMPs = true;

         double timeInCurrentState = random.nextDouble() * currentDoubleSupportDuration;

         positionMatrix.setSegmentDuration(currentDoubleSupportDuration);
         velocityMatrix.setSegmentDuration(currentDoubleSupportDuration);
         positionMatrix.update(timeInCurrentState);
         velocityMatrix.update(timeInCurrentState);

         entryCMPMatrix.compute(1, singleSupportDurations, doubleSupportDurations, useTwoCMPs, omega);

         CommonOps.mult(positionMatrix, entryCMPMatrix, position);
         CommonOps.mult(velocityMatrix, entryCMPMatrix, velocity);

         entryCMPCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(position.get(0, 0), entryCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(velocity.get(0, 0), entryCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(entryCMPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(entryCMPCurrentMultiplier.getVelocityMultiplier()));
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

      EntryCMPCurrentMultiplier entryCMPCurrentMultiplier = new EntryCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, totalTrajectoryTime, "", true, true, blendingFraction, minimumBlendingTime, registry);

      TransferEntryCMPMatrix entryCMPMatrix = new TransferEntryCMPMatrix(swingSplitFractions, transferSplitFractions);

      CubicMatrix positionMatrix = new CubicMatrix();
      CubicDerivativeMatrix velocityMatrix = new CubicDerivativeMatrix();
      DenseMatrix64F position = new DenseMatrix64F(1, 1);
      DenseMatrix64F velocity = new DenseMatrix64F(1, 1);

      for (int iter = 0; iter < iters; iter++)
      {
         double currentTransferRatio = 0.7 * random.nextDouble();
         double currentSwingRatio = 0.7 * random.nextDouble();
         double nextTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         swingSplitFractions.get(0).set(currentSwingRatio);
         transferSplitFractions.get(1).set(nextTransferRatio);

         double currentDoubleSupportDuration = 2.0 * random.nextDouble();
         double currentSingleSupportDuration = 5.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(currentDoubleSupportDuration);
         singleSupportDurations.get(0).set(currentSingleSupportDuration);
         doubleSupportDurations.get(1).set(2.0 * random.nextDouble());

         totalTrajectoryTime.set(currentSingleSupportDuration);

         boolean isInTransfer = true;
         boolean useTwoCMPs = true;

         double timeInCurrentState = random.nextDouble() * currentDoubleSupportDuration;

         positionMatrix.setSegmentDuration(currentDoubleSupportDuration);
         velocityMatrix.setSegmentDuration(currentDoubleSupportDuration);
         positionMatrix.update(timeInCurrentState);
         velocityMatrix.update(timeInCurrentState);

         entryCMPMatrix.compute(1, singleSupportDurations, doubleSupportDurations, useTwoCMPs, omega);

         CommonOps.mult(positionMatrix, entryCMPMatrix, position);
         CommonOps.mult(velocityMatrix, entryCMPMatrix, velocity);

         entryCMPCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(position.get(0, 0), entryCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(velocity.get(0, 0), entryCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(entryCMPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(entryCMPCurrentMultiplier.getVelocityMultiplier()));
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

      EntryCMPCurrentMultiplier entryCMPCurrentMultiplier = new EntryCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, totalTrajectoryTime, "", true, false, blendingFraction, minimumBlendingTime, registry);

      TransferEntryCMPMatrix entryCMPMatrix = new TransferEntryCMPMatrix(swingSplitFractions, transferSplitFractions);

      CubicMatrix positionMatrix = new CubicMatrix();
      CubicDerivativeMatrix velocityMatrix = new CubicDerivativeMatrix();
      DenseMatrix64F position = new DenseMatrix64F(1, 1);
      DenseMatrix64F velocity = new DenseMatrix64F(1, 1);

      for (int iter = 0; iter < iters; iter++)
      {
         double currentTransferRatio = 0.7 * random.nextDouble();
         double nextTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         transferSplitFractions.get(1).set(nextTransferRatio);

         double currentDoubleSupportDuration = 2.0 * random.nextDouble();
         double currentSingleSupportDuration = 5.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(currentDoubleSupportDuration);
         singleSupportDurations.get(0).set(currentSingleSupportDuration);
         doubleSupportDurations.get(1).set(2.0 * random.nextDouble());

         totalTrajectoryTime.set(currentSingleSupportDuration);

         boolean isInTransfer = true;
         boolean useTwoCMPs = false;

         double timeInCurrentState = random.nextDouble() * currentDoubleSupportDuration;

         positionMatrix.setSegmentDuration(currentDoubleSupportDuration);
         velocityMatrix.setSegmentDuration(currentDoubleSupportDuration);
         positionMatrix.update(timeInCurrentState);
         velocityMatrix.update(timeInCurrentState);

         entryCMPMatrix.compute(1, singleSupportDurations, doubleSupportDurations, useTwoCMPs, omega);

         CommonOps.mult(positionMatrix, entryCMPMatrix, position);
         CommonOps.mult(velocityMatrix, entryCMPMatrix, velocity);

         entryCMPCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(position.get(0, 0), entryCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(velocity.get(0, 0), entryCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(entryCMPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(entryCMPCurrentMultiplier.getVelocityMultiplier()));
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

      EntryCMPCurrentMultiplier entryCMPCurrentMultiplier = new EntryCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, totalTrajectoryTime, "", true, true, blendingFraction, minimumBlendingTime, registry);

      TransferEntryCMPMatrix entryCMPMatrix = new TransferEntryCMPMatrix(swingSplitFractions, transferSplitFractions);

      CubicMatrix positionMatrix = new CubicMatrix();
      CubicDerivativeMatrix velocityMatrix = new CubicDerivativeMatrix();
      DenseMatrix64F position = new DenseMatrix64F(1, 1);
      DenseMatrix64F velocity = new DenseMatrix64F(1, 1);

      for (int iter = 0; iter < iters; iter++)
      {
         double currentTransferRatio = 0.7 * random.nextDouble();
         double nextTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         transferSplitFractions.get(1).set(nextTransferRatio);

         double currentDoubleSupportDuration = 2.0 * random.nextDouble();
         double currentSingleSupportDuration = 5.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(currentDoubleSupportDuration);
         singleSupportDurations.get(0).set(currentSingleSupportDuration);
         doubleSupportDurations.get(1).set(2.0 * random.nextDouble());

         totalTrajectoryTime.set(currentSingleSupportDuration);

         boolean isInTransfer = true;
         boolean useTwoCMPs = false;

         double timeInCurrentState = random.nextDouble() * currentDoubleSupportDuration;

         positionMatrix.setSegmentDuration(currentDoubleSupportDuration);
         velocityMatrix.setSegmentDuration(currentDoubleSupportDuration);
         positionMatrix.update(timeInCurrentState);
         velocityMatrix.update(timeInCurrentState);

         entryCMPMatrix.compute(1, singleSupportDurations, doubleSupportDurations, useTwoCMPs, omega);

         CommonOps.mult(positionMatrix, entryCMPMatrix, position);
         CommonOps.mult(velocityMatrix, entryCMPMatrix, velocity);

         entryCMPCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(position.get(0, 0), entryCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(velocity.get(0, 0), entryCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(entryCMPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(entryCMPCurrentMultiplier.getVelocityMultiplier()));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationProjectForwardTwoCMPFirstSegment()
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

      EntryCMPCurrentMultiplier entryCMPCurrentMultiplier = new EntryCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, totalTrajectoryTime, "", true, false, blendingFraction, minimumBlendingTime, registry);


      for (int iter = 0; iter < iters; iter++)
      {
         double currentTransferRatio = 0.7 * random.nextDouble();
         double currentSwingRatio = 0.7 * random.nextDouble();
         double nextTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         swingSplitFractions.get(0).set(currentSwingRatio);
         transferSplitFractions.get(1).set(nextTransferRatio);

         double currentDoubleSupportDuration = 2.0 * random.nextDouble();
         double currentSingleSupportDuration = 5.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(currentDoubleSupportDuration);
         singleSupportDurations.get(0).set(currentSingleSupportDuration);
         doubleSupportDurations.get(1).set(2.0 * random.nextDouble());

         double minimumSplineTime = Math.min(currentSingleSupportDuration, 0.5);
         double startOfSpline = 0.2 * random.nextDouble();
         double endOfSpline = currentSingleSupportDuration - 0.2 * random.nextDouble();
         if (minimumSplineTime > endOfSpline - startOfSpline)
            startOfSpline = 0.0;
         if (minimumSplineTime > endOfSpline - startOfSpline)
            endOfSpline = currentSingleSupportDuration;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(currentSingleSupportDuration);

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;


         double timeInCurrentState = random.nextDouble() * startOfSpline;

         entryCMPCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         double projection = Math.exp(omega * timeInCurrentState);
         Assert.assertEquals(1.0 - projection, entryCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(- omega * projection, entryCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(entryCMPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(entryCMPCurrentMultiplier.getVelocityMultiplier()));
      }
   }


   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationProjectForwardTwoCMPFirstSegmentWithBlending()
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

      EntryCMPCurrentMultiplier entryCMPCurrentMultiplier = new EntryCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, totalTrajectoryTime, "", true, true, blendingFraction, minimumBlendingTime, registry);


      for (int iter = 0; iter < iters; iter++)
      {
         double currentTransferRatio = 0.7 * random.nextDouble();
         double currentSwingRatio = 0.7 * random.nextDouble();
         double nextTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         swingSplitFractions.get(0).set(currentSwingRatio);
         transferSplitFractions.get(1).set(nextTransferRatio);

         double currentDoubleSupportDuration = 2.0 * random.nextDouble();
         double currentSingleSupportDuration = 5.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(currentDoubleSupportDuration);
         singleSupportDurations.get(0).set(currentSingleSupportDuration);
         doubleSupportDurations.get(1).set(2.0 * random.nextDouble());

         double minimumSplineTime = Math.min(currentSingleSupportDuration, 0.5);
         double startOfSpline = 0.2 * random.nextDouble();
         double endOfSpline = currentSingleSupportDuration - 0.2 * random.nextDouble();
         if (minimumSplineTime > endOfSpline - startOfSpline)
            startOfSpline = 0.0;
         if (minimumSplineTime > endOfSpline - startOfSpline)
            endOfSpline = currentSingleSupportDuration;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(currentSingleSupportDuration);

         double swingOnEntryCMP = currentSwingRatio * currentSingleSupportDuration;

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         double timeInCurrentState = random.nextDouble() * startOfSpline;

         entryCMPCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         double initialProjection = 1.0 - Math.exp(omega * timeInCurrentState);
         double recursiveProjection = 1.0 - Math.exp(omega * (timeInCurrentState - swingOnEntryCMP));

         double initialVelocityProjection = omega * (initialProjection - 1.0);
         double recursiveVelocityProjection = omega * (recursiveProjection - 1.0);

         double blendingTime = Math.max(blendingFraction * startOfSpline, minimumBlendingTime);
         double phaseInState = MathTools.clamp(timeInCurrentState / blendingTime, 0.0, 1.0);

         double positionMultiplier = InterpolationTools.linearInterpolate(initialProjection, recursiveProjection, phaseInState);
         double velocityMultiplier = InterpolationTools.linearInterpolate(initialVelocityProjection, recursiveVelocityProjection, phaseInState);

         String name = "iter = " + iter + ", blendingTime = " + blendingTime + ", alpha = " + phaseInState;

         Assert.assertEquals(name, positionMultiplier, entryCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(name, velocityMultiplier, entryCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(name, Double.isNaN(entryCMPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(name, Double.isNaN(entryCMPCurrentMultiplier.getVelocityMultiplier()));
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

      EntryCMPCurrentMultiplier entryCMPCurrentMultiplier = new EntryCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, totalTrajectoryTime, "", true, false, blendingFraction, minimumBlendingTime, registry);

      SwingEntryCMPMatrix entryCMPMatrix = new SwingEntryCMPMatrix(swingSplitFractions, startOfSplineTime, false, minimumBlendingTime);
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

         double currentDoubleSupportDuration = 2.0 * random.nextDouble();
         double currentSingleSupportDuration = 5.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(currentDoubleSupportDuration);
         singleSupportDurations.get(0).set(currentSingleSupportDuration);
         doubleSupportDurations.get(1).set(2.0 * random.nextDouble());

         double minimumSplineTime = Math.min(currentSingleSupportDuration, 0.5);
         double startOfSpline = 0.2 * random.nextDouble();
         double endOfSpline = currentSingleSupportDuration - 0.2 * random.nextDouble();
         if (minimumSplineTime > endOfSpline - startOfSpline)
            startOfSpline = 0.0;
         if (minimumSplineTime > endOfSpline - startOfSpline)
            endOfSpline = currentSingleSupportDuration;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(currentSingleSupportDuration);

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         double timeInCurrentState = random.nextDouble() * (endOfSpline - startOfSpline) + startOfSpline;

         cubicMatrix.setSegmentDuration(endOfSpline - startOfSpline);
         cubicDerivativeMatrix.setSegmentDuration(endOfSpline - startOfSpline);

         cubicMatrix.update(timeInCurrentState - startOfSpline);
         cubicDerivativeMatrix.update(timeInCurrentState - startOfSpline);

         entryCMPMatrix.compute(singleSupportDurations, omega);
         CommonOps.mult(cubicMatrix, entryCMPMatrix, positionMatrixOut);
         CommonOps.mult(cubicDerivativeMatrix, entryCMPMatrix, velocityMatrixOut);

         entryCMPCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(positionMatrixOut.get(0, 0), entryCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(velocityMatrixOut.get(0, 0), entryCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);

         Assert.assertFalse(Double.isNaN(entryCMPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(entryCMPCurrentMultiplier.getVelocityMultiplier()));
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

      EntryCMPCurrentMultiplier entryCMPCurrentMultiplier = new EntryCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, totalTrajectoryTime, "", true, true, blendingFraction, minimumBlendingTime, registry);

      SwingEntryCMPMatrix entryCMPMatrix = new SwingEntryCMPMatrix(swingSplitFractions, startOfSplineTime, true, minimumBlendingTime);
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

         double currentDoubleSupportDuration = 2.0 * random.nextDouble();
         double currentSingleSupportDuration = 5.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(currentDoubleSupportDuration);
         singleSupportDurations.get(0).set(currentSingleSupportDuration);
         doubleSupportDurations.get(1).set(2.0 * random.nextDouble());

         double minimumSplineTime = Math.min(currentSingleSupportDuration, 0.5);
         double startOfSpline = 0.2 * random.nextDouble();
         double endOfSpline = currentSingleSupportDuration - 0.2 * random.nextDouble();
         if (minimumSplineTime > endOfSpline - startOfSpline)
            startOfSpline = 0.0;
         if (minimumSplineTime > endOfSpline - startOfSpline)
            endOfSpline = currentSingleSupportDuration;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(currentSingleSupportDuration);

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         double timeInCurrentState = random.nextDouble() * (endOfSpline - startOfSpline) + startOfSpline;

         cubicMatrix.setSegmentDuration(endOfSpline - startOfSpline);
         cubicDerivativeMatrix.setSegmentDuration(endOfSpline - startOfSpline);

         cubicMatrix.update(timeInCurrentState - startOfSpline);
         cubicDerivativeMatrix.update(timeInCurrentState - startOfSpline);

         entryCMPMatrix.compute(singleSupportDurations, omega);
         CommonOps.mult(cubicMatrix, entryCMPMatrix, positionMatrixOut);
         CommonOps.mult(cubicDerivativeMatrix, entryCMPMatrix, velocityMatrixOut);

         entryCMPCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(positionMatrixOut.get(0, 0), entryCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(velocityMatrixOut.get(0, 0), entryCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);

         Assert.assertFalse(Double.isNaN(entryCMPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(entryCMPCurrentMultiplier.getVelocityMultiplier()));
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

      EntryCMPCurrentMultiplier entryCMPCurrentMultiplier = new EntryCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, totalTrajectoryTime, "", true, false, blendingFraction, minimumBlendingTime, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double currentTransferRatio = 0.7 * random.nextDouble();
         double currentSwingRatio = 0.7 * random.nextDouble();
         double nextTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         swingSplitFractions.get(0).set(currentSwingRatio);
         transferSplitFractions.get(1).set(nextTransferRatio);

         double currentDoubleSupportDuration = 2.0 * random.nextDouble();
         double currentSingleSupportDuration = 5.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(currentDoubleSupportDuration);
         singleSupportDurations.get(0).set(currentSingleSupportDuration);
         doubleSupportDurations.get(1).set(2.0 * random.nextDouble());

         double minimumSplineTime = Math.min(currentSingleSupportDuration, 0.5);
         double startOfSpline = 0.2 * random.nextDouble();
         double endOfSpline = currentSingleSupportDuration - 0.2 * random.nextDouble();
         if (minimumSplineTime > endOfSpline - startOfSpline)
            startOfSpline = 0.0;
         if (minimumSplineTime > endOfSpline - startOfSpline)
            endOfSpline = currentSingleSupportDuration;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(currentSingleSupportDuration);

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         double timeInCurrentState = random.nextDouble() * (currentSingleSupportDuration - endOfSpline) + endOfSpline;

         entryCMPCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(0.0, entryCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(0.0, entryCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);

         Assert.assertFalse(Double.isNaN(entryCMPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(entryCMPCurrentMultiplier.getVelocityMultiplier()));
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

      EntryCMPCurrentMultiplier entryCMPCurrentMultiplier = new EntryCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, totalTrajectoryTime, "", true, true, blendingFraction, minimumBlendingTime, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double currentTransferRatio = 0.7 * random.nextDouble();
         double currentSwingRatio = 0.7 * random.nextDouble();
         double nextTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         swingSplitFractions.get(0).set(currentSwingRatio);
         transferSplitFractions.get(1).set(nextTransferRatio);

         double currentDoubleSupportDuration = 2.0 * random.nextDouble();
         double currentSingleSupportDuration = 5.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(currentDoubleSupportDuration);
         singleSupportDurations.get(0).set(currentSingleSupportDuration);
         doubleSupportDurations.get(1).set(2.0 * random.nextDouble());

         double minimumSplineTime = Math.min(currentSingleSupportDuration, 0.5);
         double startOfSpline = 0.2 * random.nextDouble();
         double endOfSpline = currentSingleSupportDuration - 0.2 * random.nextDouble();
         if (minimumSplineTime > endOfSpline - startOfSpline)
            startOfSpline = 0.0;
         if (minimumSplineTime > endOfSpline - startOfSpline)
            endOfSpline = currentSingleSupportDuration;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(currentSingleSupportDuration);

         boolean isInTransfer = false;
         boolean useTwoCMPs = true;

         double timeInCurrentState = random.nextDouble() * (currentSingleSupportDuration - endOfSpline) + endOfSpline;

         entryCMPCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(0.0, entryCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(0.0, entryCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);

         Assert.assertFalse(Double.isNaN(entryCMPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(entryCMPCurrentMultiplier.getVelocityMultiplier()));
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

      EntryCMPCurrentMultiplier entryCMPCurrentMultiplier = new EntryCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, totalTrajectoryTime, "", true, false, blendingFraction, minimumBlendingTime, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double currentTransferRatio = 0.7 * random.nextDouble();
         double nextTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         transferSplitFractions.get(1).set(nextTransferRatio);

         double currentDoubleSupportDuration = 2.0 * random.nextDouble();
         double currentSingleSupportDuration = 5.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(currentDoubleSupportDuration);
         singleSupportDurations.get(0).set(currentSingleSupportDuration);
         doubleSupportDurations.get(1).set(2.0 * random.nextDouble());

         double minimumSplineTime = Math.min(currentSingleSupportDuration, 0.5);
         double startOfSpline = 0.2 * random.nextDouble();
         double endOfSpline = currentSingleSupportDuration - 0.2 * random.nextDouble();
         if (minimumSplineTime > endOfSpline - startOfSpline)
            startOfSpline = 0.0;
         if (minimumSplineTime > endOfSpline - startOfSpline)
            endOfSpline = currentSingleSupportDuration;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(currentSingleSupportDuration);

         boolean isInTransfer = false;
         boolean useTwoCMPs = false;

         double timeInCurrentState = random.nextDouble() * currentSingleSupportDuration;

         double timeRemaining = currentSingleSupportDuration - timeInCurrentState;
         double nextTransferOnCMP = transferSplitFractions.get(1).getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();

         double projectionTime = timeRemaining + nextTransferOnCMP;

         double positionMultiplier = 1.0 - Math.exp(-omega * projectionTime);
         double velocityMultiplier = -omega * Math.exp(-omega * projectionTime);

         entryCMPCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(positionMultiplier, entryCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(velocityMultiplier, entryCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);

         Assert.assertFalse(Double.isNaN(entryCMPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(entryCMPCurrentMultiplier.getVelocityMultiplier()));

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

      EntryCMPCurrentMultiplier entryCMPCurrentMultiplier = new EntryCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, totalTrajectoryTime, "", true, true, blendingFraction, minimumBlendingTime, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double currentTransferRatio = 0.7 * random.nextDouble();
         double nextTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         transferSplitFractions.get(1).set(nextTransferRatio);

         double currentDoubleSupportDuration = 2.0 * random.nextDouble();
         double currentSingleSupportDuration = 5.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(currentDoubleSupportDuration);
         singleSupportDurations.get(0).set(currentSingleSupportDuration);
         doubleSupportDurations.get(1).set(2.0 * random.nextDouble());

         double minimumSplineTime = Math.min(currentSingleSupportDuration, 0.5);
         double startOfSpline = 0.2 * random.nextDouble();
         double endOfSpline = currentSingleSupportDuration - 0.2 * random.nextDouble();
         if (minimumSplineTime > endOfSpline - startOfSpline)
            startOfSpline = 0.0;
         if (minimumSplineTime > endOfSpline - startOfSpline)
            endOfSpline = currentSingleSupportDuration;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(currentSingleSupportDuration);

         boolean isInTransfer = false;
         boolean useTwoCMPs = false;

         double timeInCurrentState = random.nextDouble() * currentSingleSupportDuration;

         double timeRemaining = currentSingleSupportDuration - timeInCurrentState;
         double nextTransferOnCMP = transferSplitFractions.get(1).getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();

         double projectionTime = timeRemaining + nextTransferOnCMP;

         double recursionMultiplier = 1.0 - Math.exp(-omega * projectionTime);
         double projectionMultiplier = 1.0 - Math.exp(omega * timeInCurrentState);

         double recursionVelocityMultiplier = -omega * Math.exp(-omega * projectionTime);
         double projectionVelocityMultiplier = -omega * Math.exp(omega * timeInCurrentState);

         double blendingTime = Math.max(blendingFraction * singleSupportDurations.get(0).getDoubleValue(), minimumBlendingTime);
         double phaseInState = MathTools.clamp(timeInCurrentState / blendingTime, 0.0, 1.0);

         double positionMultiplier = InterpolationTools.linearInterpolate(projectionMultiplier, recursionMultiplier, phaseInState);
         double velocityMultiplier = InterpolationTools.linearInterpolate(projectionVelocityMultiplier, recursionVelocityMultiplier, phaseInState);

         entryCMPCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(positionMultiplier, entryCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(velocityMultiplier, entryCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);

         Assert.assertFalse(Double.isNaN(entryCMPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(entryCMPCurrentMultiplier.getVelocityMultiplier()));

      }
   }
}
