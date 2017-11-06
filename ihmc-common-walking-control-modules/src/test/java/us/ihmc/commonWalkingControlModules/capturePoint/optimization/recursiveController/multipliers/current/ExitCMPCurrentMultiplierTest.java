package us.ihmc.commonWalkingControlModules.capturePoint.optimization.recursiveController.multipliers.current;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.current.ExitCMPCurrentMultiplier;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.interpolation.CubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.interpolation.CubicMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.stateMatrices.swing.SwingExitCMPMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.stateMatrices.transfer.TransferExitCMPMatrix;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.InterpolationTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class ExitCMPCurrentMultiplierTest
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

      ExitCMPCurrentMultiplier exitCMPCurrentMultiplier = new ExitCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, "", true, false, blendingFraction, minimumBlendingTime, registry);

      TransferExitCMPMatrix exitCMPMatrix = new TransferExitCMPMatrix(swingSplitFractions, transferSplitFractions);
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

         totalTrajectoryTime.set(currentSingleSupportDuration);

         boolean isInTransfer = true;
         boolean useTwoCMPs = true;

         double timeInCurrentState = random.nextDouble() * currentDoubleSupportDuration;

         cubicMatrix.setSegmentDuration(currentDoubleSupportDuration);
         cubicDerivativeMatrix.setSegmentDuration(currentDoubleSupportDuration);

         cubicMatrix.update(timeInCurrentState);
         cubicDerivativeMatrix.update(timeInCurrentState);

         exitCMPMatrix.compute(1, singleSupportDurations, doubleSupportDurations, useTwoCMPs, omega);
         CommonOps.mult(cubicMatrix, exitCMPMatrix, positionMatrixOut);
         CommonOps.mult(cubicDerivativeMatrix, exitCMPMatrix, velocityMatrixOut);

         exitCMPCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(positionMatrixOut.get(0, 0), exitCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(velocityMatrixOut.get(0, 0), exitCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(exitCMPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(exitCMPCurrentMultiplier.getVelocityMultiplier()));
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

      ExitCMPCurrentMultiplier exitCMPCurrentMultiplier = new ExitCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, "", true, true, blendingFraction, minimumBlendingTime, registry);

      TransferExitCMPMatrix exitCMPMatrix = new TransferExitCMPMatrix(swingSplitFractions, transferSplitFractions);
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

         totalTrajectoryTime.set(currentSingleSupportDuration);

         boolean isInTransfer = true;
         boolean useTwoCMPs = true;

         double timeInCurrentState = random.nextDouble() * currentDoubleSupportDuration;

         cubicMatrix.setSegmentDuration(currentDoubleSupportDuration);
         cubicDerivativeMatrix.setSegmentDuration(currentDoubleSupportDuration);

         cubicMatrix.update(timeInCurrentState);
         cubicDerivativeMatrix.update(timeInCurrentState);

         exitCMPMatrix.compute(1, singleSupportDurations, doubleSupportDurations, useTwoCMPs, omega);
         CommonOps.mult(cubicMatrix, exitCMPMatrix, positionMatrixOut);
         CommonOps.mult(cubicDerivativeMatrix, exitCMPMatrix, velocityMatrixOut);

         exitCMPCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(positionMatrixOut.get(0, 0), exitCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(velocityMatrixOut.get(0, 0), exitCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(exitCMPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(exitCMPCurrentMultiplier.getVelocityMultiplier()));
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

      ExitCMPCurrentMultiplier exitCMPCurrentMultiplier = new ExitCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, "", true, false, blendingFraction, minimumBlendingTime, registry);

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

         exitCMPCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(0.0, exitCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(0.0, exitCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(exitCMPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(exitCMPCurrentMultiplier.getVelocityMultiplier()));
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

      ExitCMPCurrentMultiplier exitCMPCurrentMultiplier = new ExitCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, "", true, true, blendingFraction, minimumBlendingTime, registry);

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

         exitCMPCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(0.0, exitCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(0.0, exitCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(exitCMPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(exitCMPCurrentMultiplier.getVelocityMultiplier()));
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

      ExitCMPCurrentMultiplier exitCMPCurrentMultiplier = new ExitCMPCurrentMultiplier(swingSplitFractions, doubleSupportDurations, startOfSplineTime,
            endOfSplineTime, "", true, false, blendingFraction, minimumBlendingTime, registry);


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

         exitCMPCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(0.0, exitCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(0.0, exitCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(exitCMPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(exitCMPCurrentMultiplier.getVelocityMultiplier()));
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

      ExitCMPCurrentMultiplier exitCMPCurrentMultiplier = new ExitCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, "", true, true, blendingFraction, minimumBlendingTime, registry);


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

         exitCMPCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         double nextTransferOnExit = nextTransferRatio * doubleSupportDurations.get(1).getDoubleValue();
         double swingOnExit = (1.0 - currentSwingRatio) * currentSingleSupportDuration;
         double timeOnExit = nextTransferOnExit + swingOnExit;

         double swingOnEntry = currentSwingRatio * currentSingleSupportDuration;

         double recursion = Math.exp(omega * (timeInCurrentState - swingOnEntry)) * (1.0 - Math.exp(-omega * timeOnExit));

         double blendingTime = Math.max(blendingFraction * startOfSpline, minimumBlendingTime);
         double alpha = MathTools.clamp(timeInCurrentState / blendingTime, 0.0, 1.0);

         double positionMultiplier = InterpolationTools.linearInterpolate(0.0, recursion, alpha);


         Assert.assertEquals(positionMultiplier, exitCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(omega * positionMultiplier, exitCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(exitCMPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(exitCMPCurrentMultiplier.getVelocityMultiplier()));
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

      ExitCMPCurrentMultiplier exitCMPCurrentMultiplier = new ExitCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, "", true, false, blendingFraction, minimumBlendingTime, registry);

      SwingExitCMPMatrix exitCMPMatrix = new SwingExitCMPMatrix(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime, false, minimumBlendingTime);
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

         double splineDuration = endOfSpline - startOfSpline;
         cubicMatrix.setSegmentDuration(splineDuration);
         cubicDerivativeMatrix.setSegmentDuration(splineDuration);

         cubicMatrix.update(timeInCurrentState - startOfSpline);
         cubicDerivativeMatrix.update(timeInCurrentState - startOfSpline);

         exitCMPMatrix.compute(singleSupportDurations, doubleSupportDurations, omega);
         CommonOps.mult(cubicMatrix, exitCMPMatrix, positionMatrixOut);
         CommonOps.mult(cubicDerivativeMatrix, exitCMPMatrix, velocityMatrixOut);

         exitCMPCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(positionMatrixOut.get(0, 0), exitCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(velocityMatrixOut.get(0, 0), exitCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(exitCMPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(exitCMPCurrentMultiplier.getVelocityMultiplier()));
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

      ExitCMPCurrentMultiplier exitCMPCurrentMultiplier = new ExitCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, "", true, true, blendingFraction, minimumBlendingTime, registry);

      SwingExitCMPMatrix exitCMPMatrix = new SwingExitCMPMatrix(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime, true, minimumBlendingTime);
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

         double splineDuration = endOfSpline - startOfSpline;
         cubicMatrix.setSegmentDuration(splineDuration);
         cubicDerivativeMatrix.setSegmentDuration(splineDuration);

         cubicMatrix.update(timeInCurrentState - startOfSpline);
         cubicDerivativeMatrix.update(timeInCurrentState - startOfSpline);

         exitCMPMatrix.compute(singleSupportDurations, doubleSupportDurations, omega);
         CommonOps.mult(cubicMatrix, exitCMPMatrix, positionMatrixOut);
         CommonOps.mult(cubicDerivativeMatrix, exitCMPMatrix, velocityMatrixOut);

         exitCMPCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(positionMatrixOut.get(0, 0), exitCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(velocityMatrixOut.get(0, 0), exitCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(exitCMPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(exitCMPCurrentMultiplier.getVelocityMultiplier()));
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

      ExitCMPCurrentMultiplier exitCMPCurrentMultiplier = new ExitCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, "", true, false, blendingFraction, minimumBlendingTime, registry);

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
         double nextDoubleSupportDuration = 2.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(currentDoubleSupportDuration);
         singleSupportDurations.get(0).set(currentSingleSupportDuration);
         doubleSupportDurations.get(1).set(nextDoubleSupportDuration);

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

         double nextTransferOnExit = nextTransferRatio * nextDoubleSupportDuration;

         exitCMPCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         double timeRemaining = currentSingleSupportDuration - timeInCurrentState;
         double projectionTime = nextTransferOnExit + timeRemaining;
         double projection = Math.exp(-omega * projectionTime);

         Assert.assertEquals(1.0 - projection, exitCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(-omega * projection, exitCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(exitCMPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(exitCMPCurrentMultiplier.getVelocityMultiplier()));
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

      ExitCMPCurrentMultiplier exitCMPCurrentMultiplier = new ExitCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, "", true, true, blendingFraction, minimumBlendingTime, registry);

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
         double nextDoubleSupportDuration = 2.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(currentDoubleSupportDuration);
         singleSupportDurations.get(0).set(currentSingleSupportDuration);
         doubleSupportDurations.get(1).set(nextDoubleSupportDuration);

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

         double nextTransferOnExit = nextTransferRatio * nextDoubleSupportDuration;

         exitCMPCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         double timeRemaining = currentSingleSupportDuration - timeInCurrentState;
         double projectionTime = nextTransferOnExit + timeRemaining;
         double projection = Math.exp(-omega * projectionTime);

         Assert.assertEquals(1.0 - projection, exitCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(-omega * projection, exitCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(exitCMPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(exitCMPCurrentMultiplier.getVelocityMultiplier()));
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

      ExitCMPCurrentMultiplier exitCMPCurrentMultiplier = new ExitCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, "", true, false, blendingFraction, minimumBlendingTime, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double currentTransferRatio = 0.7 * random.nextDouble();
         double nextTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         transferSplitFractions.get(1).set(nextTransferRatio);

         double currentDoubleSupportDuration = 2.0 * random.nextDouble();
         double currentSingleSupportDuration = 5.0 * random.nextDouble();
         double nextDoubleSupportDuration = 2.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(currentDoubleSupportDuration);
         singleSupportDurations.get(0).set(currentSingleSupportDuration);
         doubleSupportDurations.get(1).set(nextDoubleSupportDuration);

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

         double timeInCurrentState = random.nextDouble() * (currentSingleSupportDuration - endOfSpline) + endOfSpline;

         exitCMPCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(0.0, exitCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(0.0, exitCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(exitCMPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(exitCMPCurrentMultiplier.getVelocityMultiplier()));
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

      ExitCMPCurrentMultiplier exitCMPCurrentMultiplier = new ExitCMPCurrentMultiplier(swingSplitFractions, transferSplitFractions, startOfSplineTime,
            endOfSplineTime, "", true, true, blendingFraction, minimumBlendingTime, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double currentTransferRatio = 0.7 * random.nextDouble();
         double nextTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         transferSplitFractions.get(1).set(nextTransferRatio);

         double currentDoubleSupportDuration = 2.0 * random.nextDouble();
         double currentSingleSupportDuration = 5.0 * random.nextDouble();
         double nextDoubleSupportDuration = 2.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(currentDoubleSupportDuration);
         singleSupportDurations.get(0).set(currentSingleSupportDuration);
         doubleSupportDurations.get(1).set(nextDoubleSupportDuration);

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

         double timeInCurrentState = random.nextDouble() * (currentSingleSupportDuration - endOfSpline) + endOfSpline;

         exitCMPCurrentMultiplier.compute(1, singleSupportDurations, doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(0.0, exitCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(0.0, exitCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(exitCMPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(exitCMPCurrentMultiplier.getVelocityMultiplier()));
      }
   }
}
