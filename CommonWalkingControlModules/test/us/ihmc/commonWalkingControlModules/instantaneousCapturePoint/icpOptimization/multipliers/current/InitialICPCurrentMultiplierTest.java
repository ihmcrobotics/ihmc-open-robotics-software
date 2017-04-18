package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.CubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.CubicMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing.SwingInitialICPMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.transfer.TransferInitialICPMatrix;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class InitialICPCurrentMultiplierTest
{
   private static final double epsilon = 0.0001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationTwoCMPTransfer()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      double omega = 3.0;
      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      List<DoubleYoVariable> transferSplitFractions = new ArrayList<>();
      List<DoubleYoVariable> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration" + i, registry);
         DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         DoubleYoVariable transferSplitRatio = new DoubleYoVariable("transferSplitRatio" + i, registry);
         DoubleYoVariable swingSplitRatio = new DoubleYoVariable("swingSplitRatio" + i, registry);
         transferSplitRatio.setToNaN();
         swingSplitRatio.setToNaN();
         transferSplitFractions.add(transferSplitRatio);
         swingSplitFractions.add(swingSplitRatio);
      }

      InitialICPCurrentMultiplier initialICPCurrentMultiplier = new InitialICPCurrentMultiplier(startOfSplineTime, endOfSplineTime, "", registry);

      TransferInitialICPMatrix initialICPMatrix = new TransferInitialICPMatrix();

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
         double nextDoubleSupportDuration = 2.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(currentDoubleSupportDuration);
         singleSupportDurations.get(0).set(currentSingleSupportDuration);
         doubleSupportDurations.get(1).set(nextDoubleSupportDuration);

         double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();

         totalTrajectoryTime.set(singleSupportDuration);

         boolean isInTransfer = true;
         boolean useTwoCMPs = true;

         double doubleSupportDuration = doubleSupportDurations.get(0).getDoubleValue();

         double timeInCurrentState = random.nextDouble() * doubleSupportDuration;

         positionMatrix.setSegmentDuration(doubleSupportDuration);
         velocityMatrix.setSegmentDuration(doubleSupportDuration);
         positionMatrix.update(timeInCurrentState);
         velocityMatrix.update(timeInCurrentState);

         initialICPMatrix.compute();

         CommonOps.mult(positionMatrix, initialICPMatrix, position);
         CommonOps.mult(velocityMatrix, initialICPMatrix, velocity);

         initialICPCurrentMultiplier.compute(doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(position.get(0, 0), initialICPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(velocity.get(0, 0), initialICPCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(initialICPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(initialICPCurrentMultiplier.getVelocityMultiplier()));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationOneCMPTransfer()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      double omega = 3.0;
      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      List<DoubleYoVariable> transferSplitFractions = new ArrayList<>();
      List<DoubleYoVariable> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration" + i, registry);
         DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         DoubleYoVariable transferSplitRatio = new DoubleYoVariable("transferSplitRatio" + i, registry);
         transferSplitRatio.setToNaN();
         transferSplitFractions.add(transferSplitRatio);
      }

      InitialICPCurrentMultiplier initialICPCurrentMultiplier = new InitialICPCurrentMultiplier(startOfSplineTime, endOfSplineTime, "", registry);

      TransferInitialICPMatrix initialICPMatrix = new TransferInitialICPMatrix();

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
         double nextDoubleSupportDuration = 2.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(currentDoubleSupportDuration);
         singleSupportDurations.get(0).set(currentSingleSupportDuration);
         doubleSupportDurations.get(1).set(nextDoubleSupportDuration);

         double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();

         totalTrajectoryTime.set(singleSupportDuration);

         boolean isInTransfer = true;
         boolean useTwoCMPs = false;

         double doubleSupportDuration = doubleSupportDurations.get(0).getDoubleValue();

         double timeInCurrentState = random.nextDouble() * doubleSupportDuration;

         positionMatrix.setSegmentDuration(doubleSupportDuration);
         velocityMatrix.setSegmentDuration(doubleSupportDuration);
         positionMatrix.update(timeInCurrentState);
         velocityMatrix.update(timeInCurrentState);

         initialICPMatrix.compute();

         CommonOps.mult(positionMatrix, initialICPMatrix, position);
         CommonOps.mult(velocityMatrix, initialICPMatrix, velocity);

         initialICPCurrentMultiplier.compute(doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(position.get(0, 0), initialICPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(velocity.get(0, 0), initialICPCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(initialICPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(initialICPCurrentMultiplier.getVelocityMultiplier()));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationProjectForwardTwoCMPFirstSegment()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      double omega = 3.0;
      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      List<DoubleYoVariable> transferSplitFractions = new ArrayList<>();
      List<DoubleYoVariable> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration" + i, registry);
         DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         DoubleYoVariable transferSplitRatio = new DoubleYoVariable("transferSplitRatio" + i, registry);
         DoubleYoVariable swingSplitRatio = new DoubleYoVariable("swingSplitRatio" + i, registry);
         transferSplitRatio.setToNaN();
         swingSplitRatio.setToNaN();
         transferSplitFractions.add(transferSplitRatio);
         swingSplitFractions.add(swingSplitRatio);
      }

      InitialICPCurrentMultiplier initialICPCurrentMultiplier = new InitialICPCurrentMultiplier(startOfSplineTime, endOfSplineTime, "", registry);

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

         initialICPCurrentMultiplier.compute(doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         double projection = Math.exp(omega * timeInCurrentState);
         Assert.assertEquals(projection, initialICPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(omega * projection, initialICPCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(initialICPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(initialICPCurrentMultiplier.getVelocityMultiplier()));
      }
   }


   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationTwoCMPSecondSegment()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      double omega = 3.0;
      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      List<DoubleYoVariable> transferSplitFractions = new ArrayList<>();
      List<DoubleYoVariable> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration" + i, registry);
         DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         DoubleYoVariable transferSplitRatio = new DoubleYoVariable("transferSplitRatio" + i, registry);
         DoubleYoVariable swingSplitRatio = new DoubleYoVariable("swingSplitRatio" + i, registry);
         transferSplitRatio.setToNaN();
         swingSplitRatio.setToNaN();
         transferSplitFractions.add(transferSplitRatio);
         swingSplitFractions.add(swingSplitRatio);
      }

      InitialICPCurrentMultiplier initialICPCurrentMultiplier = new InitialICPCurrentMultiplier(startOfSplineTime, endOfSplineTime, "", registry);

      SwingInitialICPMatrix initialICPMatrix = new SwingInitialICPMatrix(startOfSplineTime);

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
         double nextDoubleSupportDuration = 2.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(currentDoubleSupportDuration);
         singleSupportDurations.get(0).set(currentSingleSupportDuration);
         doubleSupportDurations.get(1).set(nextDoubleSupportDuration);

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

         cubicMatrix.setSegmentDuration(endOfSpline - startOfSpline);
         cubicDerivativeMatrix.setSegmentDuration(endOfSpline - startOfSpline);

         cubicMatrix.update(timeInCurrentState - startOfSpline);
         cubicDerivativeMatrix.update(timeInCurrentState - startOfSpline);

         initialICPMatrix.compute(omega);
         CommonOps.mult(cubicMatrix, initialICPMatrix, positionMatrixOut);
         CommonOps.mult(cubicDerivativeMatrix, initialICPMatrix, velocityMatrixOut);

         initialICPCurrentMultiplier.compute(doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(positionMatrixOut.get(0, 0), initialICPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(velocityMatrixOut.get(0, 0), initialICPCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(initialICPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(initialICPCurrentMultiplier.getVelocityMultiplier()));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationTwoCMPThirdSegment()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      double omega = 3.0;
      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      List<DoubleYoVariable> transferSplitFractions = new ArrayList<>();
      List<DoubleYoVariable> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration" + i, registry);
         DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         DoubleYoVariable transferSplitRatio = new DoubleYoVariable("transferSplitRatio" + i, registry);
         DoubleYoVariable swingSplitRatio = new DoubleYoVariable("swingSplitRatio" + i, registry);
         transferSplitRatio.setToNaN();
         swingSplitRatio.setToNaN();
         transferSplitFractions.add(transferSplitRatio);
         swingSplitFractions.add(swingSplitRatio);
      }

      boolean projectCMPForward = true;
      InitialICPCurrentMultiplier initialICPCurrentMultiplier = new InitialICPCurrentMultiplier(startOfSplineTime, endOfSplineTime, "", registry);

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
         double timeRemaining = singleSupportDuration - timeInCurrentState;

         initialICPCurrentMultiplier.compute(doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(0.0, initialICPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(0.0, initialICPCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(initialICPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(initialICPCurrentMultiplier.getVelocityMultiplier()));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationOneCMPSwing()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      double omega = 3.0;
      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      List<DoubleYoVariable> transferSplitFractions = new ArrayList<>();
      List<DoubleYoVariable> swingSplitFractions = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration" + i, registry);
         DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);

         DoubleYoVariable transferSplitRatio = new DoubleYoVariable("transferSplitRatio" + i, registry);
         transferSplitRatio.setToNaN();
         transferSplitFractions.add(transferSplitRatio);
      }

      InitialICPCurrentMultiplier initialICPCurrentMultiplier = new InitialICPCurrentMultiplier(startOfSplineTime, endOfSplineTime, "", registry);

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

         initialICPCurrentMultiplier.compute(doubleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(0.0, initialICPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(0.0, initialICPCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(initialICPCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(initialICPCurrentMultiplier.getVelocityMultiplier()));
      }
   }
}
