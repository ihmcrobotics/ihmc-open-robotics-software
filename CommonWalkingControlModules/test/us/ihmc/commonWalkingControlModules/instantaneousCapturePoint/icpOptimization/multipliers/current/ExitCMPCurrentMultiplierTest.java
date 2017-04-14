package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.CubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.CubicMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing.SwingExitCMPMatrix;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class ExitCMPCurrentMultiplierTest
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
         doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration" + i, registry));
         transferSplitFractions.add(new DoubleYoVariable("transferSplitFraction" + i, registry));
         swingSplitFractions.add(new DoubleYoVariable("swingSplitFraction" + i, registry));
      }

      ExitCMPCurrentMultiplier exitCMPCurrentMultiplier = new ExitCMPCurrentMultiplier(swingSplitFractions, startOfSplineTime, endOfSplineTime, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         for (int step = 0; step < maxSteps; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
            swingSplitFractions.get(step).set(0.7 * random.nextDouble());
            transferSplitFractions.get(step).set(0.7 * random.nextDouble());
         }

         double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();

         totalTrajectoryTime.set(singleSupportDuration);

         boolean isInTransfer = true;
         boolean useTwoCMPs = true;

         double doubleSupportDuration = doubleSupportDurations.get(0).getDoubleValue();

         double timeInCurrentState = random.nextDouble() * doubleSupportDuration;

         exitCMPCurrentMultiplier.compute(singleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(0.0, exitCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(0.0, exitCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
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
         doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration" + i, registry));
         transferSplitFractions.add(new DoubleYoVariable("transferSplitFraction" + i, registry));
         swingSplitFractions.add(new DoubleYoVariable("swingSplitFraction" + i, registry));
      }

      ExitCMPCurrentMultiplier exitCMPCurrentMultiplier = new ExitCMPCurrentMultiplier(swingSplitFractions, startOfSplineTime, endOfSplineTime, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double swingRatio = 0.7 * random.nextDouble();
         swingSplitFractions.get(0).set(swingRatio);
         double currentTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         double upcomingTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(1).set(upcomingTransferRatio);

         for (int step = 0; step < maxSteps; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }

         double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();

         totalTrajectoryTime.set(singleSupportDuration);

         boolean isInTransfer = true;
         boolean useTwoCMPs = false;

         double doubleSupportDuration = doubleSupportDurations.get(0).getDoubleValue();

         double timeInCurrentState = random.nextDouble() * doubleSupportDuration;

         exitCMPCurrentMultiplier.compute(singleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(0.0, exitCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(0.0, exitCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationTwoCMPFirstSegment()
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
         doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration" + i, registry));
         transferSplitFractions.add(new DoubleYoVariable("transferSplitFraction" + i, registry));
         swingSplitFractions.add(new DoubleYoVariable("swingSplitFraction" + i, registry));
      }

      ExitCMPCurrentMultiplier exitCMPCurrentMultiplier = new ExitCMPCurrentMultiplier(swingSplitFractions, startOfSplineTime, endOfSplineTime, registry);


      for (int iter = 0; iter < iters; iter++)
      {
         double swingRatio = 0.7 * random.nextDouble();
         swingSplitFractions.get(0).set(swingRatio);
         double currentTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         double upcomingTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(1).set(upcomingTransferRatio);

         for (int step = 0; step < maxSteps; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }

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

         exitCMPCurrentMultiplier.compute(singleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(0.0, exitCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(0.0, exitCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
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
         doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration" + i, registry));
         transferSplitFractions.add(new DoubleYoVariable("transferSplitFraction" + i, registry));
         swingSplitFractions.add(new DoubleYoVariable("swingSplitFraction" + i, registry));
      }

      ExitCMPCurrentMultiplier exitCMPCurrentMultiplier = new ExitCMPCurrentMultiplier(swingSplitFractions, startOfSplineTime, endOfSplineTime, registry);

      SwingExitCMPMatrix exitCMPMatrix = new SwingExitCMPMatrix(swingSplitFractions, endOfSplineTime);
      CubicMatrix cubicMatrix = new CubicMatrix();
      CubicDerivativeMatrix cubicDerivativeMatrix = new CubicDerivativeMatrix();
      DenseMatrix64F positionMatrixOut = new DenseMatrix64F(1, 1);
      DenseMatrix64F velocityMatrixOut = new DenseMatrix64F(1, 1);

      for (int iter = 0; iter < iters; iter++)
      {
         double swingRatio = 0.7 * random.nextDouble();
         swingSplitFractions.get(0).set(swingRatio);
         double currentTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         double upcomingTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(1).set(upcomingTransferRatio);

         for (int step = 0; step < maxSteps; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }

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

         cubicMatrix.update(timeInCurrentState - startOfSpline);
         cubicDerivativeMatrix.update(timeInCurrentState - startOfSpline);

         exitCMPMatrix.compute(singleSupportDurations, omega);
         CommonOps.mult(cubicMatrix, exitCMPMatrix, positionMatrixOut);
         CommonOps.mult(cubicDerivativeMatrix, exitCMPMatrix, velocityMatrixOut);

         exitCMPCurrentMultiplier.compute(singleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals("iter = " + iter, positionMatrixOut.get(0, 0), exitCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("iter = " + iter, velocityMatrixOut.get(0, 0), exitCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
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
         doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration" + i, registry));
         transferSplitFractions.add(new DoubleYoVariable("transferSplitFraction" + i, registry));
         swingSplitFractions.add(new DoubleYoVariable("swingSplitFraction" + i, registry));
      }

      ExitCMPCurrentMultiplier exitCMPCurrentMultiplier = new ExitCMPCurrentMultiplier(swingSplitFractions, startOfSplineTime, endOfSplineTime, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double swingRatio = 0.7 * random.nextDouble();
         swingSplitFractions.get(0).set(swingRatio);
         double currentTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         double upcomingTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(1).set(upcomingTransferRatio);

         for (int step = 0; step < maxSteps; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }

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

         double upcomingInitialDoubleSupportDuration = upcomingTransferRatio * doubleSupportDurations.get(1).getDoubleValue();
         double timeSpentOnExitCMP = (1.0 - swingRatio) * singleSupportDuration + upcomingInitialDoubleSupportDuration;

         exitCMPCurrentMultiplier.compute(singleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         double projectionTime = timeInCurrentState - singleSupportDuration + timeSpentOnExitCMP - upcomingInitialDoubleSupportDuration;
         double projection = Math.exp(omega * projectionTime);

         Assert.assertEquals(1.0 - projection, exitCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(-omega * projection, exitCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
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
         doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration" + i, registry));
         transferSplitFractions.add(new DoubleYoVariable("transferSplitFraction" + i, registry));
         swingSplitFractions.add(new DoubleYoVariable("swingSplitFraction" + i, registry));
      }

      ExitCMPCurrentMultiplier exitCMPCurrentMultiplier = new ExitCMPCurrentMultiplier(swingSplitFractions, startOfSplineTime, endOfSplineTime, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double swingRatio = 0.7 * random.nextDouble();
         swingSplitFractions.get(0).set(swingRatio);
         double currentTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(0).set(currentTransferRatio);
         double upcomingTransferRatio = 0.7 * random.nextDouble();
         transferSplitFractions.get(1).set(upcomingTransferRatio);

         for (int step = 0; step < maxSteps; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }

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
         boolean useTwoCMPs = false;

         double timeInCurrentState = random.nextDouble() * (singleSupportDuration - endOfSpline) + endOfSpline;

         exitCMPCurrentMultiplier.compute(singleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(0.0, exitCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(0.0, exitCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
      }
   }
}
