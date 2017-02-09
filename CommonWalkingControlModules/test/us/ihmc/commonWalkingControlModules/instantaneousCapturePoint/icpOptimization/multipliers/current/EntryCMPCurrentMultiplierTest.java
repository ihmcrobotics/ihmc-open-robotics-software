package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current;

import java.util.ArrayList;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.CubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.CubicMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing.SwingEntryCMPMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.transfer.TransferEntryCMPMatrix;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class EntryCMPCurrentMultiplierTest
{
   private static final double epsilon = 0.0001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationTwoCMPTransfer()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable defaultSplitRatio = new DoubleYoVariable("defaultSplitRatio", registry);
      DoubleYoVariable upcomingSplitRatio = new DoubleYoVariable("upcomingSplitRatio", registry);
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      double omega = 3.0;
      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration" + i, registry));
      }

      boolean projectCMPForward = false;
      EntryCMPCurrentMultiplier entryCMPCurrentMultiplier = new EntryCMPCurrentMultiplier(upcomingSplitRatio, defaultSplitRatio, exitCMPRatio,
            startOfSplineTime, endOfSplineTime, totalTrajectoryTime, projectCMPForward, registry);

      TransferEntryCMPMatrix entryCMPMatrix = new TransferEntryCMPMatrix(defaultSplitRatio);

      CubicMatrix positionMatrix = new CubicMatrix();
      CubicDerivativeMatrix velocityMatrix = new CubicDerivativeMatrix();
      DenseMatrix64F position = new DenseMatrix64F(1, 1);
      DenseMatrix64F velocity = new DenseMatrix64F(1, 1);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);
         double defaultSplit = 0.7 * random.nextDouble();
         defaultSplitRatio.set(defaultSplit);
         double upcomingSplit = 0.7 * random.nextDouble();
         upcomingSplitRatio.set(upcomingSplit);

         for (int step = 0; step < maxSteps; step++)
         {
            doubleSupportDurations.get(step).set(2.0 * random.nextDouble());
            singleSupportDurations.get(step).set(5.0 * random.nextDouble());
         }

         double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();

         totalTrajectoryTime.set(singleSupportDuration);

         boolean isInTransfer = true;
         boolean useTwoCMPs = true;

         double doubleSupportDuration = doubleSupportDurations.get(0).getDoubleValue();

         double timeInCurrentState = random.nextDouble() * doubleSupportDuration;
         double timeRemaining = doubleSupportDuration - timeInCurrentState;

         positionMatrix.setSegmentDuration(doubleSupportDuration);
         velocityMatrix.setSegmentDuration(doubleSupportDuration);
         positionMatrix.update(timeRemaining);
         velocityMatrix.update(timeRemaining);

         entryCMPMatrix.compute(doubleSupportDurations, omega);

         CommonOps.mult(positionMatrix, entryCMPMatrix, position);
         CommonOps.mult(velocityMatrix, entryCMPMatrix, velocity);

         entryCMPCurrentMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(position.get(0, 0), entryCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(velocity.get(0, 0), entryCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationOneCMPTransfer()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable defaultSplitRatio = new DoubleYoVariable("defaultSplitRatio", registry);
      DoubleYoVariable upcomingSplitRatio = new DoubleYoVariable("upcomingSplitRatio", registry);
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      double omega = 3.0;
      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration" + i, registry));
      }

      boolean projectCMPForward = false;
      EntryCMPCurrentMultiplier entryCMPCurrentMultiplier = new EntryCMPCurrentMultiplier(upcomingSplitRatio, defaultSplitRatio, exitCMPRatio,
            startOfSplineTime, endOfSplineTime, totalTrajectoryTime, projectCMPForward, registry);

      TransferEntryCMPMatrix entryCMPMatrix = new TransferEntryCMPMatrix(defaultSplitRatio);

      CubicMatrix positionMatrix = new CubicMatrix();
      CubicDerivativeMatrix velocityMatrix = new CubicDerivativeMatrix();
      DenseMatrix64F position = new DenseMatrix64F(1, 1);
      DenseMatrix64F velocity = new DenseMatrix64F(1, 1);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);
         double defaultSplit = 0.7 * random.nextDouble();
         defaultSplitRatio.set(defaultSplit);
         double upcomingSplit = 0.7 * random.nextDouble();
         upcomingSplitRatio.set(upcomingSplit);

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
         double timeRemaining = doubleSupportDuration - timeInCurrentState;

         positionMatrix.setSegmentDuration(doubleSupportDuration);
         velocityMatrix.setSegmentDuration(doubleSupportDuration);
         positionMatrix.update(timeRemaining);
         velocityMatrix.update(timeRemaining);

         entryCMPMatrix.compute(doubleSupportDurations, omega);

         CommonOps.mult(positionMatrix, entryCMPMatrix, position);
         CommonOps.mult(velocityMatrix, entryCMPMatrix, velocity);

         entryCMPCurrentMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(position.get(0, 0), entryCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(velocity.get(0, 0), entryCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationProjectForwardTwoCMPFirstSegment()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable defaultSplitRatio = new DoubleYoVariable("defaultSplitRatio", registry);
      DoubleYoVariable upcomingSplitRatio = new DoubleYoVariable("upcomingSplitRatio", registry);
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      double omega = 3.0;
      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration" + i, registry));
      }

      boolean projectCMPForward = true;
      EntryCMPCurrentMultiplier entryCMPCurrentMultiplier = new EntryCMPCurrentMultiplier(upcomingSplitRatio, defaultSplitRatio, exitCMPRatio,
            startOfSplineTime, endOfSplineTime, totalTrajectoryTime, projectCMPForward, registry);


      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);
         double defaultSplit = 0.7 * random.nextDouble();
         defaultSplitRatio.set(defaultSplit);
         double upcomingSplit = 0.7 * random.nextDouble();
         upcomingSplitRatio.set(upcomingSplit);

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

         entryCMPCurrentMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         double projection = Math.exp(omega * timeInCurrentState);
         Assert.assertEquals(1.0 - projection, entryCMPCurrentMultiplier.getPositionMultiplier(), epsilon);

         Assert.assertEquals(- omega * projection, entryCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
      }
   }


   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationTwoCMPSecondSegment()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable defaultSplitRatio = new DoubleYoVariable("defaultSplitRatio", registry);
      DoubleYoVariable upcomingSplitRatio = new DoubleYoVariable("upcomingSplitRatio", registry);
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      double omega = 3.0;
      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration" + i, registry));
      }

      boolean projectCMPForward = false;
      EntryCMPCurrentMultiplier entryCMPCurrentMultiplier = new EntryCMPCurrentMultiplier(upcomingSplitRatio, defaultSplitRatio, exitCMPRatio,
            startOfSplineTime, endOfSplineTime, totalTrajectoryTime, projectCMPForward, registry);

      SwingEntryCMPMatrix entryCMPMatrix = new SwingEntryCMPMatrix(startOfSplineTime);
      CubicMatrix cubicMatrix = new CubicMatrix();
      CubicDerivativeMatrix cubicDerivativeMatrix = new CubicDerivativeMatrix();
      DenseMatrix64F positionMatrixOut = new DenseMatrix64F(1, 1);
      DenseMatrix64F velocityMatrixOut = new DenseMatrix64F(1, 1);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);
         double defaultSplit = 0.7 * random.nextDouble();
         defaultSplitRatio.set(defaultSplit);
         double upcomingSplit = 0.7 * random.nextDouble();
         upcomingSplitRatio.set(upcomingSplit);

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
         double timeRemaining = singleSupportDuration - timeInCurrentState;

         cubicMatrix.setSegmentDuration(endOfSpline - startOfSpline);
         cubicDerivativeMatrix.setSegmentDuration(endOfSpline - startOfSpline);

         double endingSegmentDuration = singleSupportDuration - endOfSpline;
         cubicMatrix.update(timeRemaining - endingSegmentDuration);
         cubicDerivativeMatrix.update(timeRemaining - endingSegmentDuration);

         entryCMPMatrix.compute(omega);
         CommonOps.mult(cubicMatrix, entryCMPMatrix, positionMatrixOut);
         CommonOps.mult(cubicDerivativeMatrix, entryCMPMatrix, velocityMatrixOut);

         entryCMPCurrentMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(positionMatrixOut.get(0, 0), entryCMPCurrentMultiplier.getPositionMultiplier(), epsilon);

         Assert.assertEquals(velocityMatrixOut.get(0, 0), entryCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationTwoCMPThirdSegment()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable defaultSplitRatio = new DoubleYoVariable("defaultSplitRatio", registry);
      DoubleYoVariable upcomingSplitRatio = new DoubleYoVariable("upcomingSplitRatio", registry);
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      double omega = 3.0;
      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration" + i, registry));
      }

      boolean projectCMPForward = true;
      EntryCMPCurrentMultiplier entryCMPCurrentMultiplier = new EntryCMPCurrentMultiplier(upcomingSplitRatio, defaultSplitRatio, exitCMPRatio,
            startOfSplineTime, endOfSplineTime, totalTrajectoryTime, projectCMPForward, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);
         double defaultSplit = 0.7 * random.nextDouble();
         defaultSplitRatio.set(defaultSplit);
         double upcomingSplit = 0.7 * random.nextDouble();
         upcomingSplitRatio.set(upcomingSplit);

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

         entryCMPCurrentMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(0.0, entryCMPCurrentMultiplier.getPositionMultiplier(), epsilon);

         Assert.assertEquals(0.0, entryCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationOneCMPSwing()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      DoubleYoVariable defaultSplitRatio = new DoubleYoVariable("defaultSplitRatio", registry);
      DoubleYoVariable upcomingSplitRatio = new DoubleYoVariable("upcomingSplitRatio", registry);
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      double omega = 3.0;
      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration" + i, registry));
      }

      boolean projectCMPForward = true;
      EntryCMPCurrentMultiplier entryCMPCurrentMultiplier = new EntryCMPCurrentMultiplier(upcomingSplitRatio, defaultSplitRatio, exitCMPRatio,
            startOfSplineTime, endOfSplineTime, totalTrajectoryTime, projectCMPForward, registry);

      for (int iter = 0; iter < iters; iter++)
      {
         double exitRatio = 0.7 * random.nextDouble();
         exitCMPRatio.set(exitRatio);
         double defaultSplit = 0.7 * random.nextDouble();
         defaultSplitRatio.set(defaultSplit);
         double upcomingSplit = 0.7 * random.nextDouble();
         upcomingSplitRatio.set(upcomingSplit);

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

         double timeInCurrentState = random.nextDouble() * singleSupportDuration;

         double currentDoubleSupportDuration = doubleSupportDurations.get(0).getDoubleValue();
         double upcomingDoubleSupportDuration = doubleSupportDurations.get(1).getDoubleValue();
         double stepDuration = singleSupportDuration + currentDoubleSupportDuration;

         double timeSpentOnExitCMP = exitRatio * stepDuration;
         double upcomingInitialDoubleSupportDuration = upcomingSplitRatio.getDoubleValue() * upcomingDoubleSupportDuration;

         double projectionTime = timeInCurrentState - singleSupportDuration + timeSpentOnExitCMP - upcomingInitialDoubleSupportDuration;

         double positionMultiplier = 1.0 - Math.exp(omega * projectionTime);
         double velocityMultiplier = - omega * Math.exp(omega * projectionTime);

         entryCMPCurrentMultiplier.compute(doubleSupportDurations, singleSupportDurations, timeInCurrentState, useTwoCMPs, isInTransfer, omega);

         Assert.assertEquals(positionMultiplier, entryCMPCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(velocityMultiplier, entryCMPCurrentMultiplier.getVelocityMultiplier(), epsilon);

      }
   }
}
