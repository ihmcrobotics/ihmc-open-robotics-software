package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.CubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.CubicMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.transfer.TransferInitialICPVelocityMatrix;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.Random;

public class InitialICPVelocityCurrentMultiplierTest
{
   private static final double epsilon = 0.0001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationTwoCMPTransfer()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble totalTrajectoryTime = new YoDouble("totalTrajectoryTime", registry);

      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);
      }

      InitialICPVelocityCurrentMultiplier initialICPVelocityCurrentMultiplier = new InitialICPVelocityCurrentMultiplier("", registry);

      TransferInitialICPVelocityMatrix initialICPMatrix = new TransferInitialICPVelocityMatrix();

      CubicMatrix positionMatrix = new CubicMatrix();
      CubicDerivativeMatrix velocityMatrix = new CubicDerivativeMatrix();
      DenseMatrix64F position = new DenseMatrix64F(1, 1);
      DenseMatrix64F velocity = new DenseMatrix64F(1, 1);

      for (int iter = 0; iter < iters; iter++)
      {
         doubleSupportDurations.get(0).set(2.0 * random.nextDouble());
         singleSupportDurations.get(0).set(5.0 * random.nextDouble());
         doubleSupportDurations.get(1).set(2.0 * random.nextDouble());

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

         initialICPVelocityCurrentMultiplier.compute(doubleSupportDurations, timeInCurrentState, isInTransfer);

         Assert.assertEquals(position.get(0, 0), initialICPVelocityCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(velocity.get(0, 0), initialICPVelocityCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(initialICPVelocityCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(initialICPVelocityCurrentMultiplier.getVelocityMultiplier()));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationOneCMPTransfer()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble totalTrajectoryTime = new YoDouble("totalTrajectoryTime", registry);

      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);
      }

      InitialICPVelocityCurrentMultiplier initialICPVelocityCurrentMultiplier = new InitialICPVelocityCurrentMultiplier("", registry);

      TransferInitialICPVelocityMatrix initialICPMatrix = new TransferInitialICPVelocityMatrix();

      CubicMatrix positionMatrix = new CubicMatrix();
      CubicDerivativeMatrix velocityMatrix = new CubicDerivativeMatrix();
      DenseMatrix64F position = new DenseMatrix64F(1, 1);
      DenseMatrix64F velocity = new DenseMatrix64F(1, 1);

      for (int iter = 0; iter < iters; iter++)
      {
         doubleSupportDurations.get(0).set(2.0 * random.nextDouble());
         singleSupportDurations.get(0).set(5.0 * random.nextDouble());
         doubleSupportDurations.get(1).set(2.0 * random.nextDouble());

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

         initialICPVelocityCurrentMultiplier.compute(doubleSupportDurations, timeInCurrentState, isInTransfer);

         Assert.assertEquals(position.get(0, 0), initialICPVelocityCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(velocity.get(0, 0), initialICPVelocityCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(initialICPVelocityCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(initialICPVelocityCurrentMultiplier.getVelocityMultiplier()));
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

      int maxSteps = 5;
      int iters = 100;

      Random random = new Random();
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);
      }

      InitialICPVelocityCurrentMultiplier initialICPVelocityCurrentMultiplier = new InitialICPVelocityCurrentMultiplier("", registry);

      for (int iter = 0; iter < iters; iter++)
      {
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

         double timeInCurrentState = random.nextDouble() * startOfSpline;
         double timeRemaining = singleSupportDuration - timeInCurrentState;

         initialICPVelocityCurrentMultiplier.compute(doubleSupportDurations, timeRemaining, isInTransfer);

         Assert.assertEquals(0.0, initialICPVelocityCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(0.0, initialICPVelocityCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(initialICPVelocityCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(initialICPVelocityCurrentMultiplier.getVelocityMultiplier()));
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

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);
      }

      InitialICPVelocityCurrentMultiplier initialICPVelocityCurrentMultiplier = new InitialICPVelocityCurrentMultiplier("", registry);

      for (int iter = 0; iter < iters; iter++)
      {
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
         double timeRemaining = singleSupportDuration - timeInCurrentState;

         initialICPVelocityCurrentMultiplier.compute(doubleSupportDurations, timeRemaining, isInTransfer);

         Assert.assertEquals(0.0, initialICPVelocityCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(0.0, initialICPVelocityCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(initialICPVelocityCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(initialICPVelocityCurrentMultiplier.getVelocityMultiplier()));
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

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);
      }

      InitialICPVelocityCurrentMultiplier initialICPVelocityCurrentMultiplier = new InitialICPVelocityCurrentMultiplier("", registry);

      for (int iter = 0; iter < iters; iter++)
      {
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

         double timeInCurrentState = random.nextDouble() * (singleSupportDuration - endOfSpline) + endOfSpline;
         double timeRemaining = singleSupportDuration - timeInCurrentState;

         initialICPVelocityCurrentMultiplier.compute(doubleSupportDurations, timeRemaining, isInTransfer);

         Assert.assertEquals(0.0, initialICPVelocityCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(0.0, initialICPVelocityCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(initialICPVelocityCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(initialICPVelocityCurrentMultiplier.getVelocityMultiplier()));
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

      for (int i = 0 ; i < maxSteps + 1; i++)
      {
         YoDouble doubleSupportDuration = new YoDouble("doubleSupportDuration" + i, registry);
         YoDouble singleSupportDuration = new YoDouble("singleSupportDuration" + i, registry);
         doubleSupportDuration.setToNaN();
         singleSupportDuration.setToNaN();
         doubleSupportDurations.add(doubleSupportDuration);
         singleSupportDurations.add(singleSupportDuration);
      }

      InitialICPVelocityCurrentMultiplier initialICPVelocityCurrentMultiplier = new InitialICPVelocityCurrentMultiplier("", registry);

      for (int iter = 0; iter < iters; iter++)
      {
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

         double timeInCurrentState = random.nextDouble() * (singleSupportDuration - endOfSpline) + endOfSpline;
         double timeRemaining = singleSupportDuration - timeInCurrentState;

         initialICPVelocityCurrentMultiplier.compute(doubleSupportDurations, timeRemaining, isInTransfer);

         Assert.assertEquals(0.0, initialICPVelocityCurrentMultiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals(0.0, initialICPVelocityCurrentMultiplier.getVelocityMultiplier(), epsilon);
         Assert.assertFalse(Double.isNaN(initialICPVelocityCurrentMultiplier.getPositionMultiplier()));
         Assert.assertFalse(Double.isNaN(initialICPVelocityCurrentMultiplier.getVelocityMultiplier()));
      }
   }
}
