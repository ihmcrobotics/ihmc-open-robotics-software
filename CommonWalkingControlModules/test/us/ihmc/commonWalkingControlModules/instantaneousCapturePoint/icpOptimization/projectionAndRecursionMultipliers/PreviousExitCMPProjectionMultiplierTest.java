package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.interpolation.CubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.interpolation.CubicMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer.TransferPreviousExitCMPProjectionMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import java.util.ArrayList;

import static com.badlogic.gdx.math.MathUtils.random;

public class PreviousExitCMPProjectionMultiplierTest
{
   private static final double epsilon = 0.0001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testOneCMPSS()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPDurationInPercentOfSteptime = new DoubleYoVariable("exitCMPDurationInPercentOfSteptime", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      DoubleYoVariable currentDoubleSupportDuration = new DoubleYoVariable("currentDoubleSupportDuration", registry);
      DoubleYoVariable upcomingDoubleSupportDuration = new DoubleYoVariable("upcomingDoubleSupportDuration", registry);

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(currentDoubleSupportDuration);
      doubleSupportDurations.add(upcomingDoubleSupportDuration);

      DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration", registry);
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      singleSupportDurations.add(singleSupportDuration);

      PreviousExitCMPProjectionMultiplier multiplier = new PreviousExitCMPProjectionMultiplier(registry, doubleSupportSplitRatio);

      int iters = 100;
      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.7 * random.nextDouble();
         double exitRatio = 0.7 * random.nextDouble();

         omega.set(omega0);
         doubleSupportSplitRatio.set(splitRatio);
         exitCMPDurationInPercentOfSteptime.set(exitRatio);

         double currentDoubleSupport = 2.0 * random.nextDouble();
         double upcomingDoubleSupport = 2.0 * random.nextDouble();
         double singleSupport = 5.0 * random.nextDouble();

         double timeRemaining = random.nextDouble() * singleSupport;

         currentDoubleSupportDuration.set(currentDoubleSupport);
         upcomingDoubleSupportDuration.set(upcomingDoubleSupport);
         singleSupportDuration.set(singleSupport);

         double alpha1 = random.nextDouble();
         double alpha2 = random.nextDouble();
         double startOfSpline = Math.min(alpha1, alpha2) * singleSupport;
         double endOfSpline = Math.max(alpha1, alpha2) * singleSupport;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupport);

         multiplier.compute(doubleSupportDurations, timeRemaining, false, omega0, false);

         Assert.assertEquals("", 0.0, multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", 0.0, multiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testOneCMPSSUsingInitialICP()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPDurationInPercentOfSteptime = new DoubleYoVariable("exitCMPDurationInPercentOfSteptime", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      DoubleYoVariable currentDoubleSupportDuration = new DoubleYoVariable("currentDoubleSupportDuration", registry);
      DoubleYoVariable upcomingDoubleSupportDuration = new DoubleYoVariable("upcomingDoubleSupportDuration", registry);

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(currentDoubleSupportDuration);
      doubleSupportDurations.add(upcomingDoubleSupportDuration);

      DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration", registry);
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      singleSupportDurations.add(singleSupportDuration);

      PreviousExitCMPProjectionMultiplier multiplier = new PreviousExitCMPProjectionMultiplier(registry, doubleSupportSplitRatio);

      int iters = 100;
      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.7 * random.nextDouble();
         double exitRatio = 0.7 * random.nextDouble();

         omega.set(omega0);
         doubleSupportSplitRatio.set(splitRatio);
         exitCMPDurationInPercentOfSteptime.set(exitRatio);

         double currentDoubleSupport = 2.0 * random.nextDouble();
         double upcomingDoubleSupport = 2.0 * random.nextDouble();
         double singleSupport = 5.0 * random.nextDouble();

         double timeRemaining = random.nextDouble() * singleSupport;

         currentDoubleSupportDuration.set(currentDoubleSupport);
         upcomingDoubleSupportDuration.set(upcomingDoubleSupport);
         singleSupportDuration.set(singleSupport);

         double alpha1 = random.nextDouble();
         double alpha2 = random.nextDouble();
         double startOfSpline = Math.min(alpha1, alpha2) * singleSupport;
         double endOfSpline = Math.max(alpha1, alpha2) * singleSupport;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupport);

         multiplier.compute(doubleSupportDurations, timeRemaining, false, omega0, true);

         Assert.assertEquals("", 0.0, multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", 0.0, multiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testOneCMPTransfer()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPDurationInPercentOfSteptime = new DoubleYoVariable("exitCMPDurationInPercentOfSteptime", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      DoubleYoVariable currentDoubleSupportDuration = new DoubleYoVariable("currentDoubleSupportDuration", registry);
      DoubleYoVariable upcomingDoubleSupportDuration = new DoubleYoVariable("upcomingDoubleSupportDuration", registry);

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(currentDoubleSupportDuration);
      doubleSupportDurations.add(upcomingDoubleSupportDuration);

      DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration", registry);
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      singleSupportDurations.add(singleSupportDuration);

      PreviousExitCMPProjectionMultiplier multiplier = new PreviousExitCMPProjectionMultiplier(registry, doubleSupportSplitRatio);
      TransferPreviousExitCMPProjectionMatrix transferExitCMPProjectionMatrix = new TransferPreviousExitCMPProjectionMatrix(doubleSupportSplitRatio);

      CubicMatrix cubicMatrix = new CubicMatrix();
      CubicDerivativeMatrix cubicDerivativeMatrix = new CubicDerivativeMatrix();

      int iters = 100;
      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.7 * random.nextDouble();
         double exitRatio = 0.7 * random.nextDouble();

         omega.set(omega0);
         doubleSupportSplitRatio.set(splitRatio);
         exitCMPDurationInPercentOfSteptime.set(exitRatio);

         double currentDoubleSupport = 2.0 * random.nextDouble();
         double upcomingDoubleSupport = 2.0 * random.nextDouble();
         double singleSupport = 5.0 * random.nextDouble();

         currentDoubleSupportDuration.set(currentDoubleSupport);
         upcomingDoubleSupportDuration.set(upcomingDoubleSupport);
         singleSupportDuration.set(singleSupport);

         double timeRemaining = random.nextDouble() * currentDoubleSupport;

         double alpha1 = random.nextDouble();
         double alpha2 = random.nextDouble();
         double startOfSpline = Math.min(alpha1, alpha2) * singleSupport;
         double endOfSpline = Math.max(alpha1, alpha2) * singleSupport;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupport);

         transferExitCMPProjectionMatrix.compute(currentDoubleSupport, omega0, false);

         cubicDerivativeMatrix.setSegmentDuration(currentDoubleSupport);
         cubicDerivativeMatrix.update(timeRemaining);
         cubicMatrix.setSegmentDuration(currentDoubleSupport);
         cubicMatrix.update(timeRemaining);

         DenseMatrix64F positionMatrixOut = new DenseMatrix64F(1, 1);
         DenseMatrix64F velocityMatrixOut = new DenseMatrix64F(1, 1);
         CommonOps.mult(cubicMatrix, transferExitCMPProjectionMatrix, positionMatrixOut);
         CommonOps.mult(cubicDerivativeMatrix, transferExitCMPProjectionMatrix, velocityMatrixOut);

         multiplier.compute(doubleSupportDurations, timeRemaining, true, omega0, false);

         Assert.assertEquals("", positionMatrixOut.get(0, 0), multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", velocityMatrixOut.get(0, 0), multiplier.getVelocityMultiplier(), epsilon);

         multiplier.reset();
         Assert.assertEquals("", 0.0, multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", 0.0, multiplier.getVelocityMultiplier(), epsilon);
      }
   }


   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testOneCMPTransferUsingInitialICP()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPDurationInPercentOfSteptime = new DoubleYoVariable("exitCMPDurationInPercentOfSteptime", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      DoubleYoVariable currentDoubleSupportDuration = new DoubleYoVariable("currentDoubleSupportDuration", registry);
      DoubleYoVariable upcomingDoubleSupportDuration = new DoubleYoVariable("upcomingDoubleSupportDuration", registry);

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(currentDoubleSupportDuration);
      doubleSupportDurations.add(upcomingDoubleSupportDuration);

      DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration", registry);
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      singleSupportDurations.add(singleSupportDuration);

      PreviousExitCMPProjectionMultiplier multiplier = new PreviousExitCMPProjectionMultiplier(registry, doubleSupportSplitRatio);
      TransferPreviousExitCMPProjectionMatrix transferExitCMPProjectionMatrix = new TransferPreviousExitCMPProjectionMatrix(doubleSupportSplitRatio);

      CubicMatrix cubicMatrix = new CubicMatrix();
      CubicDerivativeMatrix cubicDerivativeMatrix = new CubicDerivativeMatrix();

      int iters = 100;
      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.7 * random.nextDouble();
         double exitRatio = 0.7 * random.nextDouble();

         omega.set(omega0);
         doubleSupportSplitRatio.set(splitRatio);
         exitCMPDurationInPercentOfSteptime.set(exitRatio);

         double currentDoubleSupport = 2.0 * random.nextDouble();
         double upcomingDoubleSupport = 2.0 * random.nextDouble();
         double singleSupport = 5.0 * random.nextDouble();

         currentDoubleSupportDuration.set(currentDoubleSupport);
         upcomingDoubleSupportDuration.set(upcomingDoubleSupport);
         singleSupportDuration.set(singleSupport);

         double timeRemaining = random.nextDouble() * currentDoubleSupport;

         double alpha1 = random.nextDouble();
         double alpha2 = random.nextDouble();
         double startOfSpline = Math.min(alpha1, alpha2) * singleSupport;
         double endOfSpline = Math.max(alpha1, alpha2) * singleSupport;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupport);

         transferExitCMPProjectionMatrix.compute(currentDoubleSupport, omega0, true);

         cubicDerivativeMatrix.setSegmentDuration(currentDoubleSupport);
         cubicDerivativeMatrix.update(timeRemaining);
         cubicMatrix.setSegmentDuration(currentDoubleSupport);
         cubicMatrix.update(timeRemaining);

         DenseMatrix64F positionMatrixOut = new DenseMatrix64F(1, 1);
         DenseMatrix64F velocityMatrixOut = new DenseMatrix64F(1, 1);
         CommonOps.mult(cubicMatrix, transferExitCMPProjectionMatrix, positionMatrixOut);
         CommonOps.mult(cubicDerivativeMatrix, transferExitCMPProjectionMatrix, velocityMatrixOut);

         multiplier.compute(doubleSupportDurations, timeRemaining, true, omega0, true);

         Assert.assertEquals("", positionMatrixOut.get(0, 0), multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", velocityMatrixOut.get(0, 0), multiplier.getVelocityMultiplier(), epsilon);

         multiplier.reset();
         Assert.assertEquals("", 0.0, multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", 0.0, multiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testTwoCMPSSFirstSegment()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPDurationInPercentOfSteptime = new DoubleYoVariable("exitCMPDurationInPercentOfSteptime", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      DoubleYoVariable currentDoubleSupportDuration = new DoubleYoVariable("currentDoubleSupportDuration", registry);
      DoubleYoVariable upcomingDoubleSupportDuration = new DoubleYoVariable("upcomingDoubleSupportDuration", registry);

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(currentDoubleSupportDuration);
      doubleSupportDurations.add(upcomingDoubleSupportDuration);

      DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration", registry);
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      singleSupportDurations.add(singleSupportDuration);

      PreviousExitCMPProjectionMultiplier multiplier = new PreviousExitCMPProjectionMultiplier(registry, doubleSupportSplitRatio);

      int iters = 100;
      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.7 * random.nextDouble();
         double exitRatio = 0.7 * random.nextDouble();

         omega.set(omega0);
         doubleSupportSplitRatio.set(splitRatio);
         exitCMPDurationInPercentOfSteptime.set(exitRatio);

         double currentDoubleSupport = 2.0 * random.nextDouble();
         double upcomingDoubleSupport = 2.0 * random.nextDouble();
         double singleSupport = 5.0 * random.nextDouble();

         currentDoubleSupportDuration.set(currentDoubleSupport);
         upcomingDoubleSupportDuration.set(upcomingDoubleSupport);
         singleSupportDuration.set(singleSupport);

         double alpha1 = random.nextDouble();
         double alpha2 = random.nextDouble();
         double startOfSpline = Math.min(alpha1, alpha2) * singleSupport;
         double endOfSpline = Math.max(alpha1, alpha2) * singleSupport;

         double timeInCurrentState = random.nextDouble() * startOfSpline;
         double timeRemaining = singleSupport - timeInCurrentState;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupport);

         multiplier.compute(doubleSupportDurations, timeRemaining, false, omega0, false);

         Assert.assertEquals("", 0.0, multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", 0.0, multiplier.getVelocityMultiplier(), epsilon);

         multiplier.reset();
         Assert.assertEquals("", 0.0, multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", 0.0, multiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testTwoCMPSSFirstSegmentUsingInitialICP()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPDurationInPercentOfSteptime = new DoubleYoVariable("exitCMPDurationInPercentOfSteptime", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      DoubleYoVariable currentDoubleSupportDuration = new DoubleYoVariable("currentDoubleSupportDuration", registry);
      DoubleYoVariable upcomingDoubleSupportDuration = new DoubleYoVariable("upcomingDoubleSupportDuration", registry);

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(currentDoubleSupportDuration);
      doubleSupportDurations.add(upcomingDoubleSupportDuration);

      DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration", registry);
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      singleSupportDurations.add(singleSupportDuration);

      PreviousExitCMPProjectionMultiplier multiplier = new PreviousExitCMPProjectionMultiplier(registry, doubleSupportSplitRatio);

      int iters = 100;
      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.7 * random.nextDouble();
         double exitRatio = 0.7 * random.nextDouble();

         omega.set(omega0);
         doubleSupportSplitRatio.set(splitRatio);
         exitCMPDurationInPercentOfSteptime.set(exitRatio);

         double currentDoubleSupport = 2.0 * random.nextDouble();
         double upcomingDoubleSupport = 2.0 * random.nextDouble();
         double singleSupport = 5.0 * random.nextDouble();

         currentDoubleSupportDuration.set(currentDoubleSupport);
         upcomingDoubleSupportDuration.set(upcomingDoubleSupport);
         singleSupportDuration.set(singleSupport);

         double alpha1 = random.nextDouble();
         double alpha2 = random.nextDouble();
         double startOfSpline = Math.min(alpha1, alpha2) * singleSupport;
         double endOfSpline = Math.max(alpha1, alpha2) * singleSupport;

         double timeInCurrentState = random.nextDouble() * startOfSpline;
         double timeRemaining = singleSupport - timeInCurrentState;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupport);

         multiplier.compute(doubleSupportDurations, timeRemaining, false, omega0, true);

         Assert.assertEquals("", 0.0, multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", 0.0, multiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testTwoCMPSSThirdSegment()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPDurationInPercentOfSteptime = new DoubleYoVariable("exitCMPDurationInPercentOfSteptime", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      DoubleYoVariable currentDoubleSupportDuration = new DoubleYoVariable("currentDoubleSupportDuration", registry);
      DoubleYoVariable upcomingDoubleSupportDuration = new DoubleYoVariable("upcomingDoubleSupportDuration", registry);

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(currentDoubleSupportDuration);
      doubleSupportDurations.add(upcomingDoubleSupportDuration);

      DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration", registry);
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      singleSupportDurations.add(singleSupportDuration);

      PreviousExitCMPProjectionMultiplier multiplier = new PreviousExitCMPProjectionMultiplier(registry, doubleSupportSplitRatio);

      int iters = 100;
      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.7 * random.nextDouble();
         double exitRatio = 0.7 * random.nextDouble();

         omega.set(omega0);
         doubleSupportSplitRatio.set(splitRatio);
         exitCMPDurationInPercentOfSteptime.set(exitRatio);

         double currentDoubleSupport = 2.0 * random.nextDouble();
         double upcomingDoubleSupport = 2.0 * random.nextDouble();
         double singleSupport = 5.0 * random.nextDouble();

         currentDoubleSupportDuration.set(currentDoubleSupport);
         upcomingDoubleSupportDuration.set(upcomingDoubleSupport);
         singleSupportDuration.set(singleSupport);

         double alpha1 = random.nextDouble();
         double alpha2 = random.nextDouble();
         double startOfSpline = Math.min(alpha1, alpha2) * singleSupport;
         double endOfSpline = Math.max(alpha1, alpha2) * singleSupport;

         double thirdSegmentTime = singleSupport - endOfSpline;

         double timeInCurrentState = random.nextDouble() * thirdSegmentTime + endOfSpline;
         double timeRemaining = singleSupport - timeInCurrentState;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupport);

         multiplier.compute(doubleSupportDurations, timeRemaining, false, omega0, false);

         Assert.assertEquals("", 0.0, multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", 0.0, multiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testTwoCMPSSThirdSegmentUsingInitialICP()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPDurationInPercentOfSteptime = new DoubleYoVariable("exitCMPDurationInPercentOfSteptime", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      DoubleYoVariable currentDoubleSupportDuration = new DoubleYoVariable("currentDoubleSupportDuration", registry);
      DoubleYoVariable upcomingDoubleSupportDuration = new DoubleYoVariable("upcomingDoubleSupportDuration", registry);

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(currentDoubleSupportDuration);
      doubleSupportDurations.add(upcomingDoubleSupportDuration);

      DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration", registry);
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      singleSupportDurations.add(singleSupportDuration);

      PreviousExitCMPProjectionMultiplier multiplier = new PreviousExitCMPProjectionMultiplier(registry, doubleSupportSplitRatio);

      int iters = 100;
      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.7 * random.nextDouble();
         double exitRatio = 0.7 * random.nextDouble();

         omega.set(omega0);
         doubleSupportSplitRatio.set(splitRatio);
         exitCMPDurationInPercentOfSteptime.set(exitRatio);

         double currentDoubleSupport = 2.0 * random.nextDouble();
         double upcomingDoubleSupport = 2.0 * random.nextDouble();
         double singleSupport = 5.0 * random.nextDouble();

         currentDoubleSupportDuration.set(currentDoubleSupport);
         upcomingDoubleSupportDuration.set(upcomingDoubleSupport);
         singleSupportDuration.set(singleSupport);

         double alpha1 = random.nextDouble();
         double alpha2 = random.nextDouble();
         double startOfSpline = Math.min(alpha1, alpha2) * singleSupport;
         double endOfSpline = Math.max(alpha1, alpha2) * singleSupport;

         double thirdSegmentTime = singleSupport - endOfSpline;

         double timeInCurrentState = random.nextDouble() * thirdSegmentTime + endOfSpline;
         double timeRemaining = singleSupport - timeInCurrentState;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupport);

         multiplier.compute(doubleSupportDurations, timeRemaining, false, omega0, true);

         Assert.assertEquals("", 0.0, multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", 0.0, multiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testTwoCMPSSSecondSegment()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPDurationInPercentOfSteptime = new DoubleYoVariable("exitCMPDurationInPercentOfSteptime", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      DoubleYoVariable currentDoubleSupportDuration = new DoubleYoVariable("currentDoubleSupportDuration", registry);
      DoubleYoVariable upcomingDoubleSupportDuration = new DoubleYoVariable("upcomingDoubleSupportDuration", registry);

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(currentDoubleSupportDuration);
      doubleSupportDurations.add(upcomingDoubleSupportDuration);

      DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration", registry);
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      singleSupportDurations.add(singleSupportDuration);

      PreviousExitCMPProjectionMultiplier multiplier = new PreviousExitCMPProjectionMultiplier(registry, doubleSupportSplitRatio);

      int iters = 100;
      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.7 * random.nextDouble();
         double exitRatio = 0.7 * random.nextDouble();

         omega.set(omega0);
         doubleSupportSplitRatio.set(splitRatio);
         exitCMPDurationInPercentOfSteptime.set(exitRatio);

         double currentDoubleSupport = 2.0 * random.nextDouble();
         double upcomingDoubleSupport = 2.0 * random.nextDouble();
         double singleSupport = 5.0 * random.nextDouble();

         currentDoubleSupportDuration.set(currentDoubleSupport);
         upcomingDoubleSupportDuration.set(upcomingDoubleSupport);
         singleSupportDuration.set(singleSupport);

         double alpha1 = random.nextDouble();
         double alpha2 = random.nextDouble();
         double startOfSpline = Math.min(alpha1, alpha2) * singleSupport;
         double endOfSpline = Math.max(alpha1, alpha2) * singleSupport;

         double segmentLength = endOfSpline - startOfSpline;
         double timeInSegment = random.nextDouble() * segmentLength;
         double timeInCurrentState = timeInSegment + startOfSpline;
         double timeRemaining = singleSupport - timeInCurrentState;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupport);

         multiplier.compute(doubleSupportDurations, timeRemaining, false, omega0, false);

         Assert.assertEquals("", 0.0, multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", 0.0, multiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testTwoCMPSSSecondSegmentUsingInitialICP()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPDurationInPercentOfSteptime = new DoubleYoVariable("exitCMPDurationInPercentOfSteptime", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      DoubleYoVariable currentDoubleSupportDuration = new DoubleYoVariable("currentDoubleSupportDuration", registry);
      DoubleYoVariable upcomingDoubleSupportDuration = new DoubleYoVariable("upcomingDoubleSupportDuration", registry);

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(currentDoubleSupportDuration);
      doubleSupportDurations.add(upcomingDoubleSupportDuration);

      DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration", registry);
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      singleSupportDurations.add(singleSupportDuration);

      PreviousExitCMPProjectionMultiplier multiplier = new PreviousExitCMPProjectionMultiplier(registry, doubleSupportSplitRatio);

      int iters = 100;
      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.7 * random.nextDouble();
         double exitRatio = 0.7 * random.nextDouble();

         omega.set(omega0);
         doubleSupportSplitRatio.set(splitRatio);
         exitCMPDurationInPercentOfSteptime.set(exitRatio);

         double currentDoubleSupport = 2.0 * random.nextDouble();
         double upcomingDoubleSupport = 2.0 * random.nextDouble();
         double singleSupport = 5.0 * random.nextDouble();

         currentDoubleSupportDuration.set(currentDoubleSupport);
         upcomingDoubleSupportDuration.set(upcomingDoubleSupport);
         singleSupportDuration.set(singleSupport);

         double alpha1 = random.nextDouble();
         double alpha2 = random.nextDouble();
         double startOfSpline = Math.min(alpha1, alpha2) * singleSupport;
         double endOfSpline = Math.max(alpha1, alpha2) * singleSupport;

         double segmentLength = endOfSpline - startOfSpline;
         double timeInSegment = random.nextDouble() * segmentLength;
         double timeInCurrentState = timeInSegment + startOfSpline;
         double timeRemaining = singleSupport - timeInCurrentState;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupport);

         multiplier.compute(doubleSupportDurations, timeRemaining, false, omega0, true);

         Assert.assertEquals("", 0.0, multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", 0.0, multiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testTwoCMPTransfer()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPDurationInPercentOfSteptime = new DoubleYoVariable("exitCMPDurationInPercentOfSteptime", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      DoubleYoVariable currentDoubleSupportDuration = new DoubleYoVariable("currentDoubleSupportDuration", registry);
      DoubleYoVariable upcomingDoubleSupportDuration = new DoubleYoVariable("upcomingDoubleSupportDuration", registry);

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(currentDoubleSupportDuration);
      doubleSupportDurations.add(upcomingDoubleSupportDuration);

      DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration", registry);
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      singleSupportDurations.add(singleSupportDuration);

      PreviousExitCMPProjectionMultiplier multiplier = new PreviousExitCMPProjectionMultiplier(registry, doubleSupportSplitRatio);
      TransferPreviousExitCMPProjectionMatrix transferExitCMPProjectionMatrix = new TransferPreviousExitCMPProjectionMatrix(doubleSupportSplitRatio);

      CubicMatrix cubicMatrix = new CubicMatrix();
      CubicDerivativeMatrix cubicDerivativeMatrix = new CubicDerivativeMatrix();

      int iters = 100;
      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.7 * random.nextDouble();
         double exitRatio = 0.7 * random.nextDouble();

         omega.set(omega0);
         doubleSupportSplitRatio.set(splitRatio);
         exitCMPDurationInPercentOfSteptime.set(exitRatio);

         double currentDoubleSupport = 2.0 * random.nextDouble();
         double upcomingDoubleSupport = 2.0 * random.nextDouble();
         double singleSupport = 5.0 * random.nextDouble();

         currentDoubleSupportDuration.set(currentDoubleSupport);
         upcomingDoubleSupportDuration.set(upcomingDoubleSupport);
         singleSupportDuration.set(singleSupport);

         double timeRemaining = random.nextDouble() * currentDoubleSupport;

         double alpha1 = random.nextDouble();
         double alpha2 = random.nextDouble();
         double startOfSpline = Math.min(alpha1, alpha2) * singleSupport;
         double endOfSpline = Math.max(alpha1, alpha2) * singleSupport;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupport);

         transferExitCMPProjectionMatrix.compute(currentDoubleSupport, omega0, false);

         cubicDerivativeMatrix.setSegmentDuration(currentDoubleSupport);
         cubicDerivativeMatrix.update(timeRemaining);
         cubicMatrix.setSegmentDuration(currentDoubleSupport);
         cubicMatrix.update(timeRemaining);

         DenseMatrix64F positionMatrixOut = new DenseMatrix64F(1, 1);
         DenseMatrix64F velocityMatrixOut = new DenseMatrix64F(1, 1);
         CommonOps.mult(cubicMatrix, transferExitCMPProjectionMatrix, positionMatrixOut);
         CommonOps.mult(cubicDerivativeMatrix, transferExitCMPProjectionMatrix, velocityMatrixOut);

         multiplier.compute(doubleSupportDurations, timeRemaining, true, omega0, false);

         Assert.assertEquals("", positionMatrixOut.get(0, 0), multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", velocityMatrixOut.get(0, 0), multiplier.getVelocityMultiplier(), epsilon);

         multiplier.reset();
         Assert.assertEquals("", 0.0, multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", 0.0, multiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testTwoCMPTransferUsingInitialICP()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPDurationInPercentOfSteptime = new DoubleYoVariable("exitCMPDurationInPercentOfSteptime", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      DoubleYoVariable currentDoubleSupportDuration = new DoubleYoVariable("currentDoubleSupportDuration", registry);
      DoubleYoVariable upcomingDoubleSupportDuration = new DoubleYoVariable("upcomingDoubleSupportDuration", registry);

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(currentDoubleSupportDuration);
      doubleSupportDurations.add(upcomingDoubleSupportDuration);

      DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration", registry);
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      singleSupportDurations.add(singleSupportDuration);

      PreviousExitCMPProjectionMultiplier multiplier = new PreviousExitCMPProjectionMultiplier(registry, doubleSupportSplitRatio);
      TransferPreviousExitCMPProjectionMatrix transferExitCMPProjectionMatrix = new TransferPreviousExitCMPProjectionMatrix(doubleSupportSplitRatio);

      CubicMatrix cubicMatrix = new CubicMatrix();
      CubicDerivativeMatrix cubicDerivativeMatrix = new CubicDerivativeMatrix();

      int iters = 100;
      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.7 * random.nextDouble();
         double exitRatio = 0.7 * random.nextDouble();

         omega.set(omega0);
         doubleSupportSplitRatio.set(splitRatio);
         exitCMPDurationInPercentOfSteptime.set(exitRatio);

         double currentDoubleSupport = 2.0 * random.nextDouble();
         double upcomingDoubleSupport = 2.0 * random.nextDouble();
         double singleSupport = 5.0 * random.nextDouble();

         currentDoubleSupportDuration.set(currentDoubleSupport);
         upcomingDoubleSupportDuration.set(upcomingDoubleSupport);
         singleSupportDuration.set(singleSupport);

         double timeRemaining = random.nextDouble() * currentDoubleSupport;

         double alpha1 = random.nextDouble();
         double alpha2 = random.nextDouble();
         double startOfSpline = Math.min(alpha1, alpha2) * singleSupport;
         double endOfSpline = Math.max(alpha1, alpha2) * singleSupport;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupport);

         transferExitCMPProjectionMatrix.compute(currentDoubleSupport, omega0, true);

         cubicDerivativeMatrix.setSegmentDuration(currentDoubleSupport);
         cubicDerivativeMatrix.update(timeRemaining);
         cubicMatrix.setSegmentDuration(currentDoubleSupport);
         cubicMatrix.update(timeRemaining);

         DenseMatrix64F positionMatrixOut = new DenseMatrix64F(1, 1);
         DenseMatrix64F velocityMatrixOut = new DenseMatrix64F(1, 1);
         CommonOps.mult(cubicMatrix, transferExitCMPProjectionMatrix, positionMatrixOut);
         CommonOps.mult(cubicDerivativeMatrix, transferExitCMPProjectionMatrix, velocityMatrixOut);

         multiplier.compute(doubleSupportDurations, timeRemaining, true, omega0, true);

         Assert.assertEquals("", positionMatrixOut.get(0, 0), multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", velocityMatrixOut.get(0, 0), multiplier.getVelocityMultiplier(), epsilon);

         multiplier.reset();
         Assert.assertEquals("", 0.0, multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", 0.0, multiplier.getVelocityMultiplier(), epsilon);
      }
   }
}
