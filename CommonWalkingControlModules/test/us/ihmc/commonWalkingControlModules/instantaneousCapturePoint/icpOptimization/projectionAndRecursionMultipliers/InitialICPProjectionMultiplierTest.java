package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.interpolation.CubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.interpolation.CubicMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.swing.SwingInitialICPProjectionMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer.TransferInitialICPProjectionMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import java.util.ArrayList;

import static com.badlogic.gdx.math.MathUtils.random;

public class InitialICPProjectionMultiplierTest
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

      InitialICPProjectionMultiplier multiplier = new InitialICPProjectionMultiplier(registry, startOfSplineTime, endOfSplineTime,
            totalTrajectoryTime);

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

         multiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, false, false, omega0, false);

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

      InitialICPProjectionMultiplier multiplier = new InitialICPProjectionMultiplier(registry, startOfSplineTime, endOfSplineTime,
            totalTrajectoryTime);

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
         double timeInState = singleSupport - timeRemaining;

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

         double recursionMultiplier = Math.exp(omega0 * timeInState);

         double velocityMultiplier = omega0 * recursionMultiplier;

         multiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, false, false, omega0, true);

         Assert.assertEquals("", recursionMultiplier, multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", velocityMultiplier, multiplier.getVelocityMultiplier(), epsilon);

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

      InitialICPProjectionMultiplier multiplier = new InitialICPProjectionMultiplier(registry, startOfSplineTime, endOfSplineTime,
            totalTrajectoryTime);
      TransferInitialICPProjectionMatrix transferStateEndRecursionMatrix = new TransferInitialICPProjectionMatrix();

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

         transferStateEndRecursionMatrix.compute(omega0);

         cubicDerivativeMatrix.setSegmentDuration(currentDoubleSupport);
         cubicDerivativeMatrix.update(timeRemaining);
         cubicMatrix.setSegmentDuration(currentDoubleSupport);
         cubicMatrix.update(timeRemaining);

         DenseMatrix64F positionMatrixOut = new DenseMatrix64F(1, 1);
         DenseMatrix64F velocityMatrixOut = new DenseMatrix64F(1, 1);
         CommonOps.mult(cubicMatrix, transferStateEndRecursionMatrix, positionMatrixOut);
         CommonOps.mult(cubicDerivativeMatrix, transferStateEndRecursionMatrix, velocityMatrixOut);

         multiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, false, true, omega0, true);

         Assert.assertEquals("", positionMatrixOut.get(0, 0), multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", velocityMatrixOut.get(0, 0), multiplier.getVelocityMultiplier(), epsilon);

         multiplier.reset();
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

      InitialICPProjectionMultiplier multiplier = new InitialICPProjectionMultiplier(registry, startOfSplineTime, endOfSplineTime,
            totalTrajectoryTime);

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

         multiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, false, true, omega0, false);

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

      InitialICPProjectionMultiplier multiplier = new InitialICPProjectionMultiplier(registry, startOfSplineTime, endOfSplineTime,
            totalTrajectoryTime);

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

         multiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, true, false, omega0, false);

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

      InitialICPProjectionMultiplier multiplier = new InitialICPProjectionMultiplier(registry, startOfSplineTime, endOfSplineTime,
            totalTrajectoryTime);

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

         multiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, true, false, omega0, true);

         double projectionMultiplier = Math.exp(timeInCurrentState * omega0);
         double velocityMultiplier = omega0 * Math.exp(timeInCurrentState * omega0);

         Assert.assertEquals("", projectionMultiplier, multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", velocityMultiplier, multiplier.getVelocityMultiplier(), epsilon);

         multiplier.reset();
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

      InitialICPProjectionMultiplier multiplier = new InitialICPProjectionMultiplier(registry, startOfSplineTime, endOfSplineTime,
            totalTrajectoryTime);

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

         multiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, true, false, omega0, false);

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

      InitialICPProjectionMultiplier multiplier = new InitialICPProjectionMultiplier(registry, startOfSplineTime, endOfSplineTime,
            totalTrajectoryTime);

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

         multiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, true, false, omega0, true);

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

      InitialICPProjectionMultiplier multiplier = new InitialICPProjectionMultiplier(registry, startOfSplineTime, endOfSplineTime,
            totalTrajectoryTime);

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
         double timeRemainingInSegment = segmentLength - timeInSegment;

         double timeInCurrentState = timeInSegment + startOfSpline;
         double timeRemaining = singleSupport - timeInCurrentState;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupport);

         multiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, true, false, omega0, false);

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

      InitialICPProjectionMultiplier multiplier = new InitialICPProjectionMultiplier(registry, startOfSplineTime, endOfSplineTime,
            totalTrajectoryTime);

      SwingInitialICPProjectionMatrix swingInitialICPProjectionMatrix = new SwingInitialICPProjectionMatrix(startOfSplineTime);
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

         double alpha1 = random.nextDouble();
         double alpha2 = random.nextDouble();
         double startOfSpline = Math.min(alpha1, alpha2) * singleSupport;
         double endOfSpline = Math.max(alpha1, alpha2) * singleSupport;

         double segmentLength = endOfSpline - startOfSpline;
         double timeInSegment = random.nextDouble() * segmentLength;
         double timeRemainingInSegment = segmentLength - timeInSegment;

         double timeInCurrentState = timeInSegment + startOfSpline;
         double timeRemaining = singleSupport - timeInCurrentState;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupport);

         swingInitialICPProjectionMatrix.compute(omega0);

         cubicDerivativeMatrix.setSegmentDuration(segmentLength);
         cubicDerivativeMatrix.update(timeRemainingInSegment);
         cubicMatrix.setSegmentDuration(segmentLength);
         cubicMatrix.update(timeRemainingInSegment);

         DenseMatrix64F positionMatrixOut = new DenseMatrix64F(1, 1);
         DenseMatrix64F velocityMatrixOut = new DenseMatrix64F(1, 1);
         CommonOps.mult(cubicMatrix, swingInitialICPProjectionMatrix, positionMatrixOut);
         CommonOps.mult(cubicDerivativeMatrix, swingInitialICPProjectionMatrix, velocityMatrixOut);

         multiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, true, false, omega0, true);

         Assert.assertEquals("", positionMatrixOut.get(0, 0), multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", velocityMatrixOut.get(0, 0), multiplier.getVelocityMultiplier(), epsilon);

         multiplier.reset();
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

      InitialICPProjectionMultiplier multiplier = new InitialICPProjectionMultiplier(registry, startOfSplineTime, endOfSplineTime,
            totalTrajectoryTime);

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

         multiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, true, true, omega0, false);

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

      InitialICPProjectionMultiplier multiplier = new InitialICPProjectionMultiplier(registry, startOfSplineTime, endOfSplineTime,
            totalTrajectoryTime);
      TransferInitialICPProjectionMatrix transferStateEndRecursionMatrix = new TransferInitialICPProjectionMatrix();

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

         transferStateEndRecursionMatrix.compute(omega0);

         cubicDerivativeMatrix.setSegmentDuration(currentDoubleSupport);
         cubicDerivativeMatrix.update(timeRemaining);
         cubicMatrix.setSegmentDuration(currentDoubleSupport);
         cubicMatrix.update(timeRemaining);

         DenseMatrix64F positionMatrixOut = new DenseMatrix64F(1, 1);
         DenseMatrix64F velocityMatrixOut = new DenseMatrix64F(1, 1);
         CommonOps.mult(cubicMatrix, transferStateEndRecursionMatrix, positionMatrixOut);
         CommonOps.mult(cubicDerivativeMatrix, transferStateEndRecursionMatrix, velocityMatrixOut);

         multiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, true, true, omega0, true);

         Assert.assertEquals("", positionMatrixOut.get(0, 0), multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", velocityMatrixOut.get(0, 0), multiplier.getVelocityMultiplier(), epsilon);

         multiplier.reset();
         Assert.assertEquals("", 0.0, multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", 0.0, multiplier.getVelocityMultiplier(), epsilon);
      }
   }
}
