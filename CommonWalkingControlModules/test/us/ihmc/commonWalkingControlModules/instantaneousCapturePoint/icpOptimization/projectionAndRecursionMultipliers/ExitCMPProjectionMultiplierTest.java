package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.ExitCMPProjectionMultiplier;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.interpolation.CubicProjectionDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.interpolation.CubicProjectionMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.swing.SwingExitCMPProjectionMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer.TransferExitCMPProjectionMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import java.util.ArrayList;

import static com.badlogic.gdx.math.MathUtils.random;

public class ExitCMPProjectionMultiplierTest
{
   private static final double epsilon = 0.0001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testOneCMPSS()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable defaultDoubleSupportSplitRatio = new DoubleYoVariable("defaultDoubleSupportSplitRatio", registry);
      DoubleYoVariable upcomingDoubleSupportSplitRatio = new DoubleYoVariable("upcomingDoubleSupportSplitRatio", registry);
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

      ExitCMPProjectionMultiplier multiplier = new ExitCMPProjectionMultiplier(registry, defaultDoubleSupportSplitRatio, upcomingDoubleSupportSplitRatio,
            exitCMPDurationInPercentOfSteptime, startOfSplineTime, endOfSplineTime, totalTrajectoryTime);

      int iters = 100;
      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.7 * random.nextDouble();
         double exitRatio = 0.7 * random.nextDouble();

         omega.set(omega0);
         defaultDoubleSupportSplitRatio.set(splitRatio);
         upcomingDoubleSupportSplitRatio.set(splitRatio);
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

         double projection = 1.0 - Math.exp(-omega0 * timeRemaining);
         double velocityProjection = -omega0 * Math.exp(-omega0 * timeRemaining);

         multiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, false, false, omega0, false);

         Assert.assertEquals("", projection, multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", velocityProjection, multiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testOneCMPSSUsingInitialICP()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable defaultDoubleSupportSplitRatio = new DoubleYoVariable("defaultDoubleSupportSplitRatio", registry);
      DoubleYoVariable upcomingDoubleSupportSplitRatio = new DoubleYoVariable("upcomingDoubleSupportSplitRatio", registry);
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

      ExitCMPProjectionMultiplier multiplier = new ExitCMPProjectionMultiplier(registry, defaultDoubleSupportSplitRatio, upcomingDoubleSupportSplitRatio,
            exitCMPDurationInPercentOfSteptime, startOfSplineTime, endOfSplineTime, totalTrajectoryTime);

      int iters = 100;
      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.7 * random.nextDouble();
         double exitRatio = 0.7 * random.nextDouble();

         omega.set(omega0);
         defaultDoubleSupportSplitRatio.set(splitRatio);
         upcomingDoubleSupportSplitRatio.set(splitRatio);
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

         double projection = 1.0 - Math.exp(-omega0 * timeRemaining);
         double velocityProjection = -omega0 * Math.exp(-omega0 * timeRemaining);

         multiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, false, false, omega0, true);

         Assert.assertEquals("", projection, multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", velocityProjection, multiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testOneCMPTransfer()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable defaultDoubleSupportSplitRatio = new DoubleYoVariable("defaultDoubleSupportSplitRatio", registry);
      DoubleYoVariable upcomingDoubleSupportSplitRatio = new DoubleYoVariable("upcomingDoubleSupportSplitRatio", registry);
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

      ExitCMPProjectionMultiplier multiplier = new ExitCMPProjectionMultiplier(registry, defaultDoubleSupportSplitRatio, upcomingDoubleSupportSplitRatio,
            exitCMPDurationInPercentOfSteptime, startOfSplineTime, endOfSplineTime, totalTrajectoryTime);
      TransferExitCMPProjectionMatrix transferExitCMPProjectionMatrix = new TransferExitCMPProjectionMatrix(defaultDoubleSupportSplitRatio);

      CubicProjectionMatrix cubicProjectionMatrix = new CubicProjectionMatrix();
      CubicProjectionDerivativeMatrix cubicProjectionDerivativeMatrix = new CubicProjectionDerivativeMatrix();

      int iters = 100;
      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.7 * random.nextDouble();
         double exitRatio = 0.7 * random.nextDouble();

         omega.set(omega0);
         defaultDoubleSupportSplitRatio.set(splitRatio);
         upcomingDoubleSupportSplitRatio.set(splitRatio);
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

         transferExitCMPProjectionMatrix.compute(currentDoubleSupport, false, omega0, false);

         cubicProjectionDerivativeMatrix.setSegmentDuration(currentDoubleSupport);
         cubicProjectionDerivativeMatrix.update(timeRemaining);
         cubicProjectionMatrix.setSegmentDuration(currentDoubleSupport);
         cubicProjectionMatrix.update(timeRemaining);

         DenseMatrix64F positionMatrixOut = new DenseMatrix64F(1, 1);
         DenseMatrix64F velocityMatrixOut = new DenseMatrix64F(1, 1);
         CommonOps.mult(cubicProjectionMatrix, transferExitCMPProjectionMatrix, positionMatrixOut);
         CommonOps.mult(cubicProjectionMatrix, transferExitCMPProjectionMatrix, positionMatrixOut);
         CommonOps.mult(cubicProjectionDerivativeMatrix, transferExitCMPProjectionMatrix, velocityMatrixOut);
         CommonOps.mult(cubicProjectionDerivativeMatrix, transferExitCMPProjectionMatrix, velocityMatrixOut);

         multiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, false, true, omega0, false);

         Assert.assertEquals("", positionMatrixOut.get(0, 0), multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", velocityMatrixOut.get(0, 0), multiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testOneCMPTransferUsingInitialICP()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable defaultDoubleSupportSplitRatio = new DoubleYoVariable("defaultDoubleSupportSplitRatio", registry);
      DoubleYoVariable upcomingDoubleSupportSplitRatio = new DoubleYoVariable("upcomingDoubleSupportSplitRatio", registry);
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

      ExitCMPProjectionMultiplier multiplier = new ExitCMPProjectionMultiplier(registry, defaultDoubleSupportSplitRatio, upcomingDoubleSupportSplitRatio,
            exitCMPDurationInPercentOfSteptime, startOfSplineTime, endOfSplineTime, totalTrajectoryTime);
      TransferExitCMPProjectionMatrix transferExitCMPProjectionMatrix = new TransferExitCMPProjectionMatrix(defaultDoubleSupportSplitRatio);

      CubicProjectionMatrix cubicProjectionMatrix = new CubicProjectionMatrix();
      CubicProjectionDerivativeMatrix cubicProjectionDerivativeMatrix = new CubicProjectionDerivativeMatrix();

      int iters = 100;
      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.7 * random.nextDouble();
         double exitRatio = 0.7 * random.nextDouble();

         omega.set(omega0);
         defaultDoubleSupportSplitRatio.set(splitRatio);
         upcomingDoubleSupportSplitRatio.set(splitRatio);
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

         transferExitCMPProjectionMatrix.compute(currentDoubleSupport, false, omega0, true);

         cubicProjectionDerivativeMatrix.setSegmentDuration(currentDoubleSupport);
         cubicProjectionDerivativeMatrix.update(timeRemaining);
         cubicProjectionMatrix.setSegmentDuration(currentDoubleSupport);
         cubicProjectionMatrix.update(timeRemaining);

         DenseMatrix64F positionMatrixOut = new DenseMatrix64F(1, 1);
         DenseMatrix64F velocityMatrixOut = new DenseMatrix64F(1, 1);
         CommonOps.mult(cubicProjectionMatrix, transferExitCMPProjectionMatrix, positionMatrixOut);
         CommonOps.mult(cubicProjectionMatrix, transferExitCMPProjectionMatrix, positionMatrixOut);
         CommonOps.mult(cubicProjectionDerivativeMatrix, transferExitCMPProjectionMatrix, velocityMatrixOut);
         CommonOps.mult(cubicProjectionDerivativeMatrix, transferExitCMPProjectionMatrix, velocityMatrixOut);

         multiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, false, true, omega0, true);

         Assert.assertEquals("", positionMatrixOut.get(0, 0), multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", velocityMatrixOut.get(0, 0), multiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testTwoCMPSSFirstSegment()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable defaultDoubleSupportSplitRatio = new DoubleYoVariable("defaultDoubleSupportSplitRatio", registry);
      DoubleYoVariable upcomingDoubleSupportSplitRatio = new DoubleYoVariable("upcomingDoubleSupportSplitRatio", registry);
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

      ExitCMPProjectionMultiplier multiplier = new ExitCMPProjectionMultiplier(registry, defaultDoubleSupportSplitRatio, upcomingDoubleSupportSplitRatio,
            exitCMPDurationInPercentOfSteptime, startOfSplineTime, endOfSplineTime, totalTrajectoryTime);

      int iters = 100;
      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.7 * random.nextDouble();
         double exitRatio = 0.7 * random.nextDouble();

         omega.set(omega0);
         defaultDoubleSupportSplitRatio.set(splitRatio);
         upcomingDoubleSupportSplitRatio.set(splitRatio);
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

         double stepDuration = currentDoubleSupport + singleSupport;

         double endOfDoubleSupportDuration = (1.0 - splitRatio) * currentDoubleSupport;
         double upcomingInitialDoubleSupportDuration = splitRatio * upcomingDoubleSupport;
         double exitDuration = exitRatio * stepDuration;
         double entryDuration = (1.0 - exitRatio) * stepDuration;

         double exitRecursionTime = upcomingInitialDoubleSupportDuration - exitDuration;
         double entryRecursionTime = timeInCurrentState + endOfDoubleSupportDuration - entryDuration;

         double exitRecursion = Math.exp(omega0 * exitRecursionTime);
         double entryRecursion = Math.exp(omega0 * entryRecursionTime);

         double recursionMultiplier = entryRecursion * (1.0 - exitRecursion);
         double velocityRecursionMultiplier = omega0 * entryRecursion * (1.0 - exitRecursion);

         multiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, true, false, omega0, false);

         Assert.assertEquals("", recursionMultiplier, multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", velocityRecursionMultiplier, multiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testTwoCMPSSFirstSegmentUsingInitialICP()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable defaultDoubleSupportSplitRatio = new DoubleYoVariable("defaultDoubleSupportSplitRatio", registry);
      DoubleYoVariable upcomingDoubleSupportSplitRatio = new DoubleYoVariable("upcomingDoubleSupportSplitRatio", registry);
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

      ExitCMPProjectionMultiplier multiplier = new ExitCMPProjectionMultiplier(registry, defaultDoubleSupportSplitRatio, upcomingDoubleSupportSplitRatio,
            exitCMPDurationInPercentOfSteptime, startOfSplineTime, endOfSplineTime, totalTrajectoryTime);

      int iters = 100;
      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.7 * random.nextDouble();
         double exitRatio = 0.7 * random.nextDouble();

         omega.set(omega0);
         defaultDoubleSupportSplitRatio.set(splitRatio);
         upcomingDoubleSupportSplitRatio.set(splitRatio);
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

         double stepDuration = currentDoubleSupport + singleSupport;

         double endOfDoubleSupportDuration = (1.0 - splitRatio) * currentDoubleSupport;
         double upcomingInitialDoubleSupportDuration = splitRatio * upcomingDoubleSupport;
         double exitDuration = exitRatio * stepDuration;
         double entryDuration = (1.0 - exitRatio) * stepDuration;

         double exitRecursionTime = upcomingInitialDoubleSupportDuration - exitDuration;
         double entryRecursionTime = timeInCurrentState + endOfDoubleSupportDuration - entryDuration;

         double exitRecursion = Math.exp(omega0 * exitRecursionTime);
         double entryRecursion = Math.exp(omega0 * entryRecursionTime);

         double recursionMultiplier = entryRecursion * (1.0 - exitRecursion);
         double velocityRecursionMultiplier = omega0 * entryRecursion * (1.0 - exitRecursion);

         multiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, true, false, omega0, true);

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
      DoubleYoVariable defaultDoubleSupportSplitRatio = new DoubleYoVariable("defaultDoubleSupportSplitRatio", registry);
      DoubleYoVariable upcomingDoubleSupportSplitRatio = new DoubleYoVariable("upcomingDoubleSupportSplitRatio", registry);
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

      ExitCMPProjectionMultiplier multiplier = new ExitCMPProjectionMultiplier(registry, defaultDoubleSupportSplitRatio, upcomingDoubleSupportSplitRatio,
            exitCMPDurationInPercentOfSteptime, startOfSplineTime, endOfSplineTime, totalTrajectoryTime);

      int iters = 100;
      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.7 * random.nextDouble();
         double exitRatio = 0.7 * random.nextDouble();

         omega.set(omega0);
         defaultDoubleSupportSplitRatio.set(splitRatio);
         upcomingDoubleSupportSplitRatio.set(splitRatio);
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

         double recursionMultiplier = 1.0 - Math.exp(-omega0 * timeRemaining);
         double velocityRecursionMultiplier = -omega0 * Math.exp(-omega0 * timeRemaining);

         Assert.assertEquals("", recursionMultiplier, multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", velocityRecursionMultiplier, multiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testTwoCMPSSThirdSegmentUsingInitialICP()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable defaultDoubleSupportSplitRatio = new DoubleYoVariable("defaultDoubleSupportSplitRatio", registry);
      DoubleYoVariable upcomingDoubleSupportSplitRatio = new DoubleYoVariable("upcomingDoubleSupportSplitRatio", registry);
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

      ExitCMPProjectionMultiplier multiplier = new ExitCMPProjectionMultiplier(registry, defaultDoubleSupportSplitRatio, upcomingDoubleSupportSplitRatio,
            exitCMPDurationInPercentOfSteptime, startOfSplineTime, endOfSplineTime, totalTrajectoryTime);

      int iters = 100;
      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.7 * random.nextDouble();
         double exitRatio = 0.7 * random.nextDouble();

         omega.set(omega0);
         defaultDoubleSupportSplitRatio.set(splitRatio);
         upcomingDoubleSupportSplitRatio.set(splitRatio);
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

         double recursionMultiplier = 1.0 - Math.exp(-omega0 * timeRemaining);
         double velocityRecursionMultiplier = -omega0 * Math.exp(-omega0 * timeRemaining);

         Assert.assertEquals("", velocityRecursionMultiplier, multiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testTwoCMPSSSecondSegment()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable defaultDoubleSupportSplitRatio = new DoubleYoVariable("defaultDoubleSupportSplitRatio", registry);
      DoubleYoVariable upcomingDoubleSupportSplitRatio = new DoubleYoVariable("upcomingDoubleSupportSplitRatio", registry);
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

      ExitCMPProjectionMultiplier multiplier = new ExitCMPProjectionMultiplier(registry, defaultDoubleSupportSplitRatio, upcomingDoubleSupportSplitRatio,
            exitCMPDurationInPercentOfSteptime, startOfSplineTime, endOfSplineTime, totalTrajectoryTime);
      SwingExitCMPProjectionMatrix swingExitCMPProjectionMatrix = new SwingExitCMPProjectionMatrix(defaultDoubleSupportSplitRatio, upcomingDoubleSupportSplitRatio,
            exitCMPDurationInPercentOfSteptime, startOfSplineTime, endOfSplineTime, totalTrajectoryTime);
      CubicProjectionMatrix cubicProjectionMatrix = new CubicProjectionMatrix();
      CubicProjectionDerivativeMatrix cubicProjectionDerivativeMatrix = new CubicProjectionDerivativeMatrix();

      int iters = 100;
      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.7 * random.nextDouble();
         double exitRatio = 0.7 * random.nextDouble();

         omega.set(omega0);
         defaultDoubleSupportSplitRatio.set(splitRatio);
         upcomingDoubleSupportSplitRatio.set(splitRatio);
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

         double timeRemainingInSegment = segmentLength - timeInSegment;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupport);

         swingExitCMPProjectionMatrix.compute(upcomingDoubleSupport, currentDoubleSupport, singleSupport, omega0, false);

         cubicProjectionDerivativeMatrix.setSegmentDuration(segmentLength);
         cubicProjectionDerivativeMatrix.update(timeRemainingInSegment);
         cubicProjectionMatrix.setSegmentDuration(segmentLength);
         cubicProjectionMatrix.update(timeRemainingInSegment);

         DenseMatrix64F positionMatrixOut = new DenseMatrix64F(1, 1);
         DenseMatrix64F velocityMatrixOut = new DenseMatrix64F(1, 1);
         CommonOps.mult(cubicProjectionMatrix, swingExitCMPProjectionMatrix, positionMatrixOut);
         CommonOps.mult(cubicProjectionDerivativeMatrix, swingExitCMPProjectionMatrix, velocityMatrixOut);

         multiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, true, false, omega0, false);

         Assert.assertEquals("", positionMatrixOut.get(0, 0), multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", velocityMatrixOut.get(0, 0), multiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testTwoCMPSSSecondSegmentUsingInitialICP()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable defaultDoubleSupportSplitRatio = new DoubleYoVariable("defaultDoubleSupportSplitRatio", registry);
      DoubleYoVariable upcomingDoubleSupportSplitRatio = new DoubleYoVariable("upcomingDoubleSupportSplitRatio", registry);
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

      ExitCMPProjectionMultiplier multiplier = new ExitCMPProjectionMultiplier(registry, defaultDoubleSupportSplitRatio, upcomingDoubleSupportSplitRatio,
            exitCMPDurationInPercentOfSteptime, startOfSplineTime, endOfSplineTime, totalTrajectoryTime);
      SwingExitCMPProjectionMatrix swingExitCMPProjectionMatrix = new SwingExitCMPProjectionMatrix(defaultDoubleSupportSplitRatio, upcomingDoubleSupportSplitRatio,
            exitCMPDurationInPercentOfSteptime, startOfSplineTime, endOfSplineTime, totalTrajectoryTime);
      CubicProjectionMatrix cubicProjectionMatrix = new CubicProjectionMatrix();
      CubicProjectionDerivativeMatrix cubicProjectionDerivativeMatrix = new CubicProjectionDerivativeMatrix();

      int iters = 100;
      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.7 * random.nextDouble();
         double exitRatio = 0.7 * random.nextDouble();

         omega.set(omega0);
         defaultDoubleSupportSplitRatio.set(splitRatio);
         upcomingDoubleSupportSplitRatio.set(splitRatio);
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

         double timeRemainingInSegment = segmentLength - timeInSegment;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupport);

         swingExitCMPProjectionMatrix.compute(upcomingDoubleSupport, currentDoubleSupport, singleSupport, omega0, true);

         cubicProjectionDerivativeMatrix.setSegmentDuration(segmentLength);
         cubicProjectionDerivativeMatrix.update(timeRemainingInSegment);
         cubicProjectionMatrix.setSegmentDuration(segmentLength);
         cubicProjectionMatrix.update(timeRemainingInSegment);

         DenseMatrix64F positionMatrixOut = new DenseMatrix64F(1, 1);
         DenseMatrix64F velocityMatrixOut = new DenseMatrix64F(1, 1);
         CommonOps.mult(cubicProjectionMatrix, swingExitCMPProjectionMatrix, positionMatrixOut);
         CommonOps.mult(cubicProjectionDerivativeMatrix, swingExitCMPProjectionMatrix, velocityMatrixOut);

         multiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, true, false, omega0, true);

         Assert.assertEquals("", positionMatrixOut.get(0, 0), multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", velocityMatrixOut.get(0, 0), multiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testTwoCMPTransfer()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable defaultDoubleSupportSplitRatio = new DoubleYoVariable("defaultDoubleSupportSplitRatio", registry);
      DoubleYoVariable upcomingDoubleSupportSplitRatio = new DoubleYoVariable("upcomingDoubleSupportSplitRatio", registry);
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

      ExitCMPProjectionMultiplier multiplier = new ExitCMPProjectionMultiplier(registry, defaultDoubleSupportSplitRatio, upcomingDoubleSupportSplitRatio,
            exitCMPDurationInPercentOfSteptime, startOfSplineTime, endOfSplineTime, totalTrajectoryTime);
      TransferExitCMPProjectionMatrix transferExitCMPProjectionMatrix = new TransferExitCMPProjectionMatrix(defaultDoubleSupportSplitRatio);

      CubicProjectionMatrix cubicProjectionMatrix = new CubicProjectionMatrix();
      CubicProjectionDerivativeMatrix cubicProjectionDerivativeMatrix = new CubicProjectionDerivativeMatrix();

      int iters = 100;
      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.7 * random.nextDouble();
         double exitRatio = 0.7 * random.nextDouble();

         omega.set(omega0);
         defaultDoubleSupportSplitRatio.set(splitRatio);
         upcomingDoubleSupportSplitRatio.set(splitRatio);
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

         transferExitCMPProjectionMatrix.compute(currentDoubleSupport, true, omega0, false);

         cubicProjectionDerivativeMatrix.setSegmentDuration(currentDoubleSupport);
         cubicProjectionDerivativeMatrix.update(timeRemaining);
         cubicProjectionMatrix.setSegmentDuration(currentDoubleSupport);
         cubicProjectionMatrix.update(timeRemaining);

         DenseMatrix64F positionMatrixOut = new DenseMatrix64F(1, 1);
         DenseMatrix64F velocityMatrixOut = new DenseMatrix64F(1, 1);
         CommonOps.mult(cubicProjectionMatrix, transferExitCMPProjectionMatrix, positionMatrixOut);
         CommonOps.mult(cubicProjectionDerivativeMatrix, transferExitCMPProjectionMatrix, velocityMatrixOut);

         multiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, true, true, omega0, false);

         Assert.assertEquals("", positionMatrixOut.get(0, 0), multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", velocityMatrixOut.get(0, 0), multiplier.getVelocityMultiplier(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testTwoCMPTransferUsingInitialICP()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable defaultDoubleSupportSplitRatio = new DoubleYoVariable("defaultDoubleSupportSplitRatio", registry);
      DoubleYoVariable upcomingDoubleSupportSplitRatio = new DoubleYoVariable("upcomingDoubleSupportSplitRatio", registry);
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

      ExitCMPProjectionMultiplier multiplier = new ExitCMPProjectionMultiplier(registry, defaultDoubleSupportSplitRatio, upcomingDoubleSupportSplitRatio,
            exitCMPDurationInPercentOfSteptime, startOfSplineTime, endOfSplineTime, totalTrajectoryTime);
      TransferExitCMPProjectionMatrix transferExitCMPProjectionMatrix = new TransferExitCMPProjectionMatrix(defaultDoubleSupportSplitRatio);

      CubicProjectionMatrix cubicProjectionMatrix = new CubicProjectionMatrix();
      CubicProjectionDerivativeMatrix cubicProjectionDerivativeMatrix = new CubicProjectionDerivativeMatrix();

      int iters = 100;
      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.7 * random.nextDouble();
         double exitRatio = 0.7 * random.nextDouble();

         omega.set(omega0);
         defaultDoubleSupportSplitRatio.set(splitRatio);
         upcomingDoubleSupportSplitRatio.set(splitRatio);
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

         transferExitCMPProjectionMatrix.compute(currentDoubleSupport, true, omega0, true);

         cubicProjectionDerivativeMatrix.setSegmentDuration(currentDoubleSupport);
         cubicProjectionDerivativeMatrix.update(timeRemaining);
         cubicProjectionMatrix.setSegmentDuration(currentDoubleSupport);
         cubicProjectionMatrix.update(timeRemaining);

         DenseMatrix64F positionMatrixOut = new DenseMatrix64F(1, 1);
         DenseMatrix64F velocityMatrixOut = new DenseMatrix64F(1, 1);
         CommonOps.mult(cubicProjectionMatrix, transferExitCMPProjectionMatrix, positionMatrixOut);
         CommonOps.mult(cubicProjectionDerivativeMatrix, transferExitCMPProjectionMatrix, velocityMatrixOut);

         multiplier.compute(doubleSupportDurations, singleSupportDurations, timeRemaining, true, true, omega0, true);

         Assert.assertEquals("", positionMatrixOut.get(0, 0), multiplier.getPositionMultiplier(), epsilon);
         Assert.assertEquals("", velocityMatrixOut.get(0, 0), multiplier.getVelocityMultiplier(), epsilon);
      }
   }
}
