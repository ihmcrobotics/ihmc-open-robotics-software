package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.interpolation.CubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.interpolation.CubicMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.swing.SwingStateEndRecursionMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer.TransferStateEndRecursionMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class CurrentStateProjectionMultiplier
{
   private final CubicMatrix cubicMatrix;
   private final CubicDerivativeMatrix cubicDerivativeMatrix;

   private final TransferStateEndRecursionMatrix transferStateEndRecursionMatrix;
   private final SwingStateEndRecursionMatrix swingStateEndRecursionMatrix;

   private final DoubleYoVariable defaultDoubleSupportSplitRatio;
   private final DoubleYoVariable upcomingDoubleSupportSplitRatio;

   private final DoubleYoVariable startOfSplineTime;
   private final DoubleYoVariable endOfSplineTime;
   private final DoubleYoVariable totalTrajectoryTime;

   private final DenseMatrix64F matrixOut = new DenseMatrix64F(1, 1);

   private final DoubleYoVariable positionMultiplier;
   private final DoubleYoVariable velocityMultiplier;

   public CurrentStateProjectionMultiplier(YoVariableRegistry registry, DoubleYoVariable defaultDoubleSupportSplitRatio, DoubleYoVariable upcomingDoubleSupportSplitRatio,
         DoubleYoVariable startOfSplineTime, DoubleYoVariable endOfSplineTime, DoubleYoVariable totalTrajectoryTime)
   {
      positionMultiplier = new DoubleYoVariable("CurrentStateProjectionMultiplier", registry);
      velocityMultiplier = new DoubleYoVariable("CurrentStateVelocityProjectionMultiplier", registry);

      this.defaultDoubleSupportSplitRatio = defaultDoubleSupportSplitRatio;
      this.upcomingDoubleSupportSplitRatio = upcomingDoubleSupportSplitRatio;

      this.startOfSplineTime = startOfSplineTime;
      this.endOfSplineTime = endOfSplineTime;
      this.totalTrajectoryTime = totalTrajectoryTime;

      transferStateEndRecursionMatrix = new TransferStateEndRecursionMatrix();
      swingStateEndRecursionMatrix = new SwingStateEndRecursionMatrix(defaultDoubleSupportSplitRatio, upcomingDoubleSupportSplitRatio, startOfSplineTime,
            endOfSplineTime, totalTrajectoryTime);

      cubicMatrix = new CubicMatrix();
      cubicDerivativeMatrix = new CubicDerivativeMatrix();
   }

   public void reset()
   {
      positionMultiplier.set(0.0);
      velocityMultiplier.set(0.0);
   }

   public double getPositionMultiplier()
   {
      return positionMultiplier.getDoubleValue();
   }

   public double getVelocityMultiplier()
   {
      return velocityMultiplier.getDoubleValue();
   }

   public void compute(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations, double timeRemaining,
         boolean useTwoCMPs, boolean isInTransfer, double omega0, boolean useInitialICP)
   {
      timeRemaining = Math.max(timeRemaining, 0.0);
      double positionMultiplier, velocityMultiplier;
      if (isInTransfer)
      {
         positionMultiplier = computeInTransfer(doubleSupportDurations, timeRemaining, omega0, useInitialICP);
      }
      else
      {
         if (useTwoCMPs)
            positionMultiplier = computeSegmentedProjection(doubleSupportDurations, singleSupportDurations, timeRemaining, omega0, useInitialICP);
         else
            positionMultiplier = computeInSwingOneCMP(timeRemaining, omega0);
      }
      this.positionMultiplier.set(positionMultiplier);

      if (isInTransfer)
      {
         velocityMultiplier = computeInTransferVelocity();
      }
      else
      {
         if (useTwoCMPs)
            velocityMultiplier = computeSegmentedVelocityProjection(timeRemaining, omega0);
         else
            velocityMultiplier = computeInSwingOneCMPVelocity(omega0);
      }

      this.velocityMultiplier.set(velocityMultiplier);
   }

   private double computeInTransfer(ArrayList<DoubleYoVariable> doubleSupportDurations, double timeRemaining, double omega0, boolean useInitialICP)
   {
      transferStateEndRecursionMatrix.compute(doubleSupportDurations, omega0, useInitialICP);

      double splineDuration = doubleSupportDurations.get(0).getDoubleValue();

      cubicDerivativeMatrix.setSegmentDuration(splineDuration);
      cubicDerivativeMatrix.update(timeRemaining);
      cubicMatrix.setSegmentDuration(splineDuration);
      cubicMatrix.update(timeRemaining);
      CommonOps.mult(cubicMatrix, transferStateEndRecursionMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeInSwingOneCMP(double timeRemaining, double omega0)
   {
      return Math.exp(-omega0 * timeRemaining);
   }

   private double computeSegmentedProjection(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         double timeRemaining, double omega0, boolean useInitialICP)
   {
      double timeInState = totalTrajectoryTime.getDoubleValue() - timeRemaining;

      if (timeInState < startOfSplineTime.getDoubleValue())
         return computeFirstSegmentProjection(doubleSupportDurations, singleSupportDurations, timeInState, omega0, useInitialICP);
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         return computeThirdSegmentProjection(timeRemaining, omega0);
      else
         return computeSecondSegmentProjection(doubleSupportDurations, singleSupportDurations, timeRemaining, omega0, useInitialICP);
   }

   private double computeFirstSegmentProjection(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         double timeInState, double omega0, boolean useInitialICP)
   {
      if (!useInitialICP)
      {
         double upcomingDoubleSupportDuration = doubleSupportDurations.get(1).getDoubleValue();
         double currentDoubleSupportDuration = doubleSupportDurations.get(0).getDoubleValue();
         double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();

         double stepDuration = currentDoubleSupportDuration + singleSupportDuration;

         double upcomingInitialDoubleSupportDuration = upcomingDoubleSupportSplitRatio.getDoubleValue() * upcomingDoubleSupportDuration;
         double endOfDoubleSupportDuration = (1.0 - defaultDoubleSupportSplitRatio.getDoubleValue()) * currentDoubleSupportDuration;

         double recursionTime = timeInState + upcomingInitialDoubleSupportDuration + endOfDoubleSupportDuration - stepDuration;
         return Math.exp(omega0 * recursionTime);
      }
      else
      {
         return 0.0;
      }
   }

   private double computeSecondSegmentProjection(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         double timeRemaining, double omega0, boolean useInitialICP)
   {
      swingStateEndRecursionMatrix.compute(doubleSupportDurations, singleSupportDurations, omega0, useInitialICP);

      double lastSegmentDuration = totalTrajectoryTime.getDoubleValue() - endOfSplineTime.getDoubleValue();
      double timeRemainingInSpline = timeRemaining - lastSegmentDuration;
      double splineDuration = endOfSplineTime.getDoubleValue() - startOfSplineTime.getDoubleValue();

      cubicDerivativeMatrix.setSegmentDuration(splineDuration);
      cubicDerivativeMatrix.update(timeRemainingInSpline);
      cubicMatrix.setSegmentDuration(splineDuration);
      cubicMatrix.update(timeRemainingInSpline);

      CommonOps.mult(cubicMatrix, swingStateEndRecursionMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeThirdSegmentProjection(double timeRemaining, double omega0)
   {
      return computeInSwingOneCMP(timeRemaining, omega0);
   }

   private double computeInTransferVelocity()
   {
      CommonOps.mult(cubicDerivativeMatrix, transferStateEndRecursionMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeInSwingOneCMPVelocity(double omega0)
   {
      return omega0 * positionMultiplier.getDoubleValue();
   }

   private double computeSegmentedVelocityProjection(double timeRemaining, double omega0)
   {
      double timeInState = totalTrajectoryTime.getDoubleValue() - timeRemaining;

      if (timeInState < startOfSplineTime.getDoubleValue())
         return computeFirstSegmentVelocityProjection(omega0);
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         return computeThirdSegmentVelocityProjection(omega0);
      else
         return computeSecondSegmentVelocityProjection();
   }

   private double computeFirstSegmentVelocityProjection(double omega0)
   {
      return omega0 * positionMultiplier.getDoubleValue();
   }

   private double computeSecondSegmentVelocityProjection()
   {
      CommonOps.mult(cubicDerivativeMatrix, swingStateEndRecursionMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeThirdSegmentVelocityProjection(double omega0)
   {
      return omega0 *  positionMultiplier.getDoubleValue();
   }
}
