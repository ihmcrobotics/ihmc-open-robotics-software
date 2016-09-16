package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.interpolation.CubicProjectionDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.interpolation.CubicProjectionMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.swing.SwingExitCMPProjectionMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.swing.SwingInitialICPProjectionMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer.TransferExitCMPProjectionMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer.TransferInitialICPProjectionMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class InitialICPProjectionMultiplier
{
   private final CubicProjectionMatrix cubicProjectionMatrix;
   private final CubicProjectionDerivativeMatrix cubicProjectionDerivativeMatrix;

   private final TransferInitialICPProjectionMatrix transferInitialICPProjectionMatrix;
   private final SwingInitialICPProjectionMatrix swingInitialICPProjectionMatrix;

   private final DoubleYoVariable startOfSplineTime;
   private final DoubleYoVariable endOfSplineTime;
   private final DoubleYoVariable totalTrajectoryTime;

   private final DenseMatrix64F matrixOut = new DenseMatrix64F(1, 1);

   private final DoubleYoVariable positionMultiplier;
   private final DoubleYoVariable velocityMultiplier;

   public InitialICPProjectionMultiplier(YoVariableRegistry registry, DoubleYoVariable startOfSplineTime, DoubleYoVariable endOfSplineTime, DoubleYoVariable totalTrajectoryTime)
   {
      positionMultiplier = new DoubleYoVariable("InitialICPProjectionMultiplier", registry);
      velocityMultiplier = new DoubleYoVariable("InitialICPVelocityProjectionMultiplier", registry);

      this.startOfSplineTime = startOfSplineTime;
      this.endOfSplineTime = endOfSplineTime;
      this.totalTrajectoryTime = totalTrajectoryTime;

      cubicProjectionMatrix = new CubicProjectionMatrix();
      cubicProjectionDerivativeMatrix = new CubicProjectionDerivativeMatrix();

      transferInitialICPProjectionMatrix = new TransferInitialICPProjectionMatrix();
      swingInitialICPProjectionMatrix = new SwingInitialICPProjectionMatrix(startOfSplineTime);
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
      double positionMultiplier, velocityMultiplier;
      if (isInTransfer)
      {
         positionMultiplier = computeInTransfer(doubleSupportDurations, timeRemaining, omega0, useInitialICP);
      }
      else
      {
         if (useTwoCMPs)
            positionMultiplier = computeSegmentedProjection(timeRemaining, omega0, useInitialICP);
         else
            positionMultiplier = computeInSwingOneCMP(singleSupportDurations.get(0).getDoubleValue(), timeRemaining, omega0, useInitialICP);
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
      if (useInitialICP)
      {
         transferInitialICPProjectionMatrix.compute(omega0);

         double splineDuration = doubleSupportDurations.get(0).getDoubleValue();

         cubicProjectionDerivativeMatrix.setSegmentDuration(splineDuration);
         cubicProjectionDerivativeMatrix.update(timeRemaining);
         cubicProjectionMatrix.setSegmentDuration(splineDuration);
         cubicProjectionMatrix.update(timeRemaining);

         CommonOps.mult(cubicProjectionMatrix, transferInitialICPProjectionMatrix, matrixOut);

         return matrixOut.get(0, 0);
      }
      else
      {
         return 0.0;
      }
   }

   private double computeInSwingOneCMP(double singleSupportDuration, double timeRemaining, double omega0, boolean useInitialICP)
   {
      if (useInitialICP)
      {
         double timeInState = singleSupportDuration - timeRemaining;

         return Math.exp(omega0 * timeInState);
      }
      else
      {
         return 0.0;
      }
   }

   private double computeInTransferVelocity()
   {
      CommonOps.mult(cubicProjectionDerivativeMatrix, transferInitialICPProjectionMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeInSwingOneCMPVelocity(double omega0)
   {
      return omega0 * positionMultiplier.getDoubleValue();
   }

   private double computeSegmentedProjection(double timeRemaining, double omega0, boolean useInitialICP)
   {
      double timeInState = totalTrajectoryTime.getDoubleValue() - timeRemaining;

      if (timeInState < startOfSplineTime.getDoubleValue())
         return computeFirstSegmentProjection(timeInState, omega0, useInitialICP);
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         return computeThirdSegmentProjection();
      else
         return computeSecondSegmentProjection(timeRemaining, omega0, useInitialICP);
   }

   private double computeFirstSegmentProjection(double timeInState, double omega0, boolean useInitialICP)
   {
      if (useInitialICP)
      {
         return Math.exp(omega0 * timeInState);
      }
      else
      {
         return 0.0;
      }
   }

   private double computeSecondSegmentProjection(double timeRemaining, double omega0, boolean useInitialICP)
   {
      if (useInitialICP)
      {
         swingInitialICPProjectionMatrix.compute(omega0);

         double lastSegmentDuration = totalTrajectoryTime.getDoubleValue() - endOfSplineTime.getDoubleValue();
         double timeRemainingInSpline = timeRemaining - lastSegmentDuration;
         double splineDuration = endOfSplineTime.getDoubleValue() - startOfSplineTime.getDoubleValue();

         cubicProjectionDerivativeMatrix.setSegmentDuration(splineDuration);
         cubicProjectionDerivativeMatrix.update(timeRemainingInSpline);
         cubicProjectionMatrix.setSegmentDuration(splineDuration);
         cubicProjectionMatrix.update(timeRemainingInSpline);

         CommonOps.mult(cubicProjectionMatrix, swingInitialICPProjectionMatrix, matrixOut);

         return matrixOut.get(0, 0);
      }
      else
      {
         return 0.0;
      }
   }

   private double computeThirdSegmentProjection()
   {
      return 0.0;
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
      CommonOps.mult(cubicProjectionDerivativeMatrix, swingInitialICPProjectionMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeThirdSegmentVelocityProjection(double omega0)
   {
      return omega0 * positionMultiplier.getDoubleValue();
   }
}
