package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing.NewSwingExitCMPMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.interpolation.CubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.interpolation.CubicMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.swing.SwingExitCMPProjectionMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer.TransferExitCMPProjectionMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class NewExitCMPCurrentMultiplier
{
   private final CubicMatrix cubicMatrix;
   private final CubicDerivativeMatrix cubicDerivativeMatrix;

   private final NewSwingExitCMPMatrix swingExitCMPMatrix;

   private final DoubleYoVariable exitCMPRatio;
   private final DoubleYoVariable upcomingDoubleSupportSplitRatio;

   private final DoubleYoVariable startOfSplineTime;
   private final DoubleYoVariable endOfSplineTime;
   private final DoubleYoVariable totalTrajectoryTime;

   private final DenseMatrix64F matrixOut = new DenseMatrix64F(1, 1);

   private final DoubleYoVariable positionMultiplier;
   private final DoubleYoVariable velocityMultiplier;

   public NewExitCMPCurrentMultiplier(YoVariableRegistry registry, DoubleYoVariable upcomingDoubleSupportSplitRatio,
         DoubleYoVariable exitCMPRatio, DoubleYoVariable startOfSplineTime, DoubleYoVariable endOfSplineTime, DoubleYoVariable totalTrajectoryTime)
   {
      positionMultiplier = new DoubleYoVariable("ExitCMPCurrentMultiplier", registry);
      velocityMultiplier = new DoubleYoVariable("ExitCMPCurrentVelocityMultiplier", registry);

      this.exitCMPRatio = exitCMPRatio;
      this.upcomingDoubleSupportSplitRatio = upcomingDoubleSupportSplitRatio;

      this.startOfSplineTime = startOfSplineTime;
      this.endOfSplineTime = endOfSplineTime;
      this.totalTrajectoryTime = totalTrajectoryTime;

      cubicMatrix = new CubicMatrix();
      cubicDerivativeMatrix = new CubicDerivativeMatrix();

      swingExitCMPMatrix = new NewSwingExitCMPMatrix(upcomingDoubleSupportSplitRatio, exitCMPRatio, endOfSplineTime);
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
         boolean useTwoCMPs, boolean isInTransfer, double omega0)
   {
      double timeInState = totalTrajectoryTime.getDoubleValue() - timeRemaining;

      double positionMultiplier, velocityMultiplier;
      if (isInTransfer)
      {
         positionMultiplier = computeInTransfer();
      }
      else
      {
         if (useTwoCMPs)
            positionMultiplier = computeSegmentedSwing(doubleSupportDurations, singleSupportDurations, timeRemaining, timeInState, omega0);
         else
            positionMultiplier = computeInSwingOneCMP(doubleSupportDurations, singleSupportDurations, timeInState, omega0);
      }
      this.positionMultiplier.set(positionMultiplier);

      if (isInTransfer)
      {
         velocityMultiplier = computeInTransferVelocity();
      }
      else
      {
         if (useTwoCMPs)
            velocityMultiplier = computeSegmentedSwingVelocity(timeInState, omega0);
         else
            velocityMultiplier = computeInSwingOneCMPVelocity(omega0);
      }

      this.velocityMultiplier.set(velocityMultiplier);
   }

   private double computeInTransfer()
   {
      return 0.0;
   }

   private double computeInSwingOneCMP(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         double timeInState, double omega0)
   {
      double upcomingDoubleSupportDuration = doubleSupportDurations.get(1).getDoubleValue();
      double currentDoubleSupportDuration = doubleSupportDurations.get(0).getDoubleValue();
      double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();
      double stepDuration = currentDoubleSupportDuration + singleSupportDuration;

      double upcomingInitialDoubleSupportDuration = upcomingDoubleSupportSplitRatio.getDoubleValue() * upcomingDoubleSupportDuration;
      double timeSpentOnExitCMP = exitCMPRatio.getDoubleValue() * stepDuration;

      double duration = timeInState - singleSupportDuration + timeSpentOnExitCMP - upcomingInitialDoubleSupportDuration;

      return 1.0 - Math.exp(omega0 * duration);
   }

   private double computeInTransferVelocity()
   {
      return 0.0;
   }

   private double computeInSwingOneCMPVelocity(double omega0)
   {
      return omega0 * (positionMultiplier.getDoubleValue() - 1.0);
   }

   private double computeSegmentedSwing(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         double timeRemaining, double timeInState, double omega0)
   {
      if (timeInState < startOfSplineTime.getDoubleValue())
         return computeSwingFirstSegment();
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         return computeSwingThirdSegment(doubleSupportDurations, singleSupportDurations, timeInState, omega0);
      else
         return computeSwingSecondSegment(doubleSupportDurations, singleSupportDurations, timeRemaining, omega0);
   }





   private double computeSwingFirstSegment()
   {
      return 0.0;
   }

   private double computeSwingSecondSegment(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         double timeRemaining, double omega0)
   {
      swingExitCMPMatrix.compute(doubleSupportDurations, singleSupportDurations, omega0);

      double lastSegmentDuration = totalTrajectoryTime.getDoubleValue() - endOfSplineTime.getDoubleValue();
      double timeRemainingInSpline = timeRemaining - lastSegmentDuration;
      double splineDuration = endOfSplineTime.getDoubleValue() - startOfSplineTime.getDoubleValue();

      cubicDerivativeMatrix.setSegmentDuration(splineDuration);
      cubicDerivativeMatrix.update(timeRemainingInSpline);
      cubicMatrix.setSegmentDuration(splineDuration);
      cubicMatrix.update(timeRemainingInSpline);

      CommonOps.mult(cubicMatrix, swingExitCMPMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeSwingThirdSegment(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         double timeInState, double omega0)
   {
      return computeInSwingOneCMP(doubleSupportDurations, singleSupportDurations, timeInState, omega0);
   }





   private double computeSegmentedSwingVelocity(double timeInState, double omega0)
   {
      if (timeInState < startOfSplineTime.getDoubleValue())
         return computeSwingFirstSegmentVelocity();
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         return computeSwingThirdSegmentVelocity(omega0);
      else
         return computeSwingSecondSegmentVelocity();
   }

   private double computeSwingFirstSegmentVelocity()
   {
      return 0.0;
   }

   private double computeSwingSecondSegmentVelocity()
   {
      CommonOps.mult(cubicDerivativeMatrix, swingExitCMPMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeSwingThirdSegmentVelocity(double omega0)
   {
      return omega0 * (positionMultiplier.getDoubleValue() - 1.0);
   }
}
