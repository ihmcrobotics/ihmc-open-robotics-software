package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing.NewSwingStateEndMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.transfer.NewTransferStateEndMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.interpolation.CubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.interpolation.CubicMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class NewStateEndCurrentMultiplier
{
   private final CubicMatrix cubicMatrix;
   private final CubicDerivativeMatrix cubicDerivativeMatrix;

   private final NewTransferStateEndMatrix transferStateEndMatrix;
   private final NewSwingStateEndMatrix swingStateEndMatrix;

   private final DoubleYoVariable exitCMPRatio;
   private final DoubleYoVariable upcomingDoubleSupportSplitRatio;
   private final DoubleYoVariable defaultDoubleSupportSplitRatio;

   private final DoubleYoVariable startOfSplineTime;
   private final DoubleYoVariable endOfSplineTime;
   private final DoubleYoVariable totalTrajectoryTime;

   private final DenseMatrix64F matrixOut = new DenseMatrix64F(1, 1);

   private final DoubleYoVariable positionMultiplier;
   private final DoubleYoVariable velocityMultiplier;

   private final boolean projectForward;

   public NewStateEndCurrentMultiplier(DoubleYoVariable upcomingDoubleSupportSplitRatio, DoubleYoVariable defaultDoubleSupportSplitRatio,
         DoubleYoVariable exitCMPRatio, DoubleYoVariable startOfSplineTime, DoubleYoVariable endOfSplineTime, DoubleYoVariable totalTrajectoryTime,
         boolean projectForward, YoVariableRegistry registry)
   {
      positionMultiplier = new DoubleYoVariable("StateEndCurrentMultiplier", registry);
      velocityMultiplier = new DoubleYoVariable("StateEndCurrentVelocityMultiplier", registry);

      this.exitCMPRatio = exitCMPRatio;
      this.upcomingDoubleSupportSplitRatio = upcomingDoubleSupportSplitRatio;
      this.defaultDoubleSupportSplitRatio = defaultDoubleSupportSplitRatio;

      this.startOfSplineTime = startOfSplineTime;
      this.endOfSplineTime = endOfSplineTime;
      this.totalTrajectoryTime = totalTrajectoryTime;

      this.projectForward = projectForward;

      cubicMatrix = new CubicMatrix();
      cubicDerivativeMatrix = new CubicDerivativeMatrix();

      transferStateEndMatrix = new NewTransferStateEndMatrix(defaultDoubleSupportSplitRatio);
      swingStateEndMatrix = new NewSwingStateEndMatrix(upcomingDoubleSupportSplitRatio, exitCMPRatio, endOfSplineTime);
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
      double positionMultiplier, velocityMultiplier;

      if (isInTransfer)
      {
         positionMultiplier = computeInTransfer(doubleSupportDurations, omega0, timeRemaining, useTwoCMPs);
      }
      else
      {
         if (useTwoCMPs)
            positionMultiplier = computeSwingSegmented(doubleSupportDurations, singleSupportDurations, timeRemaining, omega0);
         else
            positionMultiplier = computeInSwingOneCMP(doubleSupportDurations, singleSupportDurations, timeRemaining, omega0);
      }
      this.positionMultiplier.set(positionMultiplier);

      if (isInTransfer)
      {
         velocityMultiplier = computeInTransferVelocity();
      }
      else
      {
         if (useTwoCMPs)
            velocityMultiplier = computeSwingSegmentedVelocity(timeRemaining, omega0);
         else
            velocityMultiplier = computeInSwingOneCMPVelocity(omega0);
      }

      this.velocityMultiplier.set(velocityMultiplier);
   }

   private double computeInTransfer(ArrayList<DoubleYoVariable> doubleSupportDurations, double omega0, double timeRemaining, boolean useTwoCMPs)
   {
      transferStateEndMatrix.compute(doubleSupportDurations, omega0, useTwoCMPs);

      double splineDuration = doubleSupportDurations.get(0).getDoubleValue();

      cubicDerivativeMatrix.setSegmentDuration(splineDuration);
      cubicDerivativeMatrix.update(timeRemaining);
      cubicMatrix.setSegmentDuration(splineDuration);
      cubicMatrix.update(timeRemaining);

      CommonOps.mult(cubicMatrix, transferStateEndMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeInTransferVelocity()
   {
      CommonOps.mult(cubicDerivativeMatrix, transferStateEndMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }





   private double computeInSwingOneCMP(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations, double timeRemaining, double omega0)
   {
      return computeInSwingOneCMP(doubleSupportDurations.get(1).getDoubleValue(), doubleSupportDurations.get(0).getDoubleValue(),
            singleSupportDurations.get(0).getDoubleValue(), timeRemaining, omega0);
   }

   private double computeInSwingOneCMP(double upcomingDoubleSupportDuration, double doubleSupportDuration, double singleSupportDuration, double timeRemaining, double omega0)
   {
      double timeInState = totalTrajectoryTime.getDoubleValue() - timeRemaining;

      double stepDuration = doubleSupportDuration + singleSupportDuration;
      double timeSpentOnExitCMP = exitCMPRatio.getDoubleValue() * stepDuration;
      double upcomingInitialDoubleSupportDuration = upcomingDoubleSupportSplitRatio.getDoubleValue() * upcomingDoubleSupportDuration;

      double duration = timeInState - singleSupportDuration + timeSpentOnExitCMP - upcomingInitialDoubleSupportDuration;
      return Math.exp(omega0 * duration);
   }


   private double computeInSwingOneCMPVelocity(double omega0)
   {
      return omega0 * positionMultiplier.getDoubleValue();
   }




   private double computeSwingSegmented(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         double timeRemaining, double omega0)
   {
      double timeInState = totalTrajectoryTime.getDoubleValue() - timeRemaining;

      if (timeInState < startOfSplineTime.getDoubleValue())
         return computeSwingFirstSegment(doubleSupportDurations, singleSupportDurations, timeInState, omega0);
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         return computeSwingThirdSegment(doubleSupportDurations, singleSupportDurations, timeRemaining, omega0);
      else
         return computeSwingSecondSegment(doubleSupportDurations, singleSupportDurations, timeRemaining, omega0);
   }

   private double computeSwingFirstSegment(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         double timeInState, double omega0)
   {
      if (projectForward)
      {
         return 0.0;
      }
      else
      {
         double currentDoubleSupportDuration = doubleSupportDurations.get(0).getDoubleValue();
         double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();
         double stepDuration = currentDoubleSupportDuration + singleSupportDuration;


         double timeSpentOnEntryCMP = (1.0 - exitCMPRatio.getDoubleValue()) * stepDuration;
         double endOfDoubleSupportDuration = (1.0 - defaultDoubleSupportSplitRatio.getDoubleValue()) * currentDoubleSupportDuration;
         double initialSingleSupportDuration = timeSpentOnEntryCMP - endOfDoubleSupportDuration;

         double duration = timeInState - initialSingleSupportDuration;
         double projection = Math.exp(omega0 * duration);

         return projection;
      }
   }

   private double computeSwingSecondSegment(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         double timeRemaining, double omega0)
   {
      swingStateEndMatrix.compute(doubleSupportDurations, singleSupportDurations, omega0);

      double lastSegmentDuration = totalTrajectoryTime.getDoubleValue() - endOfSplineTime.getDoubleValue();
      double timeRemainingInSpline = timeRemaining - lastSegmentDuration;
      double splineDuration = endOfSplineTime.getDoubleValue() - startOfSplineTime.getDoubleValue();

      cubicDerivativeMatrix.setSegmentDuration(splineDuration);
      cubicDerivativeMatrix.update(timeRemainingInSpline);
      cubicMatrix.setSegmentDuration(splineDuration);
      cubicMatrix.update(timeRemainingInSpline);

      CommonOps.mult(cubicMatrix, swingStateEndMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeSwingThirdSegment(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations, double timeRemaining, double omega0)
   {
      return computeInSwingOneCMP(doubleSupportDurations, singleSupportDurations, timeRemaining, omega0);
   }




   private double computeSwingSegmentedVelocity(double timeRemaining, double omega0)
   {
      double timeInState = totalTrajectoryTime.getDoubleValue() - timeRemaining;

      if (timeInState < startOfSplineTime.getDoubleValue())
         return computeSwingFirstSegmentVelocity(omega0);
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         return computeSwingThirdSegmentVelocity(omega0);
      else
         return computeSwingSecondSegmentVelocity();
   }

   private double computeSwingFirstSegmentVelocity(double omega0)
   {
      return omega0 * positionMultiplier.getDoubleValue();
   }

   private double computeSwingSecondSegmentVelocity()
   {
      CommonOps.mult(cubicDerivativeMatrix, swingStateEndMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeSwingThirdSegmentVelocity(double omega0)
   {
      return omega0 * positionMultiplier.getDoubleValue();
   }
}
