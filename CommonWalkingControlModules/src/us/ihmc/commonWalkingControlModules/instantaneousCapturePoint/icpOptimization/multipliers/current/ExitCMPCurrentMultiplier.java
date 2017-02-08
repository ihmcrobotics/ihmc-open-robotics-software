package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing.SwingExitCMPMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.CubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.CubicMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class ExitCMPCurrentMultiplier
{
   private final CubicMatrix cubicMatrix;
   private final CubicDerivativeMatrix cubicDerivativeMatrix;

   private final SwingExitCMPMatrix swingExitCMPMatrix;

   private final DoubleYoVariable exitCMPRatio;
   private final DoubleYoVariable upcomingDoubleSupportSplitRatio;

   private final DoubleYoVariable startOfSplineTime;
   private final DoubleYoVariable endOfSplineTime;
   private final DoubleYoVariable totalTrajectoryTime;

   private final DenseMatrix64F matrixOut = new DenseMatrix64F(1, 1);

   private final DoubleYoVariable positionMultiplier;
   private final DoubleYoVariable velocityMultiplier;

   public ExitCMPCurrentMultiplier(DoubleYoVariable upcomingDoubleSupportSplitRatio, DoubleYoVariable exitCMPRatio, DoubleYoVariable startOfSplineTime,
         DoubleYoVariable endOfSplineTime, DoubleYoVariable totalTrajectoryTime, YoVariableRegistry registry)
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

      swingExitCMPMatrix = new SwingExitCMPMatrix(upcomingDoubleSupportSplitRatio, exitCMPRatio, endOfSplineTime);
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

   public void compute(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations, double timeInState,
         boolean useTwoCMPs, boolean isInTransfer, double omega0)
   {
      double positionMultiplier, velocityMultiplier;
      if (isInTransfer)
      {
         positionMultiplier = computeInTransfer();
      }
      else
      {
         if (useTwoCMPs)
            positionMultiplier = computeSegmentedSwing(doubleSupportDurations, singleSupportDurations, timeInState, omega0);
         else
            positionMultiplier = computeInSwingOneCMP();
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
            velocityMultiplier = computeInSwingOneCMPVelocity();
      }

      this.velocityMultiplier.set(velocityMultiplier);
   }

   private double computeInTransfer()
   {
      return 0.0;
   }

   private double computeInSwingOneCMP()
   {
      return 0.0;
   }

   private double computeInTransferVelocity()
   {
      return 0.0;
   }

   private double computeInSwingOneCMPVelocity()
   {
      return 0.0;
   }

   private double computeSegmentedSwing(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         double timeInState, double omega0)
   {
      if (timeInState < startOfSplineTime.getDoubleValue())
         return computeSwingFirstSegment();
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         return computeSwingThirdSegment(doubleSupportDurations, singleSupportDurations, timeInState, omega0);
      else
         return computeSwingSecondSegment(doubleSupportDurations, singleSupportDurations, timeInState, omega0);
   }





   private double computeSwingFirstSegment()
   {
      return 0.0;
   }

   private double computeSwingSecondSegment(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         double timeInState, double omega0)
   {
      swingExitCMPMatrix.compute(doubleSupportDurations, singleSupportDurations, omega0);

      double timeInSpline = timeInState - startOfSplineTime.getDoubleValue();
      double splineDuration = endOfSplineTime.getDoubleValue() - startOfSplineTime.getDoubleValue();

      cubicDerivativeMatrix.setSegmentDuration(splineDuration);
      cubicDerivativeMatrix.update(timeInSpline, true);
      cubicMatrix.setSegmentDuration(splineDuration);
      cubicMatrix.update(timeInSpline, true);

      CommonOps.mult(cubicMatrix, swingExitCMPMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeSwingThirdSegment(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
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
