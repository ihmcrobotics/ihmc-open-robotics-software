package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.InterpolationTools;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public class SwingStateEndMatrix extends DenseMatrix64F
{
   private final List<YoDouble> swingSplitFractions;
   private final List<YoDouble> transferSplitFractions;

   private final YoDouble startOfSplineTime;
   private final YoDouble endOfSplineTime;

   private final boolean blendFromInitial;
   private final double minimumBlendingTime;

   public SwingStateEndMatrix(List<YoDouble> swingSplitFractions, List<YoDouble> transferSplitFractions,
         YoDouble startOfSplineTime, YoDouble endOfSplineTime, boolean blendFromInitial, double minimumBlendingTime)
   {
      super(4, 1);

      this.swingSplitFractions = swingSplitFractions;
      this.transferSplitFractions = transferSplitFractions;

      this.startOfSplineTime = startOfSplineTime;
      this.endOfSplineTime = endOfSplineTime;

      this.blendFromInitial = blendFromInitial;
      this.minimumBlendingTime = minimumBlendingTime;
   }

   public void reset()
   {
      zero();
   }

   public void compute(List<YoDouble> singleSupportDurations, List<YoDouble> doubleSupportDurations,
         double omega0)
   {
      // recurse backward from upcoming corner point to current corner point, then project forward to end of spline
      double currentSwingOnExitCMP = (1.0 - swingSplitFractions.get(0).getDoubleValue()) * singleSupportDurations.get(0).getDoubleValue();
      double nextTransferOnExitCMP = transferSplitFractions.get(1).getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();
      double timeOnExitCMP = currentSwingOnExitCMP + nextTransferOnExitCMP;

      double currentSwingOnEntryCMP = swingSplitFractions.get(0).getDoubleValue() * singleSupportDurations.get(0).getDoubleValue();
      double splineDurationOnExitCMP = endOfSplineTime.getDoubleValue() - currentSwingOnEntryCMP;

      double projection = Math.exp(omega0 * (splineDurationOnExitCMP - timeOnExitCMP));

      zero();

      set(2, 0, projection);
      set(3, 0, omega0 * projection);

      if (blendFromInitial)
      {
         double splineDurationOnEntryCMP = currentSwingOnEntryCMP - startOfSplineTime.getDoubleValue();

         if (startOfSplineTime.getDoubleValue() >= minimumBlendingTime)
         { // recurse backward from the upcoming corner point to the current corner point, then recurse back to the start of spline location
            projection = Math.exp(-omega0 * (timeOnExitCMP + splineDurationOnEntryCMP));
         }
         else
         { // haven't phased in completely yet, so its a combination of both recursion and projection
            double phaseAtStart = startOfSplineTime.getDoubleValue() / minimumBlendingTime;

            double recursionMultiplier = Math.exp(-omega0 * (timeOnExitCMP + splineDurationOnEntryCMP));
            double projectionMultiplier = 0.0;

            projection = InterpolationTools.linearInterpolate(projectionMultiplier, recursionMultiplier, phaseAtStart);
         }

         set(0, 0, projection);
         set(1, 0, omega0 * projection);
      }
   }
}

