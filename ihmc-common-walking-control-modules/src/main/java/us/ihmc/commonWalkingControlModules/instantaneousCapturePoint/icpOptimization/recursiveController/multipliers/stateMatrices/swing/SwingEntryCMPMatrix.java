package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.stateMatrices.swing;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.InterpolationTools;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public class SwingEntryCMPMatrix extends DenseMatrix64F
{
   private final List<YoDouble> swingSplitFractions;

   private final YoDouble startOfSplineTime;

   private final boolean blendFromInitial;
   private final double minimumBlendingTime;

   public SwingEntryCMPMatrix(List<YoDouble> swingSplitFractions, YoDouble startOfSplineTime, boolean blendFromInitial, double minimumBlendingTime)
   {
      super(4, 1);

      this.swingSplitFractions = swingSplitFractions;

      this.startOfSplineTime = startOfSplineTime;
      this.blendFromInitial = blendFromInitial;
      this.minimumBlendingTime = minimumBlendingTime;
   }

   public void reset()
   {
      zero();
   }

   public void compute(List<YoDouble> singleSupportDurations, double omega0)
   {
      zero();

      double projection;

      if (blendFromInitial)
      {
         double currentSwingOnEntryCMP = swingSplitFractions.get(0).getDoubleValue() * singleSupportDurations.get(0).getDoubleValue();
         double splineDurationOnEntryCMP = currentSwingOnEntryCMP - startOfSplineTime.getDoubleValue();

         if (startOfSplineTime.getDoubleValue() >= minimumBlendingTime)
         { // recurse backward from the current corner point to the start of spline location
            projection = 1.0 - Math.exp(-omega0 * splineDurationOnEntryCMP);
         }
         else
         { // haven't phased in completely yet, so its a combination of both recursion and projection
            double phaseAtStart = startOfSplineTime.getDoubleValue() / minimumBlendingTime;

            double recursionMultiplier = 1.0 - Math.exp(-omega0 * splineDurationOnEntryCMP);
            double projectionMultiplier = 1.0 - Math.exp(omega0 * startOfSplineTime.getDoubleValue());

            projection = InterpolationTools.linearInterpolate(projectionMultiplier, recursionMultiplier, phaseAtStart);
         }
      }
      else
      { // project forward from the initial ICP state
         projection = 1.0 - Math.exp(omega0 * startOfSplineTime.getDoubleValue());
      }

      set(0, 0, projection);
      set(1, 0, omega0 * (projection - 1.0));
   }
}
