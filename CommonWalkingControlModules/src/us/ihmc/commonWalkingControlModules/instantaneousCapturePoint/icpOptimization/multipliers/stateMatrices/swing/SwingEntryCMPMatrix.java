package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.List;

public class SwingEntryCMPMatrix extends DenseMatrix64F
{
   private final List<DoubleYoVariable> swingSplitFractions;

   private final DoubleYoVariable startOfSplineTime;

   private final boolean blendFromInitial;

   public SwingEntryCMPMatrix(List<DoubleYoVariable> swingSplitFractions, DoubleYoVariable startOfSplineTime, boolean blendFromInitial)
   {
      super(4, 1);

      this.swingSplitFractions = swingSplitFractions;

      this.startOfSplineTime = startOfSplineTime;
      this.blendFromInitial = blendFromInitial;
   }

   public void reset()
   {
      zero();
   }

   public void compute(List<DoubleYoVariable> singleSupportDurations, double omega0)
   {
      zero();

      double projection;

      if (blendFromInitial)
      { // recurse backward from the current corner point to the start of spline location
         double currentSwingOnEntryCMP = swingSplitFractions.get(0).getDoubleValue() * singleSupportDurations.get(0).getDoubleValue();
         double splineDurationOnEntryCMP = currentSwingOnEntryCMP - startOfSplineTime.getDoubleValue();

         projection = Math.exp(-omega0 * splineDurationOnEntryCMP);
      }
      else
      { // project forward from the initial ICP state
         projection = Math.exp(omega0 * startOfSplineTime.getDoubleValue());
      }

      set(0, 0, 1.0 - projection);
      set(1, 0, -omega0 * projection);
   }
}
