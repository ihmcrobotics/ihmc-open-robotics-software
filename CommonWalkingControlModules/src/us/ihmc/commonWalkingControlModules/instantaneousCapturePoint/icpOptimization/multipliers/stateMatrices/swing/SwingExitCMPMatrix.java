package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.List;

public class SwingExitCMPMatrix extends DenseMatrix64F
{
   private final List<DoubleYoVariable> swingSplitFractions;
   private final List<DoubleYoVariable> transferSplitFractions;

   private final DoubleYoVariable endOfSplineTime;

   public SwingExitCMPMatrix(List<DoubleYoVariable> swingSplitFractions, List<DoubleYoVariable> transferSplitFractions,
         DoubleYoVariable endOfSplineTime)
   {
      super(4, 1);

      this.swingSplitFractions = swingSplitFractions;
      this.transferSplitFractions = transferSplitFractions;

      this.endOfSplineTime = endOfSplineTime;
   }

   public void reset()
   {
      zero();
   }

   public void compute(List<DoubleYoVariable> singleSupportDurations, List<DoubleYoVariable> doubleSupportDurations,
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

      set(2, 0, 1.0 - projection);
      set(3, 0, -omega0 * projection);
   }
}
