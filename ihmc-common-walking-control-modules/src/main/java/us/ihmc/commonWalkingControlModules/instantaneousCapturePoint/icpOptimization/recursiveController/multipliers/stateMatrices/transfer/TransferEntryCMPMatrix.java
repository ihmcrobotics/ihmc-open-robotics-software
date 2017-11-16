package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.stateMatrices.transfer;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public class TransferEntryCMPMatrix extends DenseMatrix64F
{
   private final List<YoDouble> swingSplitFractions;
   private final List<YoDouble> transferSplitFractions;

   public TransferEntryCMPMatrix(List<YoDouble> swingSplitFractions, List<YoDouble> transferSplitFractions)
   {
      super(4, 1);

      this.swingSplitFractions = swingSplitFractions;
      this.transferSplitFractions = transferSplitFractions;
   }

   public void reset()
   {
      zero();
   }

   public void compute(int numberOfFootstepsToConsider,
         List<YoDouble> singleSupportDurations, List<YoDouble> doubleSupportDurations,
         boolean useTwoCMPs, double omega0)
   {
      zero();

      double currentTransferOnEntry = (1.0 - transferSplitFractions.get(0).getDoubleValue()) * doubleSupportDurations.get(0).getDoubleValue();
      double timeOnEntryCMP;

      if (numberOfFootstepsToConsider == 0)
      { // the ending corner point is the current entry corner point
         timeOnEntryCMP = 0.0;
      }
      else
      { // the ending corner point is after the next step
         if (useTwoCMPs)
         {
            // first must recurse back from the ending corner point on the upcoming foot, then from the exit corner to the entry corner, then
            // project forward to the end of double support
            double currentSwingOnEntry = swingSplitFractions.get(0).getDoubleValue() * singleSupportDurations.get(0).getDoubleValue();
            timeOnEntryCMP = currentTransferOnEntry + currentSwingOnEntry;
         }
         else
         {
            // first must recurse back from the ending corner point on the upcoming foot, then from the exit corner to the entry corner, then
            // project forward to the end of double support
            double nextTransferOnEntry = transferSplitFractions.get(1).getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();
            timeOnEntryCMP = currentTransferOnEntry + singleSupportDurations.get(0).getDoubleValue() + nextTransferOnEntry;
         }
      }

      double projection = Math.exp(omega0 * (currentTransferOnEntry - timeOnEntryCMP));

      set(2, 0, 1.0 - projection);
      set(3, 0, -omega0 * projection);
   }
}
