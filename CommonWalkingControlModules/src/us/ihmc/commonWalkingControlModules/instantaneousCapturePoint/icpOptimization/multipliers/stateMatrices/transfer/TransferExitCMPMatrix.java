package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.transfer;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public class TransferExitCMPMatrix extends DenseMatrix64F
{
   private final List<YoDouble> swingSplitFractions;
   private final List<YoDouble> transferSplitFractions;

   public TransferExitCMPMatrix(List<YoDouble> swingSplitFractions, List<YoDouble> transferSplitFractions)
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

      double timeOnEntryCMP, timeOnExitCMP, splineOnEntryCMP;
      if (useTwoCMPs)
      {
         double currentTransferOnEntry = (1.0 - transferSplitFractions.get(0).getDoubleValue()) * doubleSupportDurations.get(0).getDoubleValue();

         if (numberOfFootstepsToConsider == 0)
         { // the ending corner point is the current entry corner point
            timeOnEntryCMP = 0.0;
            timeOnExitCMP = 0.0;
            splineOnEntryCMP = currentTransferOnEntry;
         }
         else
         { // the ending corner is the Nth entry corner
            // first must recurse back from the ending corner point on the upcoming foot, then from the exit corner to the entry corner, then
            // project forward to the end of double support
            double currentSwingOnEntry = swingSplitFractions.get(0).getDoubleValue() * singleSupportDurations.get(0).getDoubleValue();
            double currentSwingOnExit = (1.0 - swingSplitFractions.get(0).getDoubleValue()) * singleSupportDurations.get(0).getDoubleValue();
            double nextTransferOnExit = transferSplitFractions.get(1).getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();
            timeOnEntryCMP = currentTransferOnEntry + currentSwingOnEntry;
            timeOnExitCMP = currentSwingOnExit + nextTransferOnExit;
            splineOnEntryCMP = currentTransferOnEntry;
         }
      }
      else
      {
         // there is no value
         timeOnEntryCMP = 0.0;
         timeOnExitCMP = 0.0;
         splineOnEntryCMP = 0.0;
      }

      double projection = Math.exp(omega0 * (splineOnEntryCMP - timeOnEntryCMP)) * (1.0 - Math.exp(-omega0 * timeOnExitCMP));

      set(2, 0, projection);
      set(3, 0, omega0 * projection);
   }
}
