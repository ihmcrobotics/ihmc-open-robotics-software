package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.transfer;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class TransferEntryCMPMatrix extends DenseMatrix64F
{
   private final List<DoubleYoVariable> transferSplitFractions;

   public TransferEntryCMPMatrix(List<DoubleYoVariable> transferSplitFractions)
   {
      super(4, 1);

      this.transferSplitFractions = transferSplitFractions;
   }

   public void reset()
   {
      zero();
   }

   public void compute(List<DoubleYoVariable> doubleSupportDurations, double omega0)
   {
      zero();

      double endOfDoubleSupportDuration = (1.0 - transferSplitFractions.get(0).getDoubleValue()) * doubleSupportDurations.get(0).getDoubleValue();

      double projection = Math.exp(omega0 * endOfDoubleSupportDuration);

      set(2, 0, 1.0 - projection);
      set(3, 0, -omega0 * projection);
   }
}
