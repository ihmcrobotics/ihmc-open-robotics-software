package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.transfer;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class TransferEntryCMPMatrix extends DenseMatrix64F
{
   private final DoubleYoVariable defaultDoubleSupportSplitRatio;

   public TransferEntryCMPMatrix(DoubleYoVariable defaultDoubleSupportSplitRatio)
   {
      super(4, 1);

      this.defaultDoubleSupportSplitRatio = defaultDoubleSupportSplitRatio;
   }

   public void reset()
   {
      zero();
   }

   public void compute(ArrayList<DoubleYoVariable> doubleSupportDurations, double omega0)
   {
      this.compute(doubleSupportDurations.get(0).getDoubleValue(), omega0);
   }

   public void compute(double doubleSupportDuration, double omega0)
   {
      zero();

      double endOfDoubleSupportDuration = (1.0 - defaultDoubleSupportSplitRatio.getDoubleValue()) * doubleSupportDuration;

      double endOfDoubleSupportProjection = Math.exp(omega0 * endOfDoubleSupportDuration);

      double stepProjection = (1.0 - endOfDoubleSupportProjection);

      set(2, 0, stepProjection);
      set(3, 0, -omega0 * endOfDoubleSupportProjection);
   }
}
