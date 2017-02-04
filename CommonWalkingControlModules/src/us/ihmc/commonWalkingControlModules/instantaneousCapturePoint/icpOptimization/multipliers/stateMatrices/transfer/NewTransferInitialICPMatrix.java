package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.transfer;

import org.ejml.data.DenseMatrix64F;

public class NewTransferInitialICPMatrix extends DenseMatrix64F
{
   public NewTransferInitialICPMatrix()
   {
      super(4, 1);
   }

   public void reset()
   {
      zero();
   }

   public void compute()
   {
      set(0, 0, 1);
   }
}
