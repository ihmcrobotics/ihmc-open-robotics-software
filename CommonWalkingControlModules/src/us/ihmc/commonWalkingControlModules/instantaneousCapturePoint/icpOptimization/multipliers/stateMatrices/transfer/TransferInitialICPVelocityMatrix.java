package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.transfer;

import org.ejml.data.DenseMatrix64F;

public class TransferInitialICPVelocityMatrix extends DenseMatrix64F
{
   public TransferInitialICPVelocityMatrix()
   {
      super(4, 1);
   }

   public void reset()
   {
      zero();
   }

   public void compute()
   {
      set(1, 0, 1);
   }
}
