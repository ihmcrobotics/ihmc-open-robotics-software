package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.transfer;

import org.ejml.data.DenseMatrix64F;

public class NewTransferInitialICPVelocityMatrix extends DenseMatrix64F
{
   public NewTransferInitialICPVelocityMatrix()
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
