package us.ihmc.commonWalkingControlModules.capturePoint.optimization.recursiveController.multipliers;

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
      // this is the initial spline rate
      set(1, 0, 1);
   }
}
