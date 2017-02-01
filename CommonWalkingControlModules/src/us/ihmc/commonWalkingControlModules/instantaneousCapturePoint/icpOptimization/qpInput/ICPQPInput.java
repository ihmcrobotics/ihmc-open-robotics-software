package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput;

import org.ejml.data.DenseMatrix64F;

public abstract class ICPQPInput
{
   public DenseMatrix64F quadraticTerm;
   public DenseMatrix64F linearTerm;
   public DenseMatrix64F residualCost = new DenseMatrix64F(1, 1);

   public void reset()
   {
      quadraticTerm.zero();
      linearTerm.zero();
      residualCost.zero();
   }
}
