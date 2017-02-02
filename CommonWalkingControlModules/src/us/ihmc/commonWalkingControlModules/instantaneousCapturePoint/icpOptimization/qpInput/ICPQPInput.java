package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput;

import org.ejml.data.DenseMatrix64F;

public class ICPQPInput
{
   public DenseMatrix64F quadraticTerm;
   public DenseMatrix64F linearTerm;
   public DenseMatrix64F residualCost = new DenseMatrix64F(1, 1);

   public ICPQPInput(int size)
   {
      quadraticTerm = new DenseMatrix64F(size, size);
      linearTerm = new DenseMatrix64F(size, 1);
      residualCost = new DenseMatrix64F(1, 1);
   }

   public void reshape(int size)
   {
      quadraticTerm.reshape(size, size);
      linearTerm.reshape(size, 1);
   }

   public void reset()
   {
      quadraticTerm.zero();
      linearTerm.zero();
      residualCost.zero();
   }
}
