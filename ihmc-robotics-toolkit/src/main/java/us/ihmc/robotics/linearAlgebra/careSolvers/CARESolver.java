package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;

public interface CARESolver
{
   /**
    * Constructor of the solver. A and B should be compatible. B and R must be
    * multiplicative compatible. A and Q must be multiplicative compatible. R
    * must be invertible.
    *
    * @param A state transition matrix
    * @param B control multipliers matrix
    * @param Q state cost matrix
    * @param R control cost matrix
    */
   default void setMatrices(DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F Q, DenseMatrix64F R)
   {
      setMatrices(A, B, Q, R, true);
   }

   /**
    * Constructor of the solver. A and B should be compatible. B and R must be
    * multiplicative compatible. A and Q must be multiplicative compatible. R
    * must be invertible.
    *
    * @param A state transition matrix
    * @param B control multipliers matrix
    * @param Q state cost matrix
    * @param R control cost matrix
    */
   void setMatrices(DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F Q, DenseMatrix64F R, boolean checkMatrices);

   void computeP();

   DenseMatrix64F getP();
}
