package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
import us.ihmc.matrixlib.NativeCommonOps;

/**
 * This solver computes the solution to the algebraic Riccati equation
 *
 * <p>
 * A' P + P A - P B R^-1 B' P + Q = 0
 * </p>
 * <p> which can also be written as</p>
 * <p>A' P + P A - P M P + Q = 0</p>*
 * <p>where P is the unknown to be solved for, R is symmetric positive definite, Q is symmetric positive semi-definite, A is the state transition matrix,
 * and B is the control matrix.</p>
 */
public interface CARESolver
{
   /**
    * Setter of the solver. A and B should be compatible. B and R must be
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
    * Setter of the solver. A and B should be compatible. B and R must be
    * multiplicative compatible. A and Q must be multiplicative compatible. R
    * must be invertible.
    *
    * @param A state transition matrix
    * @param B control multipliers matrix
    * @param Q state cost matrix
    * @param R control cost matrix
    */
   void setMatrices(DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F Q, DenseMatrix64F R, boolean checkMatrices);

   /**
    * Computes and returns the P matrix.
    */
   DenseMatrix64F computeP();

   /**
    * Returns the P matrix.
    */
   DenseMatrix64F getP();
}
