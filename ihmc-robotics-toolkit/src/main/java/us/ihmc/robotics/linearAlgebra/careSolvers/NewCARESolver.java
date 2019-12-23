package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;

/**
 * This solver computes the solution to the algebraic Riccati equation
 *
 * <p>
 * A' P + P A - P B R^-1 B' P + Q = 0
 * </p>
 * <p>
 * or
 * </p>
 * <p>
 *  A' P + P A - P M P + Q = 0
 *  </p>
 * Or the more complex one
 * A' P E + E' P A - (B' P E + S' C)' R^-1 (B' P E + S' C) + C' Q C = 0
 * <p>
 * <p>
 *    or
 * </p>
 * </p>
 * A*' P E + E' P A* - E' P M P E + Q* = 0
 * where
 * A* = A - B R^-1 S' C
 * Q* = C' Q C - C' S R^-1 S' C
 * <p>
 * where
 * @param P is the unknown to be solved for
 * @param R is the symmetric positive definite cost on the control.
 * @param Q is the symmetric positive semi-definite cost on the output.
 * @param S is the cross-cost on the control and state
 * @param E is the output matrix
 * @param A is the state transition matrix
 * @param B is the control matrix.
 * @param C is the output matrix,
 *
 * I.e., we're solving the system
 *  J = y' Q y + u' R u' + u' S' y + y' S u
 *  E x = A x + B u
 *  y = C x
 *  </p>
 *          <p>
 *          It can be seen that, in the simpler system above, S = 0, E = C = I
 *          </p>
 *
 *          <p>
 *          We can also
 *          </p>
 */
public interface NewCARESolver
{
   void setMatrices(DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F C, DenseMatrix64F E, DenseMatrix64F Q, DenseMatrix64F R, DenseMatrix64F S);

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
   void setMatrices(DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F E, DenseMatrix64F Q, DenseMatrix64F R);

   void setMatrices(DenseMatrix64F A, DenseMatrix64F E, DenseMatrix64F M, DenseMatrix64F Q);

   /**
    * Computes and returns the P matrix.
    */
   DenseMatrix64F computeP();

   /**
    * Returns the P matrix.
    */
   DenseMatrix64F getP();
}
