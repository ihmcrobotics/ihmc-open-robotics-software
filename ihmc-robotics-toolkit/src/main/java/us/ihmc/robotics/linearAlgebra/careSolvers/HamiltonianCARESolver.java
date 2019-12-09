/**
 * This is a rework of the hipparchus project solver found in
 * https://github.com/Hipparchus-Math/hipparchus/blob/master/hipparchus-core/src/main/java/org/hipparchus/linear/RiccatiEquationSolverImpl.java
 */
package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.decomposition.EigenDecomposition;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeCommonOps;

/**
 * This solver computes the solution using the following approach:
 *
 * 1. Compute the Hamiltonian matrix 2. Extract its complex eigen vectors (not
 * the best solution, a better solution would be ordered Schur transformation)
 */

/**
 * Compute P using the Hamiltonian and the ordered eigen values decomposition, as shown here:
 *
 * https://en.wikipedia.org/wiki/Algebraic_Riccati_equation
 *
 * @param A state transition matrix
 * @param B control multipliers matrix
 * @param Q state cost matrix
 * @param Rinv inverse of matrix R
 * @return initial solution
 */
public class HamiltonianCARESolver implements CARESolver
{
   /** The solution of the algebraic Riccati equation. */
   private final DenseMatrix64F P = new DenseMatrix64F(0, 0);

   /** The computed K. */
   private final DenseMatrix64F K = new DenseMatrix64F(0, 0);

   private final SingularValueDecomposition<DenseMatrix64F> svd = DecompositionFactory.svd(0, 0, false, false, false);
   private final EigenDecomposition<DenseMatrix64F> eigen = DecompositionFactory.eig(0, true);

   private final DenseMatrix64F BTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F ATranspose = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F A = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F B = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Q = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F R = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Rinv = new DenseMatrix64F(0, 0);

   /** Hamiltonian of the Riccati equation. */
   private final DenseMatrix64F H = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F M = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F u = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F u1 = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F u2 = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F u1Inv = new DenseMatrix64F(0, 0);

   private int m;
   private int n;

   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(0, 0);

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
   public void setMatrices(DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F Q, DenseMatrix64F R, boolean checkMatrices)
   {
      if (checkMatrices)
      {
         // checking A
         MatrixChecking.assertIsSquare(A);
         MatrixChecking.assertMultiplicationCompatible(A, B);
         MatrixChecking.assertMultiplicationCompatible(B, R);
         MatrixChecking.assertMultiplicationCompatible(A, Q);

         // checking R
         svd.decompose(R);
         if (MathTools.min(svd.getSingularValues()) == 0.0)
            throw new IllegalArgumentException("R Matrix is singular.");
      }

      this.A.set(A);
      this.B.set(B);
      this.Q.set(Q);
      this.R.set(R);
   }

   /** {@inheritDoc} */
   public void computeP()
   {
      this.n = A.getNumRows();
      this.m = B.getNumCols();
      this.Rinv.reshape(n, n);

      NativeCommonOps.invert(R, Rinv);

      ATranspose.reshape(n, n);
      BTranspose.reshape(m, n);
      CommonOps.transpose(A, ATranspose);
      CommonOps.transpose(B, BTranspose);

      // computing the Hamiltonian Matrix
      M.reshape(n, n);
      NativeCommonOps.multQuad(BTranspose, Rinv, M);

      // defining Hamiltonian
      H.reshape(2 * n, 2 * n);

      MatrixTools.setMatrixBlock(H, 0, 0, A, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(H, n, 0, Q, 0, 0, n, n, -1.0);
      MatrixTools.setMatrixBlock(H, 0, n, M, 0, 0, n, n, -1.0);
      MatrixTools.setMatrixBlock(H, n, n, ATranspose, 0, 0, n, n, -1.0);

      // eigen decomposition
      // it must be ordered in order to work with submatrices
      eigen.decompose(H);
      u.reshape(2 * n, n);
      u1.reshape(n, n);
      u2.reshape(n, n);
      int valueIdx = 0;

      for (int i = 0; i < 2 * n; i++)
      {
         if (eigen.getEigenvalue(i).getReal() < 0.0)
         {
            MatrixTools.setMatrixBlock(u, 0, valueIdx, eigen.getEigenVector(i), 0, 0, 2 * n, 1, 1.0);
            valueIdx++;
         }
      }

      // solving linear system
      // P = U_2 * U_1^-1
      MatrixTools.setMatrixBlock(u1, 0, 0, u, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(u2, 0, 0, u, n, 0, n, n, 1.0);

      u1Inv.reshape(n, n);
      NativeCommonOps.invert(u1, u1Inv);

      P.reshape(n, n);
      CommonOps.mult(u2, u1Inv, P);

      // K = R^-1 B^T P
      K.reshape(m, n);
      tempMatrix.reshape(m, n);
      CommonOps.mult(BTranspose, P, tempMatrix);
      CommonOps.mult(Rinv, tempMatrix, K);
   }

   /** {inheritDoc} */
   public DenseMatrix64F getP()
   {
      return P;
   }

   /** {inheritDoc} */
   public DenseMatrix64F getK()
   {
      return K;
   }

}
