package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.EigenDecomposition;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
import us.ihmc.matrixlib.MatrixTools;
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
 *
 * <p>
 *    The solution is found by computing the Hamiltonian and performing an ordered eigen value decomposition, as outlined in
 *    https://en.wikipedia.org/wiki/Algebraic_Riccati_equation and https://stanford.edu/class/ee363/lectures/clqr.pdf. This assumes that the Hamiltonian
 *    has only real eigenvalues, with no complex conjugate pairs.
 * </p>
 */
public class HamiltonianEigenCARESolver implements CARESolver
{
   /** The solution of the algebraic Riccati equation. */
   private final DenseMatrix64F P = new DenseMatrix64F(0, 0);

   private final SingularValueDecomposition<DenseMatrix64F> svd = DecompositionFactory.svd(0, 0, false, false, false);
   private final EigenDecomposition<DenseMatrix64F> eigen = DecompositionFactory.eig(0, true);

   private final DenseMatrix64F BTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F ATranspose = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F A = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Q = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Rinv = new DenseMatrix64F(0, 0);

   /** Hamiltonian of the Riccati equation. */
   private final DenseMatrix64F H = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F M = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F u = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F u1 = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F u2 = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F u1Inv = new DenseMatrix64F(0, 0);

   private boolean isUpToDate = false;
   private int n;

   /** {@inheritDoc} */
   public void setMatrices(DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F Q, DenseMatrix64F R, boolean checkMatrices)
   {
      isUpToDate = false;

      // checking A
      MatrixChecking.assertIsSquare(A);
      MatrixChecking.assertMultiplicationCompatible(A, B);
      MatrixChecking.assertMultiplicationCompatible(B, R);
      MatrixChecking.assertMultiplicationCompatible(A, Q);

      if (checkMatrices)
      {
         // checking R
         svd.decompose(R);
         if (MathTools.min(svd.getSingularValues()) == 0.0)
            throw new IllegalArgumentException("R Matrix is singular.");
      }

      this.n = A.getNumRows();

      BTranspose.set(B);
      CommonOps.transpose(BTranspose);
      ATranspose.reshape(n, n);
      CommonOps.transpose(A, ATranspose);

      CARETools.computeM(BTranspose, R, Rinv, M);

      this.n = A.getNumRows();

      this.A.set(A);
      this.Q.set(Q);
      this.M.set(M);
   }

   /** {@inheritDoc} */
   public DenseMatrix64F computeP()
   {
      // defining Hamiltonian
      CARETools.assembleHamiltonian(A, ATranspose, Q, M, H);

      // Eigen decomposition
      eigen.decompose(H);
      u.reshape(2 * n, n);
      u1.reshape(n, n);
      u2.reshape(n, n);
      int valueIdx = 0;
      for (int i = 0; i < 2 * n; i++)
      {
         if (eigen.getEigenvalue(i).getImaginary() != 0.0)
            throw new IllegalArgumentException("The imaginary part of the eigenvalue must be equal to zero.");

         if (eigen.getEigenvalue(i).getReal() < 0.0)
         {
            MatrixTools.setMatrixBlock(u, 0, valueIdx++, eigen.getEigenVector(i), 0, 0, 2 * n, 1, 1.0);
         }

         if (valueIdx == n)
            break;
      }

      // solving linear system
      // P = U_2 * U_1^-1
      MatrixTools.setMatrixBlock(u1, 0, 0, u, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(u2, 0, 0, u, n, 0, n, n, 1.0);

      u1Inv.reshape(n, n);
      NativeCommonOps.invert(u1, u1Inv);

      P.reshape(n, n);
      CommonOps.mult(u2, u1Inv, P);

      isUpToDate = true;

      return P;
   }

   /** {inheritDoc} */
   public DenseMatrix64F getP()
   {
      return isUpToDate ? P : computeP();
   }
}
