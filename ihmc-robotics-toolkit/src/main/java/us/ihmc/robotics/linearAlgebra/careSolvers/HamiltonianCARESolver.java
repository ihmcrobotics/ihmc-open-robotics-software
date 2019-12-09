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
 * This solver computes the solution to the algebraic Riccati equation
 *
 * A' P + P A - P B R^-1 B' P + Q = 0
 * or
 * A' P + P A - P M P + Q = 0
 *
 * where P is unknown.
 *
 * The solution using the following approach:
 *
 * Compute P using the Hamiltonian and the ordered eigen values decomposition, as shown here:
 *
 * https://en.wikipedia.org/wiki/Algebraic_Riccati_equation
 *
 * A good set of notes to follow are
 *
 * https://stanford.edu/class/ee363/lectures/clqr.pdf
 */
public class HamiltonianCARESolver implements CARESolver
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

   private int n;

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

      this.n = A.getNumRows();

      BTranspose.set(B);
      CommonOps.transpose(BTranspose);

      Rinv.reshape(n, n);
      NativeCommonOps.invert(R, Rinv);
      M.reshape(n, n);
      NativeCommonOps.multQuad(BTranspose, Rinv, M);

      setMatrices(A, Q, M, false);
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
   public void setMatrices(DenseMatrix64F A, DenseMatrix64F Q, DenseMatrix64F M, boolean checkMatrices)
   {
      if (checkMatrices)
      {
         // checking A
         MatrixChecking.assertIsSquare(A);
         MatrixChecking.assertMultiplicationCompatible(A, Q);

      }

      this.n = A.getNumRows();

      this.A.set(A);
      this.Q.set(Q);
      this.M.set(M);
   }


   /** {@inheritDoc} */
   public void computeP()
   {
      // defining Hamiltonian
      assembleHamiltonian(H);

      // eigen decomposition
      // it must be ordered in order to work with submatrices
      eigen.decompose(H);
      u.reshape(2 * n, n);
      u1.reshape(n, n);
      u2.reshape(n, n);
      int valueIdx = 0;
      for (int i = 0; i < 2 * n; i++)
      {
         if (eigen.getEigenvalue(i).getImaginary() != 0.0)
            throw new RuntimeException("The imaginary part of the eigenvalue must be equal to zero.");

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
   }

   void assembleHamiltonian(DenseMatrix64F hamiltonianToPack)
   {
      ATranspose.reshape(n, n);
      CommonOps.transpose(A, ATranspose);

      hamiltonianToPack.reshape(2 * n, 2 * n);

      MatrixTools.setMatrixBlock(hamiltonianToPack, 0, 0, A, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(hamiltonianToPack, n, 0, Q, 0, 0, n, n, -1.0);
      MatrixTools.setMatrixBlock(hamiltonianToPack, 0, n, M, 0, 0, n, n, -1.0);
      MatrixTools.setMatrixBlock(hamiltonianToPack, n, n, ATranspose, 0, 0, n, n, -1.0);
   }

   /** {inheritDoc} */
   public DenseMatrix64F getP()
   {
      return P;
   }


}
