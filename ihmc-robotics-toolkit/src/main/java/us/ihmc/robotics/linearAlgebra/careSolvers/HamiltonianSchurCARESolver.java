package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.data.Matrix;
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
public class HamiltonianSchurCARESolver implements CARESolver
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

      Rinv.reshape(n, n);
      NativeCommonOps.invert(R, Rinv);
      M.reshape(n, n);
      NativeCommonOps.multQuad(BTranspose, Rinv, M);

      this.n = A.getNumRows();

      this.A.set(A);
      this.Q.set(Q);
      this.M.set(M);
   }

   LinearSolver<DenseMatrix64F> lu = LinearSolverFactory.lu(3);
   /** {@inheritDoc} */
   public DenseMatrix64F computeP()
   {
      // defining Hamiltonian
      assembleHamiltonian(H);

      boolean converged = false;
      int iterations = 0;

      DenseMatrix64F MLocal = new DenseMatrix64F(H);
      DenseMatrix64F MDiff= new DenseMatrix64F(H);
      DenseMatrix64F Mnew = new DenseMatrix64F(H);
      DenseMatrix64F MLocalInverse = new DenseMatrix64F(H);

      while (!converged)
      {
         if (iterations > maxIterations)
            return null;

         MLocalInverse.set(MLocal);
         NativeCommonOps.invert(MLocal, MLocalInverse);

         CommonOps.subtract(MLocal, MLocalInverse, MDiff);
         Mnew.set(MDiff);
         CommonOps.scale(-0.5, Mnew);
         CommonOps.addEquals(Mnew, MLocal);

         converged = MatrixToolsLocal.distance(Mnew, MLocal) < epsilon;

         MLocal.set(Mnew);
         iterations++;
      }

      DenseMatrix64F U = new DenseMatrix64F(2 * n, n);
      DenseMatrix64F V = new DenseMatrix64F(2 * n, n);

      DenseMatrix64F M11 = new DenseMatrix64F(n, n);
      DenseMatrix64F M12 = new DenseMatrix64F(n, n);
      DenseMatrix64F M21 = new DenseMatrix64F(n, n);
      DenseMatrix64F M22 = new DenseMatrix64F(n, n);

      MatrixTools.setMatrixBlock(M11, 0, 0, MLocal, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(M12, 0, 0, MLocal, 0, n, n, n, 1.0);
      MatrixTools.setMatrixBlock(M21, 0, 0, MLocal, n, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(M22, 0, 0, MLocal, n, n, n, n, 1.0);

      CommonOps.addEquals(M22, CommonOps.identity(n));
      MatrixTools.setMatrixBlock(U, 0, 0, M12, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(U, n, 0, M22, 0, 0, n, n, 1.0);

      CommonOps.addEquals(M11, CommonOps.identity(n));
      MatrixTools.setMatrixBlock(V, 0, 0, M11, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(V, n, 0, M21, 0, 0, n, n, 1.0);


      P.reshape(n, n);
      CommonOps.scale(-1.0, V);
      lu.setA(U);
      lu.solve(V, P);

      isUpToDate = true;

      return P;
   }

   private static final int maxIterations = Integer.MAX_VALUE;
   private static final double epsilon = 1e-12;


   /** {inheritDoc} */
   public DenseMatrix64F getP()
   {
      return isUpToDate ? P : computeP();
   }

   private void assembleHamiltonian(DenseMatrix64F hamiltonianToPack)
   {
      ATranspose.reshape(n, n);
      CommonOps.transpose(A, ATranspose);

      hamiltonianToPack.reshape(2 * n, 2 * n);

      MatrixTools.setMatrixBlock(hamiltonianToPack, 0, 0, A, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(hamiltonianToPack, n, 0, Q, 0, 0, n, n, -1.0);
      MatrixTools.setMatrixBlock(hamiltonianToPack, 0, n, M, 0, 0, n, n, -1.0);
      MatrixTools.setMatrixBlock(hamiltonianToPack, n, n, ATranspose, 0, 0, n, n, -1.0);
   }


}
