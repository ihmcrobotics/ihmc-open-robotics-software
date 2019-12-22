package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.ejml.ops.EjmlUnitTests;
import us.ihmc.commons.MathTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeCommonOps;

/**
 * This solver computes the solution to the algebraic Riccati equation
 *
 * <p>
 * A' P + P A - P B R^-1 B' P + Q = 0
 * </p>
 * or
 * <p>
 *  * A' P + P A - P S P + Q = 0
 *  * </p>
 * <p> which can also be written as</p>
 * <p>A' P + P A - P S P + Q = 0</p>*
 * <p>where P is the unknown to be solved for, R is symmetric positive definite, Q is symmetric positive semi-definite, A is the state transition matrix,
 * and B is the control matrix.</p>
 *
 * <p>
 *    The solution is found by performing the Matrix Sign Function solution of the Hamiltonian, as outlined in
 *    https://stanford.edu/class/ee363/lectures/clqr.pdf.
 * </p>
 */
public class MatrixSignFunctionCARESolver implements CARESolver
{
   /** The solution of the algebraic Riccati equation. */
   private final DenseMatrix64F P = new DenseMatrix64F(0, 0);

   private int maxIterations = Integer.MAX_VALUE;
   private double epsilon = 1e-12;

   private final SingularValueDecomposition<DenseMatrix64F> svd = DecompositionFactory.svd(0, 0, false, false, false);
   private final LinearSolver<DenseMatrix64F> lu = LinearSolverFactory.lu(3);

   private final DenseMatrix64F BTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F ATranspose = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F A = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Q = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Rinv = new DenseMatrix64F(0, 0);

   /** Hamiltonian of the Riccati equation. */
   private final DenseMatrix64F H = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F S = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F W11 = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F W21 = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F W12 = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F W22 = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F I = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F M = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F N = new DenseMatrix64F(0, 0);

   private boolean isUpToDate = false;
   private int n;

   public void setMaxIterations(int maxIterations)
   {
      this.maxIterations = maxIterations;
   }

   public void setConvergenceEpsilon(double epsilon)
   {
      this.epsilon = epsilon;
   }

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

      CARETools.computeS(BTranspose, R, Rinv, S);

      this.n = A.getNumRows();

      this.A.set(A);
      this.Q.set(Q);
   }


   private final DenseMatrix64F Wprev = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F W = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F WInverse = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F WDiff = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F WAlt = new DenseMatrix64F(0, 0);

   /** {@inheritDoc} */
   public DenseMatrix64F computeP()
   {
      // defining Hamiltonian
      CARETools.assembleHamiltonian(A, ATranspose, Q, S, H);

      boolean converged = false;
      int iterations = 0;

      Wprev.set(H);
      W.reshape(2 * n, 2 * n);
      WDiff.reshape(2 * n, 2 * n);
      WInverse.reshape(2 * n, 2 * n);
      WAlt.reshape(2 * n, 2 * n);

      while (!converged)
      {
         if (iterations > maxIterations)
            return null;

         NativeCommonOps.invert(Wprev, WInverse);

         CommonOps.subtract(Wprev, WInverse, WDiff);
         CommonOps.add(Wprev, -0.5, WDiff, W);

         converged = MatrixToolsLocal.distance(W, Wprev) < epsilon;

         Wprev.set(W);
         iterations++;
      }

      M.reshape(2 * n, n);
      N.reshape(2 * n, n);

      W11.reshape(n, n);
      W12.reshape(n, n);
      W21.reshape(n, n);
      W22.reshape(n, n);

      MatrixTools.setMatrixBlock(W11, 0, 0, W, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(W12, 0, 0, W, 0, n, n, n, 1.0);
      MatrixTools.setMatrixBlock(W21, 0, 0, W, n, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(W22, 0, 0, W, n, n, n, n, 1.0);

      I.reshape(n, n);
      CommonOps.setIdentity(I);
      CommonOps.addEquals(W22, I);
      MatrixTools.setMatrixBlock(M, 0, 0, W12, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(M, n, 0, W22, 0, 0, n, n, 1.0);

      CommonOps.addEquals(W11, I);
      MatrixTools.setMatrixBlock(N, 0, 0, W11, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(N, n, 0, W21, 0, 0, n, n, 1.0);




      P.reshape(n, n);
      CommonOps.scale(-1.0, N);
      lu.setA(M);
      lu.solve(N, P);

      isUpToDate = true;

      DenseMatrix64F PDot = new DenseMatrix64F(n, n);
      if (!CARESolver.riccatiEquationIsZero(A, S, Q, P, PDot, 1e-7))
         throw new RuntimeException("Bad stuff.");

      return P;
   }

   /** {inheritDoc} */
   public DenseMatrix64F getP()
   {
      return isUpToDate ? P : computeP();
   }
}
