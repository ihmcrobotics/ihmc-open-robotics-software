package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.linearAlgebra.careSolvers.matrixSignFunction.MatrixSignFunction;
import us.ihmc.robotics.linearAlgebra.careSolvers.matrixSignFunction.NewtonMatrixSignFunction;
import us.ihmc.robotics.linearAlgebra.careSolvers.matrixSignFunction.QuadraticMatrixSignFunction;

/**
 * <p>
 *    The solution is found by performing the Matrix Sign Function solution of the Hamiltonian, as outlined in
 *    https://www.sciencedirect.com/science/article/pii/0024379587902229
 * </p>
 */
public class MatrixSignFunctionCARESolver implements CARESolver
{
   private final DenseMatrix64F P = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F PAssym = new DenseMatrix64F(0, 0);

   private final SingularValueDecomposition<DenseMatrix64F> svd = DecompositionFactory.svd(0, 0, false, false, false);
   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.lu(3);
   private final MatrixSignFunction matrixSignFunction = new QuadraticMatrixSignFunction();

   private final DenseMatrix64F BTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F ATranspose = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F A = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Q = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Rinv = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F H = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F S = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F I = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F M = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F N = new DenseMatrix64F(0, 0);

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

      CARETools.computeS(BTranspose, R, Rinv, S);

      this.n = A.getNumRows();

      this.A.set(A);
      this.Q.set(Q);
   }

   private final DenseMatrix64F W = new DenseMatrix64F(0, 0);

   /** {@inheritDoc} */
   public DenseMatrix64F computeP()
   {
      // defining Hamiltonian
      H.reshape(2 * n, 2 * n);
      MatrixTools.setMatrixBlock(H, 0, 0, A, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(H, 0, n, S, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(H, n, 0, Q, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(H, n, n, ATranspose, 0, 0, n, n, -1.0);

      if (!matrixSignFunction.compute(H))
         throw new RuntimeException("Error.");

      W.reshape(2 * n, 2 * n);
      matrixSignFunction.getW(W);

      I.reshape(n, n);
      CommonOps.setIdentity(I);

      M.reshape(2 * n, n);
      N.reshape(2 * n, n);

      // M P = N
      MatrixTools.setMatrixBlock(M, 0, 0, W, 0, n, n, n, 1.0);
      MatrixTools.setMatrixBlock(M, n, 0, W, n, n, n, n, 1.0);
      MatrixTools.addMatrixBlock(M, n, 0, I, 0, 0, n, n, 1.0);

      MatrixTools.setMatrixBlock(N, 0, 0, W, 0, 0, n, n, 1.0);
      MatrixTools.addMatrixBlock(N, 0, 0, I, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(N, n, 0, W, n, 0, n, n, 1.0);

      P.reshape(n, n);
      PAssym.reshape(n, n);
      solver.setA(M);
      solver.solve(N, P);

      isUpToDate = true;

      return P;
   }

   /** {inheritDoc} */
   public DenseMatrix64F getP()
   {
      return isUpToDate ? P : computeP();
   }
}
