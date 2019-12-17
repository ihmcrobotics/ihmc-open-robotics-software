package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.matrixlib.MatrixTools;

/**
 * This solver is a solver fo the continuous Lyapunov equation, which is defined as
 *
 * <p>A' X + X A + Q = 0,</p>
 * <p>where X is an unknown quantity, Q is a Hermitian matrix, and A' is the conjugate transpose of A.</p>
 * <p>Note that this implementation assumes only real values, making Q symmetric and A' a simple transpose.</p>
 * <p>This solver implements the Kronecker product notation, as found in https://en.wikipedia.org/wiki/Lyapunov_equation</p>
 *<p>The matrix equation of this problem is </p>
 * <p> I<sub>n</sub>&otimes; A + A'&otimes;I<sub>n</sub> vec X = -vec Q</p>
 * which is a simple linear system.
 */
public class LyapunovEquationSolver
      {
   private final DenseMatrix64F A = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F aTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Q = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F qVector = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F xVector = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F X = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F X1 = new  DenseMatrix64F(0, 0);
   private final DenseMatrix64F eyeX = new DenseMatrix64F(0, 0);

   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.lu(0);

   private int n;

   private boolean isUpToDate = false;

   public void setMatrices(DenseMatrix64F A, DenseMatrix64F Q)
   {
      isUpToDate = false;

      MatrixChecking.assertIsSquare(A);
      MatrixChecking.assertIsSquare(Q);

      this.A.set(A);
      this.Q.set(Q);

      n = this.A.getNumRows();
   }

   public DenseMatrix64F solve()
   {
      aTranspose.reshape(n, n);
      eyeX.reshape(n, n);
      X1.reshape(n * n, n * n);
      tempMatrix.reshape(n * n, n * n);

      CommonOps.transpose(A, aTranspose);
      CommonOps.setIdentity(eyeX);
      CommonOps.kron(aTranspose, eyeX, X1);
      CommonOps.kron(eyeX, aTranspose, tempMatrix);
      CommonOps.addEquals(X1, tempMatrix);

      // stack all of Q into a vector
      stack(Q, qVector);
      CommonOps.scale(-1.0, qVector);

      xVector.reshape(X1.getNumRows(), 1);
      solver.setA(X1);
      solver.solve(qVector, xVector);

      toSquareMatrix(xVector, X);

      isUpToDate = true;
      return X;
   }

   public DenseMatrix64F getX()
   {
      return isUpToDate ? X : solve();
   }

   private static void stack(DenseMatrix64F QMatrix, DenseMatrix64F qVector)
   {
      int rows = QMatrix.getNumRows();
      qVector.reshape(rows * rows, 1);
      for (int i = 0; i < rows; i++)
         MatrixTools.setMatrixBlock(qVector, i * rows, 0, QMatrix, 0, i, rows, 1, 1.0);
   }

   private static void toSquareMatrix(DenseMatrix64F xVector, DenseMatrix64F xMatrix)
   {
      int rows = (int) Math.sqrt(xVector.getNumRows());
      xMatrix.reshape(rows, rows);
      for (int i = 0; i < rows; i++)
         MatrixTools.setMatrixBlock(xMatrix, 0, i, xVector, i * rows, 0, rows, 1, 1.0);
   }
}
