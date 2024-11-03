package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

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
   private final DMatrixRMaj A = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj aTranspose = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj Q = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj tempMatrix = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj qVector = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj xVector = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj X = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj X1 = new  DMatrixRMaj(0, 0);
   private final DMatrixRMaj eyeX = new DMatrixRMaj(0, 0);

   private final LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.lu(0);

   private int n;

   private boolean isUpToDate = false;

   public void setMatrices(DMatrixRMaj A, DMatrixRMaj Q)
   {
      isUpToDate = false;

      MatrixChecking.assertIsSquare(A);
      MatrixChecking.assertIsSquare(Q);

      this.A.set(A);
      this.Q.set(Q);

      n = this.A.getNumRows();
   }

   public DMatrixRMaj solve()
   {
      aTranspose.reshape(n, n);
      eyeX.reshape(n, n);
      X1.reshape(n * n, n * n);
      tempMatrix.reshape(n * n, n * n);

      CommonOps_DDRM.transpose(A, aTranspose);
      CommonOps_DDRM.setIdentity(eyeX);
      CommonOps_DDRM.kron(aTranspose, eyeX, X1);
      CommonOps_DDRM.kron(eyeX, aTranspose, tempMatrix);
      CommonOps_DDRM.addEquals(X1, tempMatrix);

      // stack all of Q into a vector
      stack(Q, qVector);
      CommonOps_DDRM.scale(-1.0, qVector);

      xVector.reshape(X1.getNumRows(), 1);
      solver.setA(X1);
      solver.solve(qVector, xVector);

      toSquareMatrix(xVector, X);

      isUpToDate = true;
      return X;
   }

   public DMatrixRMaj getX()
   {
      return isUpToDate ? X : solve();
   }

   private static void stack(DMatrixRMaj QMatrix, DMatrixRMaj qVector)
   {
      int rows = QMatrix.getNumRows();
      qVector.reshape(rows * rows, 1);
      for (int i = 0; i < rows; i++)
         MatrixTools.setMatrixBlock(qVector, i * rows, 0, QMatrix, 0, i, rows, 1, 1.0);
   }

   private static void toSquareMatrix(DMatrixRMaj xVector, DMatrixRMaj xMatrix)
   {
      int rows = (int) Math.sqrt(xVector.getNumRows());
      xMatrix.reshape(rows, rows);
      for (int i = 0; i < rows; i++)
         MatrixTools.setMatrixBlock(xMatrix, 0, i, xVector, i * rows, 0, rows, 1, 1.0);
   }
}
