/**
 * This is a rework of the hipparchus project solver found in
 * https://github.com/Hipparchus-Math/hipparchus/blob/master/hipparchus-core/src/main/java/org/hipparchus/linear/RiccatiEquationSolverImpl.java
 */
package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.factory.LinearSolverFactory;
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
 * 3. Approximate the initial solution given by 2 using the Kleinman algorithm
 * (an iterative method)
 */
public class KleinmanCARESolver implements CARESolver
{
   /** Internally used maximum iterations. */
   private static final int MAX_ITERATIONS = 100;

   /** Internally used epsilon criteria. */
   private static final double EPSILON = 1e-8;

   /** The solution of the algebraic Riccati equation. */
   private DenseMatrix64F P;

   /** The computed K. */
   private DenseMatrix64F K;

   private final HamiltonianCARESolver hamiltonianCARESolver = new HamiltonianCARESolver();
   private final SingularValueDecomposition<DenseMatrix64F> svd = DecompositionFactory.svd(0, 0, false, false, false);


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
   public void setMatrices(DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F Q, DenseMatrix64F R)
   {
      // checking A
      if (!MatrixChecking.isSquare(A))
         throw new IllegalArgumentException("A is not square : " + A.getNumRows() + " x " + A.getNumCols());
      if (A.getNumCols() != B.getNumRows()) {
         throw new IllegalArgumentException("Dimensions do not match : " + A.getNumRows() + ", " + B.getNumCols());
      }
      MatrixChecking.assertMultiplicationCompatible(B, R);
      MatrixChecking.assertMultiplicationCompatible(A, Q);

      // checking R
      svd.decompose(R);
      if (MathTools.min(svd.getSingularValues()) == 0.0)
         throw new IllegalArgumentException("R Matrix is singular.");

      LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.general(R.getNumRows(), R.getNumCols());
      solver.setA(R);
      DenseMatrix64F R_inv = new DenseMatrix64F(R.getNumRows(), R.getNumCols());
      solver.invert(R_inv);

      hamiltonianCARESolver.setMatrices(A, B, Q, R);

      P = approximateP(A, B, Q, R, R_inv, hamiltonianCARESolver.getP(), MAX_ITERATIONS, EPSILON);

      // K = R^-1 B^T P
      K = new DenseMatrix64F(R_inv.getNumRows(), P.getNumCols());
      DenseMatrix64F tempMatrix = new DenseMatrix64F(0, 0);
      tempMatrix.reshape(B.getNumCols(), P.getNumCols());
      CommonOps.multTransA(B, P, tempMatrix);
      CommonOps.mult(R_inv, tempMatrix, K);
   }

   public void computeP()
   {}


   /**
    * Applies the Kleinman's algorithm.
    *
    * @param A state transition matrix
    * @param B control multipliers matrix
    * @param Q state cost matrix
    * @param R control cost matrix
    * @param R_inv inverse of matrix R
    * @param initialP initial solution
    * @param maxIterations maximum number of iterations allowed
    * @param epsilon convergence threshold
    * @return improved solution
    */
   private DenseMatrix64F approximateP(final DenseMatrix64F A, final DenseMatrix64F B, final DenseMatrix64F Q, final DenseMatrix64F R,
                                       final DenseMatrix64F R_inv, final DenseMatrix64F initialP, final int maxIterations, final double epsilon)
   {
      DenseMatrix64F K = new DenseMatrix64F(R_inv.getNumRows(), initialP.getNumCols());
      DenseMatrix64F P = new DenseMatrix64F(initialP);

      double error = 1;
      int i = 1;
      while (error > epsilon)
      {
         // K_i = -R_inv B' P_i-1
         DenseMatrix64F tempMatrix = new DenseMatrix64F(0, 0);
         tempMatrix.reshape(B.getNumCols(), P.getNumCols());
         CommonOps.multTransA(B, P, tempMatrix);
         CommonOps.mult(-1.0, R_inv, tempMatrix, K);

         // X = A+B*K_i;
         DenseMatrix64F X = new DenseMatrix64F(A.getNumRows(), A.getNumCols());
         CommonOps.mult(B, K, X);
         CommonOps.addEquals(X, A);

         // Y = -K_i*R*K_i' - Q;
         DenseMatrix64F kTranspose = new DenseMatrix64F(K.getNumCols(), K.getNumRows());
         DenseMatrix64F Y = new DenseMatrix64F(Q.getNumRows(), Q.getNumCols());
         CommonOps.transpose(K, kTranspose);
         NativeCommonOps.multQuad(K, R, Y);
         CommonOps.addEquals(Y, Q);
         CommonOps.scale(-1.0, Q);

         DenseMatrix64F xTranspose = new DenseMatrix64F(X.getNumCols(), X.getNumRows());
         CommonOps.transpose(X, xTranspose);
         DenseMatrix64F eyeX = CommonOps.identity(X.getNumRows());
         DenseMatrix64F X1 = new DenseMatrix64F(xTranspose.getNumRows() * eyeX.getNumRows(), xTranspose.getNumCols() * eyeX.getNumCols());
         tempMatrix.reshape(xTranspose.getNumRows() * eyeX.getNumRows(), xTranspose.getNumCols() * eyeX.getNumCols());
         // X1=kron(X',eye(size(X))) + kron(eye(size(X)),X');
         CommonOps.kron(xTranspose, eyeX, X1);
         CommonOps.kron(eyeX, xTranspose, tempMatrix);
         CommonOps.addEquals(X1, tempMatrix);

         // TODO see if we have to do this stack and unstack
         // stack all of Y into a vector
         DenseMatrix64F yVector = stack(Y);

         // PX = inv(X1)*Y1;
         // sensitive to numerical errors
         // final RealMatrix PX = MatrixUtils.inverse(X__).multiply(Y__);
         DenseMatrix64F PX = new DenseMatrix64F(X1.getNumRows(), 1);
         LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.lu(X1.getNumRows());
         solver.setA(X1);
         solver.solve(yVector, PX);

         // P = reshape(PX,sqrt(length(PX)),sqrt(length(PX))); %%unstack
         final DenseMatrix64F nextP = toSquareMatrix(PX);

         // aerror = norm(P - P1);
         DenseMatrix64F diff = new DenseMatrix64F(nextP);
         CommonOps.subtractEquals(diff, P);
         error = l2Norm(diff);

         P.set(nextP);
         i++;
         if (i > maxIterations) {
            throw new RuntimeException("Convergence failed.");
         }
      }

      return P;
   }

   private static DenseMatrix64F stack(DenseMatrix64F y)
   {
      int rows = y.getNumRows();
      DenseMatrix64F yVector = new DenseMatrix64F(rows * rows, 1);
      for (int i = 0; i < rows; i++)
         MatrixTools.setMatrixBlock(yVector, i * rows, 0, y, 0, i, rows, 1, 1.0);

      return yVector;
   }

   private static DenseMatrix64F toSquareMatrix(DenseMatrix64F yVector)
   {
      int rows = (int) Math.sqrt(yVector.getNumRows());
      DenseMatrix64F yMatrix = new DenseMatrix64F(rows, rows);
      for (int i = 0; i < rows; i++)
         MatrixTools.setMatrixBlock(yMatrix, 0, i, yVector, i * rows, 0, rows, 1, 1.0);

      return yMatrix;
   }

   private static double l2Norm(DenseMatrix64F matrix)
   {
      double norm = 0.0;
      for (int col = 0; col < matrix.getNumCols(); col++)
      {
         double rowSum = 0.0;
         for (int row = 0; row < matrix.getNumRows(); row++)
         {
            rowSum += MathTools.square(matrix.get(row, col));
         }
         norm += MathTools.square(rowSum);
      }

      return norm;
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
