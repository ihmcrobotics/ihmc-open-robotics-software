/**
 * This is a rework of the hipparchus project solver found in
 * https://github.com/Hipparchus-Math/hipparchus/blob/master/hipparchus-core/src/main/java/org/hipparchus/linear/RiccatiEquationSolverImpl.java
 */
package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
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
   private int maxIterations = 100;

   /** Internally used epsilon criteria. */
   private double convergenceEpsilon = 1e-8;

   /** The solution of the algebraic Riccati equation. */
   private final DenseMatrix64F P = new DenseMatrix64F(0, 0);

   /** The computed K. */
   private final DenseMatrix64F K = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F A = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F B = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Q = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F R = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Rinv = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F Ak = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Qk = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(0, 0);

   private final LyapunovSolver lyapunovSolver = new LyapunovSolver();

   private final HamiltonianCARESolver hamiltonianCARESolver = new HamiltonianCARESolver();
   private final SingularValueDecomposition<DenseMatrix64F> svd = DecompositionFactory.svd(0, 0, false, false, false);

   private int n;
   private int m;

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

   public void computeP()
   {
      n = A.getNumRows();
      m = R.getNumRows();

      Rinv.reshape(m, m);
      NativeCommonOps.invert(R, Rinv);

      hamiltonianCARESolver.setMatrices(A, B, Q, R, false);
      hamiltonianCARESolver.computeP();

      approximateP(hamiltonianCARESolver.getP(), maxIterations, convergenceEpsilon);

      // K = R^-1 B^T P
      tempMatrix.reshape(B.getNumCols(), P.getNumCols());
      CommonOps.multTransA(B, P, tempMatrix);
      CommonOps.mult(Rinv, tempMatrix, K);
   }

   /** {inheritDoc} */
   public DenseMatrix64F getP()
   {
      return P;
   }

   /**
    * Applies the Newton algorithm.
    *
    * @param initialP initial solution
    * @param maxIterations maximum number of iterations allowed
    * @param epsilon convergence threshold
    */
   private void approximateP(DenseMatrix64F initialP, int maxIterations, double epsilon)
   {
      n = A.getNumRows();
      m = Rinv.getNumRows();
      K.reshape(m, n);
      P.set(initialP);

      double error = 1.0;
      int i = 1;
      while (error > epsilon)
      {
         // K_i = R_inv B' P_i-1
         tempMatrix.reshape(B.getNumCols(), P.getNumCols());
         CommonOps.multTransA(B, P, tempMatrix);
         CommonOps.mult(Rinv, tempMatrix, K);

         // Ak = A - B K
         Ak.set(A);
         CommonOps.multAdd(-1.0, B, K, Ak);

         // Qk = Q + K' R K
         Qk.reshape(n, n);
         NativeCommonOps.multQuad(K, R, Qk);
         CommonOps.addEquals(Qk, Q);
         CommonOps.scale(-1.0, Q);

         lyapunovSolver.setMatrices(Ak, Qk);
         lyapunovSolver.solve();
         DenseMatrix64F Pk = lyapunovSolver.getX();

         // error = norm(P - P1);
         error = distance(P, Pk);

         P.set(Pk);
         i++;
         if (i > maxIterations)
            throw new RuntimeException("Convergence failed.");
      }
   }

   private static double distance(DenseMatrix64F A, DenseMatrix64F B)
   {
      MatrixChecking.assertRowDimensionsMatch(A, B);
      MatrixChecking.assertColDimensionsMatch(A, B);

      double norm = 0.0;
      for (int col = 0; col < A.getNumCols(); col++)
      {
         double rowSum = 0.0;
         for (int row = 0; row < A.getNumRows(); row++)
         {
            rowSum += MathTools.square(A.get(row, col) - B.get(row, col));
         }
         norm += MathTools.square(rowSum);
      }

      return norm;
   }
}
