package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
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
 *    The solution is found by first finding a solution using the Hamiltonian using the solver {@link HamiltonianEigenCARESolver}.
 *    This is then used as initial guess for the Newton iterative algorithm outlined here: http://et.engr.iupui.edu//~skoskie/ECE684/Riccati_algorithms.pdf.
 * </p>
  * <p>
  *    The maximum number of iterations can be set using the maxIterations value in the constructor. The convergence epsilon, which says when the value of P
  *    stops changing, can be set in the constructor as well.
  * </p>
 */
public class NewtonCARESolver implements CARESolver
{
   /** Internally used maximum iterations. */
   private static final int defaultMaxIterations = 100;
   private final int maxIterations;

   /** Internally used epsilon criteria. */
   private static final double defaultConvergenceEpsilon = 1e-8;
   private final double convergenceEpsilon;

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

   private final LyapunovEquationSolver lyapunovSolver = new LyapunovEquationSolver();

   private final HamiltonianEigenCARESolver hamiltonianCARESolver = new HamiltonianEigenCARESolver();
   private final SingularValueDecomposition<DenseMatrix64F> svd = DecompositionFactory.svd(0, 0, false, false, false);

   private int n;
   private int m;

   private boolean isUpToDate = false;

   public NewtonCARESolver()
   {
      this(defaultMaxIterations, defaultConvergenceEpsilon);
   }

   public NewtonCARESolver(int maxIterations)
   {
      this(maxIterations, defaultConvergenceEpsilon);
   }

   public NewtonCARESolver(double convergenceEpsilon)
   {
      this(defaultMaxIterations, convergenceEpsilon);
   }

   public NewtonCARESolver(int maxIterations, double convergenceEpsilon)
   {
      this.maxIterations = maxIterations;
      this.convergenceEpsilon = convergenceEpsilon;
   }

   /** {@inheritDoc} */
   public void setMatrices(DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F Q, DenseMatrix64F R, boolean checkMatrices)
   {
      isUpToDate = false;
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

      this.A.set(A);
      this.B.set(B);
      this.Q.set(Q);
      this.R.set(R);
   }

   /** {@inheritDoc} */
   public DenseMatrix64F computeP()
   {
      n = A.getNumRows();
      m = R.getNumRows();

      Rinv.reshape(m, m);
      NativeCommonOps.invert(R, Rinv);

      hamiltonianCARESolver.setMatrices(A, B, Q, R, false);
      hamiltonianCARESolver.computeP();

      approximateP(hamiltonianCARESolver.getP(), maxIterations, convergenceEpsilon);

      isUpToDate = true;
      return P;
   }

   /** {inheritDoc} */
   public DenseMatrix64F getP()
   {
      return isUpToDate ? P : computeP();
   }

   /**\
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
      tempMatrix.reshape(m, n);

      double error = 1.0;
      int i = 1;
      while (error > epsilon)
      {
         // K_i = R_inv B' P_i-1
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
         error = MatrixToolsLocal.distance(P, Pk);

         P.set(Pk);
         i++;
         if (i > maxIterations)
            throw new RuntimeException("Convergence failed.");
      }
   }
}
