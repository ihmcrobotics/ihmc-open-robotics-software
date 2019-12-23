package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;

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
public class Newton2CARESolver implements CARESolver
{
  /** Internally used maximum iterations. */
  private static final int defaultMaxIterations = 10000;
  private final int maxIterations;

  /** Internally used epsilon criteria. */
  private static final double defaultConvergenceEpsilon = 1e-12;
  private final double convergenceEpsilon;

  /** The solution of the algebraic Riccati equation. */
  private final DenseMatrix64F P = new DenseMatrix64F(0, 0);

  private final DenseMatrix64F A = new DenseMatrix64F(0, 0);
  private final DenseMatrix64F B = new DenseMatrix64F(0, 0);
  private final DenseMatrix64F BTranspose = new DenseMatrix64F(0, 0);
  private final DenseMatrix64F Q = new DenseMatrix64F(0, 0);
  private final DenseMatrix64F R = new DenseMatrix64F(0, 0);
  private final DenseMatrix64F Rinv = new DenseMatrix64F(0, 0);
  private final DenseMatrix64F PDotk = new DenseMatrix64F(0, 0);
  private final DenseMatrix64F S = new DenseMatrix64F(0, 0);

  private final DenseMatrix64F Ak = new DenseMatrix64F(0, 0);

  private final CARESolver backendSolver;

  private final LyapunovEquationSolver lyapunovSolver = new LyapunovEquationSolver();

  private final SingularValueDecomposition<DenseMatrix64F> svd = DecompositionFactory.svd(0, 0, false, false, false);

  private boolean isUpToDate = false;

  public Newton2CARESolver(CARESolver backendSolver)
  {
     this.backendSolver = backendSolver;
     this.maxIterations = defaultMaxIterations;
     this.convergenceEpsilon = defaultConvergenceEpsilon;
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
     int n = A.getNumRows();
     int m = R.getNumRows();

     BTranspose.set(B);
     CommonOps.transpose(BTranspose);
     CARETools.computeS(BTranspose, R, Rinv, S);

     backendSolver.setMatrices(A, B, Q, R, false);
     P.set(backendSolver.getP());

     Ak.reshape(n, n);
     PDotk.reshape(m, m);

     int iteration = 0;
     boolean converged = false;

     while (!converged)
     {
        CARETools.computeRiccatiRate(P, A, Q, S, PDotk);

        // Ak = A - S P
        CommonOps.mult(-1.0, S, P, Ak);
        CommonOps.addEquals(Ak, A);

        lyapunovSolver.setMatrices(Ak, PDotk);
        DenseMatrix64F Pk = lyapunovSolver.getX();

        converged = isZero(Pk, convergenceEpsilon);

        CommonOps.addEquals(P, Pk);

        iteration++;
        if (iteration > maxIterations)
           throw new RuntimeException("Convergence failed.");
     }

     isUpToDate = true;
     return P;
  }

  /** {inheritDoc} */
  public DenseMatrix64F getP()
  {
     return isUpToDate ? P : computeP();
  }

  private static boolean isZero(DenseMatrix64F P, double epsilon)
  {
     for (int i = 0; i < P.getNumElements(); i++)
     {
        if (!MathTools.epsilonEquals(P.get(i), 0.0, epsilon))
           return false;
     }

     return true;
  }
}
