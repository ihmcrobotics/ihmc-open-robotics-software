package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
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
public class Newton2CARESolver extends AbstractCARESolver
{
  /** Internally used maximum iterations. */
  private static final int defaultMaxIterations = 10000;
  private static final double defaultConvergenceEpsilon = 1e-12;

  private final int maxIterations;
  private final double convergenceEpsilon;

  private final DenseMatrix64F PE = new DenseMatrix64F(0, 0);
  private final DenseMatrix64F PDotk = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F EInverse = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Ak = new DenseMatrix64F(0, 0);

  private final CARESolver backendSolver;

  private final LyapunovEquationSolver lyapunovSolver = new LyapunovEquationSolver();

  public Newton2CARESolver(CARESolver backendSolver)
  {
     this.backendSolver = backendSolver;
     this.maxIterations = defaultMaxIterations;
     this.convergenceEpsilon = defaultConvergenceEpsilon;
  }


  /** {@inheritDoc} */
  public DenseMatrix64F computeP()
  {
     int n = A.getNumRows();


     Ak.reshape(n, n);
     PDotk.reshape(n, n);
     PE.reshape(n, n);

     backendSolver.setMatrices(A, hasE ? E : null, M, Q);
     if (hasE)
        CommonOps.mult(backendSolver.getP(), E, PE);
     else
        PE.set(backendSolver.getP());

     int iteration = 0;
     boolean converged = false;

     while (!converged)
     {
        CARETools.computeRiccatiRate(PE, A, Q, M, PDotk);

        // Ak = A - M P
        CommonOps.mult(-1.0, M, PE, Ak);
        CommonOps.addEquals(Ak, A);

        lyapunovSolver.setMatrices(Ak, PDotk);
        DenseMatrix64F Pk = lyapunovSolver.getX();

        converged = MatrixToolsLocal.isZero(Pk, convergenceEpsilon);

        CommonOps.addEquals(PE, Pk);

        iteration++;
        if (iteration > maxIterations)
           throw new RuntimeException("Convergence failed.");
     }

     if (hasE)
     {
        P.reshape(n, n);
        EInverse.reshape(n, n);
        NativeCommonOps.invert(E, EInverse);
        CommonOps.mult(PE, EInverse, P);
     }
     else
     {
        P.set(PE);
     }

     isUpToDate = true;
     return P;
  }
}
