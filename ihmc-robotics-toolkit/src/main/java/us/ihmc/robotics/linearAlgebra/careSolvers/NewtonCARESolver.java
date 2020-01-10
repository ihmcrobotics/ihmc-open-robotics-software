package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.matrixlib.NativeCommonOps;

/**
 * This solver computes the solution to the algebraic Riccati equation, using an iterative Newton algorithm outlined in
 * http://et.engr.iupui.edu//~skoskie/ECE684/Riccati_algorithms.pdf.
 *
 * <p>
 *    An initial estimate of the solution is required, and calculated using the backend solver provided at construction.
 * </p>
 * <p>
 *    The maximum number of iterations can be set using the maxIterations value in the constructor. The convergence epsilon, which says when the value of P
 *    stops changing, can be set in the constructor as well.
 * </p>
 */
public class NewtonCARESolver extends AbstractCARESolver
{
  private static final int defaultMaxIterations = 100000;
  private final int maxIterations;

  private static final double defaultConvergenceEpsilon = 1e-8;
  private final double convergenceEpsilon;

  private final DenseMatrix64F PE = new DenseMatrix64F(0, 0);

  private final DenseMatrix64F Ak = new DenseMatrix64F(0, 0);
  private final DenseMatrix64F Qk = new DenseMatrix64F(0, 0);

  private final DenseMatrix64F EInverse = new DenseMatrix64F(0, 0);

  private final LyapunovEquationSolver lyapunovSolver = new LyapunovEquationSolver();

  private final CARESolver backendSolver;

  public NewtonCARESolver()
  {
     this(new EigenvectorCARESolver());
  }

  public NewtonCARESolver(CARESolver backendSolver)
  {
     this(backendSolver, defaultMaxIterations, defaultConvergenceEpsilon);
  }

  public NewtonCARESolver(CARESolver backendSolver, int maxIterations, double convergenceEpsilon)
  {
     this.backendSolver = backendSolver;
     this.maxIterations = maxIterations;
     this.convergenceEpsilon = convergenceEpsilon;
  }

  /** {@inheritDoc} */
  public DenseMatrix64F computeP()
  {
     backendSolver.setMatrices(A, hasE ? E : null, M, Q);
     backendSolver.computeP();

     if (hasE)
     {
        PE.reshape(n, n);
        CommonOps.mult(backendSolver.getP(), E, PE);
     }
     else
        PE.set(backendSolver.getP());

     Ak.reshape(n, n);
     Qk.reshape(n, n);

     double error = 1.0;
     int i = 1;
     while (error > convergenceEpsilon)
     {
        // Ak = A - M PEk
        CommonOps.mult(-1.0, M, PE, Ak);
        CommonOps.addEquals(Ak, A);

        // Qk = Q + K' R K
        NativeCommonOps.multQuad(PE, M, Qk);
        CommonOps.addEquals(Qk, Q);
        CommonOps.scale(-1.0, Q);

        lyapunovSolver.setMatrices(Ak, Qk);
        lyapunovSolver.solve();
        DenseMatrix64F Pk = lyapunovSolver.getX();

        // error = normSquared(P - P1);
        error = MatrixToolsLocal.distance(PE, Pk);

        PE.set(Pk);
        i++;
        if (i > maxIterations)
           throw new RuntimeException("Convergence failed.");
     }

     if (hasE)
     {
        EInverse.reshape(n, n);
        P.reshape(n, n);
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
