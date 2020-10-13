package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

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

  private final DMatrixRMaj PE = new DMatrixRMaj(0, 0);

  private final DMatrixRMaj Ak = new DMatrixRMaj(0, 0);
  private final DMatrixRMaj Qk = new DMatrixRMaj(0, 0);

  private final DMatrixRMaj EInverse = new DMatrixRMaj(0, 0);

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
  public DMatrixRMaj computeP()
  {
     backendSolver.setMatrices(A, hasE ? E : null, M, Q);
     backendSolver.computeP();

     if (hasE)
     {
        PE.reshape(n, n);
        CommonOps_DDRM.mult(backendSolver.getP(), E, PE);
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
        CommonOps_DDRM.mult(-1.0, M, PE, Ak);
        CommonOps_DDRM.addEquals(Ak, A);

        // Qk = Q + K' R K
        NativeCommonOps.multQuad(PE, M, Qk);
        CommonOps_DDRM.addEquals(Qk, Q);
        CommonOps_DDRM.scale(-1.0, Q);

        lyapunovSolver.setMatrices(Ak, Qk);
        lyapunovSolver.solve();
        DMatrixRMaj Pk = lyapunovSolver.getX();

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
        CommonOps_DDRM.mult(PE, EInverse, P);
     }
     else
     {
        P.set(PE);
     }

     isUpToDate = true;
     return P;
  }
}
