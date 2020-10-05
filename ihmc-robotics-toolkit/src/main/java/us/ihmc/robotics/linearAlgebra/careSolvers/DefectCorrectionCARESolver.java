package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.matrixlib.NativeCommonOps;

/**
 * This solver performs a numerical correction to the solution to the algebraic Riccati equation, using an iterative defect correction algorithm outlined in
 * Chapter 10 of "The Autonomous Linear Quadratic Control Problem." In practice, this algorithm should converge in one to two iterations. It is only meant to
 * correct for numerical inaccuracies in the original CARE solver, and is not meant as a standalone solver on its own.
 *
 * <p>
 *    An initial estimate of the solution is required, and calculated using the backend solver provided at construction.
 * </p>
 */
public class DefectCorrectionCARESolver extends AbstractCARESolver
{
  private static final int defaultMaxIterations = 1000;
  private final int maxIterations;

  private static final double defaultConvergenceEpsilon = 1e-12;
  private final double convergenceEpsilon;

  private final DMatrixRMaj X = new DMatrixRMaj(0, 0);

  private final DMatrixRMaj PE = new DMatrixRMaj(0, 0);
  private final DMatrixRMaj ASquiggle = new DMatrixRMaj(0, 0);
  private final DMatrixRMaj QSquiggle = new DMatrixRMaj(0, 0);

  private final CARESolver backendSolver;

  public DefectCorrectionCARESolver(CARESolver backendSolver)
  {
     this.backendSolver = backendSolver;
     this.maxIterations = defaultMaxIterations;
     this.convergenceEpsilon = defaultConvergenceEpsilon;
  }

  /** {@inheritDoc} */
  public DMatrixRMaj computeP()
  {
     backendSolver.setMatrices(A, hasE ? E : null, M, Q);

     P.set(backendSolver.getP());

     int iterations = 0;
     boolean converged = false;
     while (!converged)
     {
        X.set(computeErrorEstimate(P));
        CommonOps_DDRM.addEquals(P, X);

        converged = MatrixToolsLocal.isZero(X, convergenceEpsilon);

        if (iterations > maxIterations)
           throw new RuntimeException("Failed to converge.");

        iterations++;
     }

     isUpToDate = true;
     return P;
  }

  private DMatrixRMaj computeErrorEstimate(DMatrixRMaj currentValue)
  {
     PE.reshape(n, n);
     if (!hasE)
        PE.set(currentValue);
     else
        CommonOps_DDRM.mult(currentValue, E, PE);

     // QSquiggle = Q + A' P E + E' P A - E' P M P E
     NativeCommonOps.multQuad(PE, M, QSquiggle);
     CommonOps_DDRM.scale(-1.0, QSquiggle);

     CommonOps_DDRM.multAddTransA(PE, A, QSquiggle);
     CommonOps_DDRM.multAddTransA(A, PE, QSquiggle);
     CommonOps_DDRM.addEquals(QSquiggle, Q);

     // ASquiggle = A - M P E
     ASquiggle.set(A);
     CommonOps_DDRM.multAdd(-1.0, M, PE, ASquiggle);

     backendSolver.setMatrices(ASquiggle, hasE ? E : null, M, QSquiggle);

     return backendSolver.getP();
  }
}
