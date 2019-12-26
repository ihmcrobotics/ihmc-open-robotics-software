package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
import us.ihmc.matrixlib.NativeCommonOps;

public class NewNewtonCARESolver extends AbstractCARESolver
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

  private final NewCARESolver backendSolver;

  public NewNewtonCARESolver()
  {
     this(new NewEigenvectorCARESolver());
  }

  public NewNewtonCARESolver(NewCARESolver backendSolver)
  {
     this(backendSolver, defaultMaxIterations, defaultConvergenceEpsilon);
  }

  public NewNewtonCARESolver(NewCARESolver backendSolver, int maxIterations, double convergenceEpsilon)
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
        CommonOps.mult(backendSolver.getP(), E, PE);
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

        // error = norm(P - P1);
        error = MatrixToolsLocal.distance(PE, Pk);

        PE.set(Pk);
        i++;
        if (i > maxIterations)
           throw new RuntimeException("Convergence failed.");
     }

     if (hasE)
     {
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
