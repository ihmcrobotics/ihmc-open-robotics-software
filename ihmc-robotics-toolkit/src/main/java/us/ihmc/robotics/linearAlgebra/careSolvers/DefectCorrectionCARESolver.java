package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;

/**
* <p>
*    The solution is found by first finding a solution using the Hamiltonian using the solver {@link HamiltonianEigenCARESolver}.
*    This is then used as initial guess for the Newton iterative algorithm outlined here: http://et.engr.iupui.edu//~skoskie/ECE684/Riccati_algorithms.pdf.
* </p>
 * <p>
 *    The maximum number of iterations can be set using the maxIterations value in the constructor. The convergence epsilon, which says when the value of P
 *    stops changing, can be set in the constructor as well.
 * </p>
*/
public class DefectCorrectionCARESolver implements CARESolver
{
  /** Internally used maximum iterations. */
  private static final int defaultMaxIterations = 1000;
  private final int maxIterations;

  /** Internally used epsilon criteria. */
  private static final double defaultConvergenceEpsilon = 1e-8;
  private final double convergenceEpsilon;

  /** The solution of the algebraic Riccati equation. */
  private final DenseMatrix64F P = new DenseMatrix64F(0, 0);
  private final DenseMatrix64F X = new DenseMatrix64F(0, 0);

  /** The computed K. */
  private final DenseMatrix64F K = new DenseMatrix64F(0, 0);

  private final DenseMatrix64F A = new DenseMatrix64F(0, 0);
  private final DenseMatrix64F B = new DenseMatrix64F(0, 0);
  private final DenseMatrix64F BTranspose = new DenseMatrix64F(0, 0);
  private final DenseMatrix64F Q = new DenseMatrix64F(0, 0);
  private final DenseMatrix64F R = new DenseMatrix64F(0, 0);
  private final DenseMatrix64F Rinv = new DenseMatrix64F(0, 0);
  private final DenseMatrix64F Rk = new DenseMatrix64F(0, 0);
  private final DenseMatrix64F S = new DenseMatrix64F(0, 0);

  private final DenseMatrix64F Ak = new DenseMatrix64F(0, 0);
  private final DenseMatrix64F tempMatrix = new DenseMatrix64F(0, 0);


  private final CARESolver backendSolver;
  private final SingularValueDecomposition<DenseMatrix64F> svd = DecompositionFactory.svd(0, 0, false, false, false);

  private int n;
  private int m;

  private boolean isUpToDate = false;

  public DefectCorrectionCARESolver(CARESolver backendSolver)
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
     n = A.getNumRows();
     m = R.getNumRows();

     BTranspose.set(B);
     CommonOps.transpose(BTranspose);
     CARETools.computeM(BTranspose, R, Rinv, S);

     backendSolver.setMatrices(A, B, Q, R, false);

     approximateP(backendSolver.getP(), maxIterations, convergenceEpsilon);

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
     Ak.reshape(n, n);

     Rk.reshape(m, m);
     X.reshape(n, n);

     int iteration = 0;
     boolean converged = false;

     while (!converged)
     {
        CommonOps.mult(-1.0, S, P, Ak);
        CommonOps.addEquals(Ak, A);

//        backendSolver.setMatrices(Ak, );

        CARETools.computeRiccatiRate(P, A, Q, S, Rk);

        converged = isZero(Rk, epsilon);

        CommonOps.addEquals(P, Rk);

        iteration++;
        if (iteration > maxIterations)
           throw new RuntimeException("Convergence failed.");
     }
  }

  private boolean isZero(DenseMatrix64F P, double epsilon)
  {
     for (int i = 0; i < P.getNumElements(); i++)
     {
        if (!MathTools.epsilonEquals(P.get(i), 0.0, epsilon))
           return false;
     }

     return true;
  }
}
