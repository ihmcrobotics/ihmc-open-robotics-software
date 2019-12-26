package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.linearAlgebra.careSolvers.schur.SchurDecomposition;
import us.ihmc.robotics.linearAlgebra.careSolvers.schur.SchurDecompositionFactory;

/**
 * <p>
 *   Algorithm taken from
 *   Laub, "A Schur Method for Solving Algebraic Riccati Equations."
 *   http://dspace.mit.edu/bitstream/handle/1721.1/1301/R-0859-05666488.pdf
 * </p>
 */
public class SchurCARESolver
{
   /** The solution of the algebraic Riccati equation. */
   private final DenseMatrix64F P = new DenseMatrix64F(0, 0);

   private final SingularValueDecomposition<DenseMatrix64F> svd = DecompositionFactory.svd(0, 0, false, false, false);
   private final SchurDecomposition<DenseMatrix64F> schur = SchurDecompositionFactory.qrBased(0);

   private final DenseMatrix64F BTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F ATranspose = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F A = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Q = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Rinv = new DenseMatrix64F(0, 0);

   /** Hamiltonian of the Riccati equation. */
   private final DenseMatrix64F H = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F M = new DenseMatrix64F(0, 0);

   private boolean isUpToDate = false;
   private int n;

   /** {@inheritDoc} */
   public void setMatrices(DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F Q, DenseMatrix64F R)
   {
      setMatrices(A, B, Q, R, true);
   }

   public void setMatrices(DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F Q, DenseMatrix64F R, boolean checkMatrices)
   {
      isUpToDate = false;

      // checking A
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

      this.n = A.getNumRows();

      BTranspose.set(B);
      ATranspose.set(A);
      CommonOps.transpose(BTranspose);
      CommonOps.transpose(ATranspose);

      CARETools.computeM(BTranspose, R, Rinv, M);

      this.n = A.getNumRows();

      this.A.set(A);
      this.Q.set(Q);
   }

   /** {@inheritDoc} */
   public DenseMatrix64F computeP()
   {
      // defining Hamiltonian
      CARETools.assembleHamiltonian(A, ATranspose, Q, M, H);

      schur.decompose(H);
      // FIXME need to reorder so that the negative eigen values precede nonnegative eigen values
      DenseMatrix64F U = schur.getU(null);
      DenseMatrix64F T = schur.getT(null);

      DenseMatrix64F U11 = new DenseMatrix64F(n, n);
      DenseMatrix64F U11Inv = new DenseMatrix64F(n, n);
      DenseMatrix64F U21 = new DenseMatrix64F(n, n);

      MatrixTools.setMatrixBlock(U11, 0 ,0, U, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(U21, 0 ,0, U, n, 0, n, n, 1.0);

      // TODO solve this with a solver.
      NativeCommonOps.invert(U11, U11Inv);

      P.reshape(n, n);
      CommonOps.mult(U21, U11Inv, P);

      isUpToDate = true;

      return P;
   }

   /** {inheritDoc} */
   public DenseMatrix64F getP()
   {
      return isUpToDate ? P : computeP();
   }
}
