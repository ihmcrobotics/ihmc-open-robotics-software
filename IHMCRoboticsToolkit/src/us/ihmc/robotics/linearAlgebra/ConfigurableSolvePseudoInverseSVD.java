package us.ihmc.robotics.linearAlgebra;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

/**
 * <p>
 * The pseudo-inverse is typically used to solve over determined system for which there is no unique solution.<br>
 * x=inv(A<sup>T</sup>A)A<sup>T</sup>b<br>
 * where A &isin; &real; <sup>m &times; n</sup> and m &ge; n.
 * </p>
 *
 * <p>
 * This class implements the Moore-Penrose pseudo-inverse using SVD and should never fail.  Alternative implementations
 * can use Cholesky decomposition, but those will fail if the A<sup>T</sup>A matrix is singular.
 * However the Cholesky implementation is much faster.
 * </p>
 *
 * @author Peter Abeles
 */
public class ConfigurableSolvePseudoInverseSVD implements LinearSolver<DenseMatrix64F>
{
   // Used to compute pseudo inverse
   private SingularValueDecomposition<DenseMatrix64F> svd;

   // the results of the pseudo-inverse
   private DenseMatrix64F pinv = new DenseMatrix64F(1, 1);

   // if below this limit: consider singular value zero
   private final double singularValueLimit;

   /**
    * Creates a new solver targeted at the specified matrix size.
    *
    * @param maxRows The expected largest matrix it might have to process.  Can be larger.
    * @param maxCols The expected largest matrix it might have to process.  Can be larger.
    */
   public ConfigurableSolvePseudoInverseSVD(int maxRows, int maxCols, double singularValueLimit)
   {
      svd = DecompositionFactory.svd(maxRows, maxCols, true, true, true);
      this.singularValueLimit = singularValueLimit;
   }

   public boolean setA(DenseMatrix64F A)
   {
      pinv.reshape(A.numCols, A.numRows, false);

      if (!svd.decompose(A))
         return false;

      DenseMatrix64F U_t = svd.getU(null, true);
      DenseMatrix64F V = svd.getV(null, false);
      double[] S = svd.getSingularValues();
      int N = Math.min(A.numRows, A.numCols);

      // compute the pseudo inverse of A
      for (int i = 0; i < N; i++)
      {
         double s = S[i];
         if (s < singularValueLimit)
            S[i] = 0;
         else
            S[i] = 1.0 / S[i];
      }

      // V*W
      for (int i = 0; i < V.numRows; i++)
      {
         int index = i * V.numCols;
         for (int j = 0; j < V.numCols; j++)
         {
            V.data[index++] *= S[j];
         }
      }

      // V*W*U^T
      CommonOps.mult(V, U_t, pinv);

      return true;
   }

   public double quality()
   {
      throw new IllegalArgumentException("Not supported by this solver.");
   }

   public void solve(DenseMatrix64F b, DenseMatrix64F x)
   {
      CommonOps.mult(pinv, b, x);
   }

   public void invert(DenseMatrix64F A_inv)
   {
      A_inv.set(pinv);
   }

   public boolean modifiesA()
   {
      return svd.inputModified();
   }

   public boolean modifiesB()
   {
      return false;
   }
}
