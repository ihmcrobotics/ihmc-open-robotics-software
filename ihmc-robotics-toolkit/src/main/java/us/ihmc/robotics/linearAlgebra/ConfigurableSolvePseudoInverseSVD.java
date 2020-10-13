package us.ihmc.robotics.linearAlgebra;

import org.ejml.UtilEjml;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.SingularValueDecomposition_F64;
import org.ejml.interfaces.linsol.LinearSolverDense;

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
public class ConfigurableSolvePseudoInverseSVD implements LinearSolverDense<DMatrixRMaj>
{
   // Used to compute pseudo inverse
   private SingularValueDecomposition_F64<DMatrixRMaj> svd;

   // the results of the pseudo-inverse
   private DMatrixRMaj pinv = new DMatrixRMaj(1, 1);

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
      svd = DecompositionFactory_DDRM.svd(maxRows, maxCols, true, true, true);
      this.singularValueLimit = singularValueLimit;
   }

   /**
    * Will target matrices around size 100.
    */
   public ConfigurableSolvePseudoInverseSVD()
   {
      this(100, 100, 100.0 * UtilEjml.EPS);
   }

   private final DMatrixRMaj V = new DMatrixRMaj(1, 1);

   @Override
   public boolean setA(DMatrixRMaj A)
   {
      pinv.reshape(A.numCols, A.numRows, false);

      if (!svd.decompose(A))
         return false;

      DMatrixRMaj U_t = svd.getU(null, true);
      DMatrixRMaj V_t = svd.getV(null, true);
      V.reshape(V_t.getNumCols(), V_t.getNumRows());
      CommonOps_DDRM.transpose(V_t, V);
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
      CommonOps_DDRM.mult(V, U_t, pinv);

      return true;
   }

   @Override
   public double quality()
   {
      throw new IllegalArgumentException("Not supported by this solver.");
   }

   @Override
   public void solve(DMatrixRMaj b, DMatrixRMaj x)
   {
      CommonOps_DDRM.mult(pinv, b, x);
   }

   @Override
   public void invert(DMatrixRMaj A_inv)
   {
      A_inv.set(pinv);
   }

   @Override
   public boolean modifiesA()
   {
      return svd.inputModified();
   }

   @Override
   public boolean modifiesB()
   {
      return false;
   }
   
   @Override
   public SingularValueDecomposition_F64<DMatrixRMaj> getDecomposition()
   {
      return svd;
   }
}
