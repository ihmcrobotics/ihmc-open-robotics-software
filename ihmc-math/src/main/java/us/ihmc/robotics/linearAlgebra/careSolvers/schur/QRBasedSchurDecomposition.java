package us.ihmc.robotics.linearAlgebra.careSolvers.schur;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.QRDecomposition;

import us.ihmc.robotics.linearAlgebra.careSolvers.MatrixToolsLocal;

/**
 * This algorithm decomposes the A matrix into its Schur components, formed as
 * A = UTU'
 * It is based on the QR algorithm, as described in
 * http://people.inf.ethz.ch/arbenz/ewp/Lnotes/chapter4.pdf
 */
public class QRBasedSchurDecomposition implements SchurDecomposition<DMatrixRMaj>
{
   private final QRDecomposition<DMatrixRMaj> qrDecomposition;
   private final DMatrixRMaj T;
   private final DMatrixRMaj Tprev;
   private final DMatrixRMaj Uprev;

   private final DMatrixRMaj U;
   private final DMatrixRMaj Q;
   private final DMatrixRMaj R;

   private int maxIterations = 1000000;
   private double epsilon = 1e-6;

   public QRBasedSchurDecomposition(int size)
   {
      qrDecomposition = DecompositionFactory_DDRM.qr(size, size);
      T = new DMatrixRMaj(size, size);
      Tprev = new DMatrixRMaj(size, size);
      Uprev = new DMatrixRMaj(size, size);
      Q = new DMatrixRMaj(size, size);
      R = new DMatrixRMaj(size, size);
      U = new DMatrixRMaj(size, size);
   }

   public void setMaxIterations(int maxIterations)
   {
      this.maxIterations = maxIterations;
   }

   public void setConvergenceEpsilon(double epsilon)
   {
      this.epsilon = epsilon;
   }

   public boolean decompose(DMatrixRMaj A)
   {
      if (A.getNumRows() != A.getNumCols())
         throw new IllegalArgumentException("A matrix is not square.");

      boolean converged = false;
      int iterations = 0;

      R.reshape(A.getNumRows(), A.getNumCols());
      Q.reshape(A.getNumRows(), A.getNumCols());
      T.reshape(A.getNumRows(), A.getNumCols());
      U.reshape(A.getNumRows(), A.getNumCols());
      Uprev.reshape(A.getNumRows(), A.getNumCols());

      Tprev.set(A);
      CommonOps_DDRM.setIdentity(Uprev);
      while (!converged)
      {
         if (iterations > maxIterations)
            return false;

         qrDecomposition.decompose(Tprev);
         qrDecomposition.getQ(Q, false);
         qrDecomposition.getR(R, false);

         CommonOps_DDRM.mult(R, Q, T);

         CommonOps_DDRM.mult(Uprev, Q, U);

         double TDistance = MatrixToolsLocal.distance(T, Tprev);
         double UDistance = MatrixToolsLocal.distance(U, Uprev);
         converged = TDistance < epsilon;
         converged &= UDistance < epsilon;

         Tprev.set(T);
         Uprev.set(U);

         iterations++;
      }

      return true;
   }

   public DMatrixRMaj getU(DMatrixRMaj UToPack)
   {
      if (UToPack != null)
      {
         UToPack.set(U);
         return null;
      }
      else
      {
         return new DMatrixRMaj(U);
      }
   }

   public DMatrixRMaj getT(DMatrixRMaj TToPack)
   {
      if (TToPack != null)
      {
         TToPack.set(T);
         return null;
      }
      else
      {
         return new DMatrixRMaj(T);
      }
   }

   @Override
   public boolean inputModified()
   {
      return false;
   }
}
