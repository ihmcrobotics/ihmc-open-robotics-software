package us.ihmc.robotics.linearAlgebra.cdreSolvers;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.linearAlgebra.careSolvers.CARESolver;
import us.ihmc.robotics.linearAlgebra.careSolvers.CARETools;
import us.ihmc.robotics.linearAlgebra.careSolvers.MatrixChecking;

public abstract class AbstractCDRESolver implements CDRESolver
{
   private final DMatrixRMaj Rinv = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj tempMatrix = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj RinvSTranspose = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj SRinvSTranspose = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj RinvSTransposeC = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj AHat = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj QHat = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj BTranspose = new DMatrixRMaj(0, 0);

   protected int n;
   protected final DMatrixRMaj M = new DMatrixRMaj(0, 0);
   protected final DMatrixRMaj A = new DMatrixRMaj(0, 0);
   protected final DMatrixRMaj E = new DMatrixRMaj(0, 0);
   protected final DMatrixRMaj Q = new DMatrixRMaj(0, 0);
   protected boolean hasE = false;

   protected final DMatrixRMaj P = new DMatrixRMaj(0, 0);

   protected boolean isUpToDate = false;

   /** {inheritDoc} */
   @Override
   public void setMatrices(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj C, DMatrixRMaj E, DMatrixRMaj Q, DMatrixRMaj R, DMatrixRMaj S)
   {
      int m = Rinv.getNumRows();
      int n = B.getNumRows();


      BTranspose.set(B);
      CommonOps_DDRM.transpose(BTranspose);

      Rinv.reshape(m, m);
      M.reshape(n, n);

      CARETools.computeM(BTranspose, R, Rinv, M);

      // A - B R^-1 S' C
      DMatrixRMaj ALocal;
      // C' Q C - C' S R^-1 S' C
      DMatrixRMaj QLocal;
      if (S == null)
      {
         ALocal = A;

         if (C == null)
         {
            QLocal = Q;
         }
         else
         {
            QHat.reshape(C.getNumCols(), C.getNumCols());
            NativeCommonOps.multQuad(C, Q, QHat);
            QLocal = QHat;
         }
      }
      else
      {
         AHat.set(A);

         RinvSTranspose.reshape(m, S.getNumCols());
         CommonOps_DDRM.multTransB(Rinv, S, RinvSTranspose);

         SRinvSTranspose.reshape(S.getNumRows(), S.getNumRows());
         CommonOps_DDRM.mult(S, RinvSTranspose, SRinvSTranspose);

         tempMatrix.set(Q);
         CommonOps_DDRM.subtractEquals(tempMatrix, SRinvSTranspose);

         if (C != null)
         {
            QHat.reshape(C.getNumCols(), C.getNumCols());

            RinvSTransposeC.reshape(RinvSTranspose.getNumRows(), C.getNumCols());
            CommonOps_DDRM.mult(RinvSTranspose, C, RinvSTransposeC);

            NativeCommonOps.multQuad(C, tempMatrix, QHat);
         }
         else
         {
            RinvSTransposeC.set(RinvSTranspose);

            QHat.set(tempMatrix);
         }

         CommonOps_DDRM.multAdd(-1.0, B, RinvSTransposeC, AHat);

         ALocal = AHat;
         QLocal = QHat;
      }

      setMatrices(ALocal, E, M, QLocal);
   }

   /** {inheritDoc} */
   @Override
   public void setMatrices(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj E, DMatrixRMaj Q, DMatrixRMaj R)
   {
      int m = Rinv.getNumRows();
      int n = B.getNumRows();
      Rinv.reshape(m, m);

      BTranspose.set(B);
      CommonOps_DDRM.transpose(BTranspose);

      M.reshape(n, n);

      CARETools.computeM(BTranspose, R, Rinv, M);

      setMatrices(A, E, M, Q);
   }

   /** {inheritDoc} */
   @Override
   public void setMatrices(DMatrixRMaj A, DMatrixRMaj E, DMatrixRMaj M, DMatrixRMaj Q)
   {
      MatrixChecking.assertMultiplicationCompatible(A, Q);

      this.n = A.getNumRows();
      this.A.set(A);
      if (E != null)
      {
         this.E.set(E);
         hasE = true;
      }
      else
      {
         hasE = false;
      }
      this.M.set(M);
      this.Q.set(Q);

      isUpToDate = false;
   }
}
