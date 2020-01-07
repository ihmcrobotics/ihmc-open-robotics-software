package us.ihmc.robotics.linearAlgebra.cdreSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.linearAlgebra.careSolvers.CARESolver;
import us.ihmc.robotics.linearAlgebra.careSolvers.CARETools;
import us.ihmc.robotics.linearAlgebra.careSolvers.MatrixChecking;

public abstract class AbstractCDRESolver implements CDRESolver
{
   private final DenseMatrix64F Rinv = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F RinvSTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F SRinvSTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F RinvSTransposeC = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F AHat = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F QHat = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F BTranspose = new DenseMatrix64F(0, 0);

   protected int n;
   protected final DenseMatrix64F M = new DenseMatrix64F(0, 0);
   protected final DenseMatrix64F A = new DenseMatrix64F(0, 0);
   protected final DenseMatrix64F E = new DenseMatrix64F(0, 0);
   protected final DenseMatrix64F Q = new DenseMatrix64F(0, 0);
   protected boolean hasE = false;

   protected final DenseMatrix64F P = new DenseMatrix64F(0, 0);

   protected boolean isUpToDate = false;

   /** {inheritDoc} */
   @Override
   public void setMatrices(DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F C, DenseMatrix64F E, DenseMatrix64F Q, DenseMatrix64F R, DenseMatrix64F S)
   {
      int m = Rinv.getNumRows();
      int n = B.getNumRows();


      BTranspose.set(B);
      CommonOps.transpose(BTranspose);

      Rinv.reshape(m, m);
      M.reshape(n, n);

      CARETools.computeM(BTranspose, R, Rinv, M);

      // A - B R^-1 S' C
      DenseMatrix64F ALocal;
      // C' Q C - C' S R^-1 S' C
      DenseMatrix64F QLocal;
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
         CommonOps.multTransB(Rinv, S, RinvSTranspose);

         SRinvSTranspose.reshape(S.getNumRows(), S.getNumRows());
         CommonOps.mult(S, RinvSTranspose, SRinvSTranspose);

         tempMatrix.set(Q);
         CommonOps.subtractEquals(tempMatrix, SRinvSTranspose);

         if (C != null)
         {
            QHat.reshape(C.getNumCols(), C.getNumCols());

            RinvSTransposeC.reshape(RinvSTranspose.getNumRows(), C.getNumCols());
            CommonOps.mult(RinvSTranspose, C, RinvSTransposeC);

            NativeCommonOps.multQuad(C, tempMatrix, QHat);
         }
         else
         {
            RinvSTransposeC.set(RinvSTranspose);

            QHat.set(tempMatrix);
         }

         CommonOps.multAdd(-1.0, B, RinvSTransposeC, AHat);

         ALocal = AHat;
         QLocal = QHat;
      }

      setMatrices(ALocal, E, M, QLocal);
   }

   /** {inheritDoc} */
   @Override
   public void setMatrices(DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F E, DenseMatrix64F Q, DenseMatrix64F R)
   {
      int m = Rinv.getNumRows();
      int n = B.getNumRows();
      Rinv.reshape(m, m);

      BTranspose.set(B);
      CommonOps.transpose(BTranspose);

      M.reshape(n, n);

      CARETools.computeM(BTranspose, R, Rinv, M);

      setMatrices(A, E, M, Q);
   }

   /** {inheritDoc} */
   @Override
   public void setMatrices(DenseMatrix64F A, DenseMatrix64F E, DenseMatrix64F M, DenseMatrix64F Q)
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
