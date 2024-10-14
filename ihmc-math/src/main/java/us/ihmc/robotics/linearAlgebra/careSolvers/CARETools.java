package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeCommonOps;

public class CARETools
{
   public static void computeM(DMatrixRMaj BTranspose, DMatrixRMaj R, DMatrixRMaj RinvToPack, DMatrixRMaj SToPack)
   {
      int n = BTranspose.getNumCols();
      if (RinvToPack == null)
         RinvToPack = new DMatrixRMaj(n, n);
      else
         RinvToPack.reshape(n, n);
      SToPack.reshape(n, n);

      NativeCommonOps.invert(R, RinvToPack);
      NativeCommonOps.multQuad(BTranspose, RinvToPack, SToPack);
   }

   public static void assembleHamiltonian(DMatrixRMaj A, DMatrixRMaj ATranspose, DMatrixRMaj Q, DMatrixRMaj S, DMatrixRMaj hamiltonianToPack)
   {
      int n = A.getNumRows();

      if (ATranspose == null)
      {
         ATranspose = new DMatrixRMaj(n, n);
         CommonOps_DDRM.transpose(A, ATranspose);
      }

      hamiltonianToPack.reshape(2 * n, 2 * n);

      MatrixTools.setMatrixBlock(hamiltonianToPack, 0, 0, A, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(hamiltonianToPack, n, 0, Q, 0, 0, n, n, -1.0);
      MatrixTools.setMatrixBlock(hamiltonianToPack, 0, n, S, 0, 0, n, n, -1.0);
      MatrixTools.setMatrixBlock(hamiltonianToPack, n, n, ATranspose, 0, 0, n, n, -1.0);
   }

   public static void computeRiccatiRate(DMatrixRMaj P, DMatrixRMaj A, DMatrixRMaj Q, DMatrixRMaj S, DMatrixRMaj PDotToPack)
   {
      NativeCommonOps.multQuad(P, S, PDotToPack);
      CommonOps_DDRM.scale(-1.0, PDotToPack);
      CommonOps_DDRM.multAddTransA(A, P, PDotToPack);
      CommonOps_DDRM.multAdd(P, A, PDotToPack);
      CommonOps_DDRM.addEquals(PDotToPack, Q);
   }
}
