package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeCommonOps;

public class CARETools
{
   public static void computeS(DenseMatrix64F BTranspose, DenseMatrix64F R, DenseMatrix64F RinvToPack, DenseMatrix64F SToPack)
   {
      int n = BTranspose.getNumCols();
      if (RinvToPack == null)
         RinvToPack = new DenseMatrix64F(n, n);
      else
         RinvToPack.reshape(n, n);
      SToPack.reshape(n, n);

      NativeCommonOps.invert(R, RinvToPack);
      NativeCommonOps.multQuad(BTranspose, RinvToPack, SToPack);
   }

   public static void assembleHamiltonian(DenseMatrix64F A, DenseMatrix64F ATranspose, DenseMatrix64F Q, DenseMatrix64F S, DenseMatrix64F hamiltonianToPack)
   {
      int n = A.getNumRows();

      if (ATranspose == null)
      {
         ATranspose = new DenseMatrix64F(n, n);
         CommonOps.transpose(A, ATranspose);
      }

      hamiltonianToPack.reshape(2 * n, 2 * n);

      MatrixTools.setMatrixBlock(hamiltonianToPack, 0, 0, A, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(hamiltonianToPack, n, 0, Q, 0, 0, n, n, -1.0);
      MatrixTools.setMatrixBlock(hamiltonianToPack, 0, n, S, 0, 0, n, n, -1.0);
      MatrixTools.setMatrixBlock(hamiltonianToPack, n, n, ATranspose, 0, 0, n, n, -1.0);
   }

   public static void computeRiccatiRate(DenseMatrix64F P, DenseMatrix64F A, DenseMatrix64F Q, DenseMatrix64F S, DenseMatrix64F PDotToPack)
   {
      NativeCommonOps.multQuad(P, S, PDotToPack);
      CommonOps.scale(-1.0, PDotToPack);
      CommonOps.multAddTransA(A, P, PDotToPack);
      CommonOps.multAdd(P, A, PDotToPack);
      CommonOps.addEquals(PDotToPack, Q);
   }
}
