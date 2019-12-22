package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeCommonOps;

public class CARETools
{
   public static void computeS(DenseMatrix64F BTranspose, DenseMatrix64F R, DenseMatrix64F RinvToPack, DenseMatrix64F SToPack)
   {
      int n = BTranspose.getNumCols();
      RinvToPack.reshape(n, n);
      SToPack.reshape(n, n);

      NativeCommonOps.invert(R, RinvToPack);
      NativeCommonOps.multQuad(BTranspose, RinvToPack, SToPack);
   }

   public static void assembleHamiltonian(DenseMatrix64F A, DenseMatrix64F ATranspose, DenseMatrix64F Q, DenseMatrix64F S, DenseMatrix64F hamiltonianToPack)
   {
      int n = A.getNumRows();

      hamiltonianToPack.reshape(2 * n, 2 * n);

      MatrixTools.setMatrixBlock(hamiltonianToPack, 0, 0, A, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(hamiltonianToPack, n, 0, Q, 0, 0, n, n, -1.0);
      MatrixTools.setMatrixBlock(hamiltonianToPack, 0, n, S, 0, 0, n, n, -1.0);
      MatrixTools.setMatrixBlock(hamiltonianToPack, n, n, ATranspose, 0, 0, n, n, -1.0);
   }
}
