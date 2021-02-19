package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.NormOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.linearAlgebra.MatrixExponentialCalculator;

public class FirstOrderHoldDiscretizationCalculator implements DiscretizationCalculator
{
   private final MatrixExponentialCalculator matrixExponentialCalculator = new MatrixExponentialCalculator(6);
   private final DMatrixRMaj bigMatrix = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj bigMatrixD = new DMatrixRMaj(0, 0);

   @Override
   public void compute(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj C, DMatrixRMaj Ad, DMatrixRMaj Bd, DMatrixRMaj Cd, double tickDuration)
   {
      int rows = A.getNumRows();
      int aCols = A.getNumCols();
      int bCols = B.getNumCols();
      int cCols = C.getNumCols();
      int size = aCols + bCols + cCols;
      bigMatrix.reshape(size, size);
      bigMatrixD.reshape(size, size);
      bigMatrix.zero();
      Ad.reshape(rows, aCols);
      Bd.reshape(rows, bCols);
      Cd.reshape(rows, cCols);

      int col = 0;
      MatrixTools.setMatrixBlock(bigMatrix, 0, col, A, 0, 0, rows, aCols, tickDuration);
      MatrixTools.setMatrixBlock(bigMatrix, 0, aCols, B, 0, 0, rows, bCols, tickDuration);
      MatrixTools.setMatrixBlock(bigMatrix, 0, aCols + bCols, C, 0, 0, rows, cCols, tickDuration);

      matrixExponentialCalculator.reshape(size);
      matrixExponentialCalculator.compute(bigMatrixD, bigMatrix);

      MatrixTools.setMatrixBlock(Ad, 0, 0, bigMatrixD, 0, 0, rows, aCols, 1.0);
      MatrixTools.setMatrixBlock(Bd, 0, 0, bigMatrixD, 0, aCols, rows, bCols, 1.0);
      MatrixTools.setMatrixBlock(Cd, 0, 0, bigMatrixD, 0, aCols + bCols, rows, cCols, 1.0);
   }
}
