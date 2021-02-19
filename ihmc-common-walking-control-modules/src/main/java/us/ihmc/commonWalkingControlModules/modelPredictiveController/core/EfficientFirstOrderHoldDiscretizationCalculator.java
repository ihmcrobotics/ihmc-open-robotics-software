 package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.tools.EfficientMatrixExponentialCalculator;

 public class EfficientFirstOrderHoldDiscretizationCalculator implements DiscretizationCalculator
{
   private final EfficientMatrixExponentialCalculator matrixExponentialCalculator = new EfficientMatrixExponentialCalculator(6, 10, 10);

   private final DMatrixRMaj At = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj Bt = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj Ct = new DMatrixRMaj(0, 0);

   @Override
   public void compute(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj C, DMatrixRMaj Ad, DMatrixRMaj Bd, DMatrixRMaj Cd, double tickDuration)
   {
      int rows = A.getNumRows();
      int aCols = A.getNumCols();
      int bCols = B.getNumCols();
      int cCols = C.getNumCols();
      At.reshape(rows, rows);
      Bt.reshape(rows, bCols);
      Ct.reshape(rows, cCols);
      Ad.reshape(rows, aCols);
      Bd.reshape(rows, bCols);
      Cd.reshape(rows, cCols);

      CommonOps_DDRM.scale(tickDuration, A, At);
      CommonOps_DDRM.scale(tickDuration, B, Bt);
      CommonOps_DDRM.scale(tickDuration, C, Ct);

      matrixExponentialCalculator.reshape(rows, bCols, cCols);
      matrixExponentialCalculator.compute(At, Bt, Ct, Ad, Bd, Cd);
   }
}
