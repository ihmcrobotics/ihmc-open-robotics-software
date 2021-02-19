package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.robotics.linearAlgebra.MatrixExponentialCalculator;

public class HybridDiscretizationCalculator implements DiscretizationCalculator
{
   private final MatrixExponentialCalculator matrixExponentialCalculator = new MatrixExponentialCalculator(6);

   @Override
   public void compute(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj C, DMatrixRMaj Ad, DMatrixRMaj Bd, DMatrixRMaj Cd, double tickDuration)
   {
      matrixExponentialCalculator.reshape(6);
      CommonOps_DDRM.scale(tickDuration, A);
      matrixExponentialCalculator.compute(Ad, A);

      CommonOps_DDRM.scale(tickDuration, B, Bd);
      CommonOps_DDRM.scale(tickDuration, C, Cd);
   }
}
