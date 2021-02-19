package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commons.MathTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.linearAlgebra.MatrixExponentialCalculator;

public class DiscreteDiscretizationCalculator implements DiscretizationCalculator
{
   @Override
   public void compute(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj C, DMatrixRMaj Ad, DMatrixRMaj Bd, DMatrixRMaj Cd, double tickDuration)
   {
      CommonOps_DDRM.scale(tickDuration, A, Ad);
      MatrixTools.addDiagonal(Ad, 1.0);

      CommonOps_DDRM.scale(tickDuration, B, Bd);
      CommonOps_DDRM.scale(tickDuration, C, Cd);
   }
}
