package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.matrixlib.MatrixTools;

public class BilinearDiscretizationCalculator implements DiscretizationCalculator
{
   private final LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.general(6, 6);
   private final DMatrixRMaj tempA = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj leftA = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj rightA = new DMatrixRMaj(6, 6);

   @Override
   public void compute(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj C, DMatrixRMaj Ad, DMatrixRMaj Bd, DMatrixRMaj Cd, double tickDuration)
   {
      int size = A.getNumRows();
      tempA.reshape(size, size);
      leftA.reshape(size, size);
      rightA.reshape(size, size);

      CommonOps_DDRM.scale(-0.5 * tickDuration, A, tempA);
      MatrixTools.addDiagonal(tempA, 1.0);

      solver.setA(tempA);
      solver.invert(rightA);

      CommonOps_DDRM.scale(0.5 * tickDuration, A, leftA);
      MatrixTools.addDiagonal(leftA, 1.0);

      CommonOps_DDRM.mult(leftA, rightA, Ad);
      CommonOps_DDRM.scale(tickDuration, B, Bd);
      CommonOps_DDRM.scale(tickDuration, C, Cd);
   }
}
