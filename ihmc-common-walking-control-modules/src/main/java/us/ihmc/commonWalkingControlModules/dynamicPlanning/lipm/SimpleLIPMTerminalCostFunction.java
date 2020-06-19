package us.ihmc.commonWalkingControlModules.dynamicPlanning.lipm;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.lipm.SimpleLIPMDynamics.controlVectorSize;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.lipm.SimpleLIPMDynamics.stateVectorSize;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.matrixlib.DiagonalMatrixTools;
import us.ihmc.trajectoryOptimization.DefaultDiscreteState;
import us.ihmc.trajectoryOptimization.LQTrackingCostFunction;

public class SimpleLIPMTerminalCostFunction implements LQTrackingCostFunction<DefaultDiscreteState>
{
   private final DMatrixRMaj Q = new DMatrixRMaj(stateVectorSize, stateVectorSize);
   private final DMatrixRMaj R = new DMatrixRMaj(controlVectorSize, controlVectorSize);

   public SimpleLIPMTerminalCostFunction()
   {
      Q.set(0, 0, 1e3);
      Q.set(1, 1, 1e3);
      Q.set(2, 2, 1e6);
      Q.set(3, 3, 1e6);

      R.set(0, 0, 0.0);
      R.set(1, 1, 0.0);
   }

   private DMatrixRMaj tempStateMatrix = new DMatrixRMaj(stateVectorSize, 1);
   private DMatrixRMaj tempControlMatrix = new DMatrixRMaj(controlVectorSize, 1);
   private DMatrixRMaj tempWX = new DMatrixRMaj(stateVectorSize, 1);
   private DMatrixRMaj tempWU = new DMatrixRMaj(controlVectorSize, 1);

   @Override
   public double getCost(DefaultDiscreteState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj desiredControlVector,
                         DMatrixRMaj desiredStateVector, DMatrixRMaj constants)
   {
      CommonOps_DDRM.subtract(controlVector, desiredControlVector, tempControlMatrix);
      CommonOps_DDRM.subtract(stateVector, desiredStateVector, tempStateMatrix);

      DiagonalMatrixTools.preMult(Q, tempStateMatrix, tempWX);
      DiagonalMatrixTools.preMult(R, tempControlMatrix, tempWU);

      return CommonOps_DDRM.dot(tempControlMatrix, tempWU) + CommonOps_DDRM.dot(tempStateMatrix, tempWX);
   }

   /** L_x(X_k, U_k) */
   @Override
   public void getCostStateGradient(DefaultDiscreteState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj desiredControlVector,
                                    DMatrixRMaj desiredStateVector, DMatrixRMaj constants, DMatrixRMaj matrixToPack)
   {
      CommonOps_DDRM.subtract(stateVector, desiredStateVector, tempStateMatrix);
      DiagonalMatrixTools.preMult(Q, tempStateMatrix, matrixToPack);
   }

   /** L_u(X_k, U_k) */
   @Override
   public void getCostControlGradient(DefaultDiscreteState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj desiredControlVecotr,
                                      DMatrixRMaj desiredStateVector, DMatrixRMaj constants, DMatrixRMaj matrixToPack)
   {
      CommonOps_DDRM.subtract(controlVector, desiredControlVecotr, tempControlMatrix);
      DiagonalMatrixTools.preMult(R, tempControlMatrix, matrixToPack);
   }

   /** L_xx(X_k, U_k) */
   @Override
   public void getCostStateHessian(DefaultDiscreteState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj constants,
                                   DMatrixRMaj matrixToPack)
   {
      matrixToPack.set(Q);
   }

   /** L_uu(X_k, U_k) */
   @Override
   public void getCostControlHessian(DefaultDiscreteState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj constants,
                                     DMatrixRMaj matrixToPack)

   {
      matrixToPack.set(R);
   }

   /** L_ux(X_k, U_k) */
   @Override
   public void getCostStateGradientOfControlGradient(DefaultDiscreteState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector,
                                                     DMatrixRMaj constants, DMatrixRMaj matrixToPack)
   {
      matrixToPack.reshape(controlVectorSize, stateVectorSize);
   }

   /** L_ux(X_k, U_k) */
   @Override
   public void getCostControlGradientOfStateGradient(DefaultDiscreteState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector,
                                                     DMatrixRMaj constants, DMatrixRMaj matrixToPack)
   {
      matrixToPack.reshape(stateVectorSize, controlVectorSize);
   }
}
