package us.ihmc.commonWalkingControlModules.dynamicPlanning.lipm;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.lipm.LIPMDynamics.controlVectorSize;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.lipm.LIPMDynamics.stateVectorSize;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.matrixlib.DiagonalMatrixTools;
import us.ihmc.trajectoryOptimization.DefaultDiscreteState;
import us.ihmc.trajectoryOptimization.LQTrackingCostFunction;

public class LIPMSimpleCostFunction implements LQTrackingCostFunction<DefaultDiscreteState>
{
   static final double qX = 1e-6;
   static final double qY = 1e-6;
   static final double qZ = 1e1;
   static final double qXDot = 1e-6;
   static final double qYDot = 1e-6;
   static final double qZDot = 1e-6;

   static final double rXf = 1e2;
   static final double rYf = 1e2;
   static final double rFz = 1e-6;

   private final DMatrixRMaj Q = new DMatrixRMaj(stateVectorSize, stateVectorSize);
   private final DMatrixRMaj R = new DMatrixRMaj(controlVectorSize, controlVectorSize);

   /**
    * This is a cost of the form 0.5 (X - X_d)^T Q (X - X_d) + 0.5 (U - U_d)^T R (U - U_d)
    */

   public LIPMSimpleCostFunction()
   {
      Q.set(0, 0, qX);
      Q.set(1, 1, qY);
      Q.set(2, 2, qZ);
      Q.set(3, 3, qXDot);
      Q.set(4, 4, qYDot);
      Q.set(5, 5, qZDot);

      R.set(0, 0, rXf);
      R.set(1, 1, rYf);
      R.set(2, 2, rFz);
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
      CommonOps_DDRM.scale(2.0, matrixToPack);
   }

   /** L_u(X_k, U_k) */
   @Override
   public void getCostControlGradient(DefaultDiscreteState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj desiredControlVector,
                                      DMatrixRMaj desiredStateVector, DMatrixRMaj constants, DMatrixRMaj matrixToPack)
   {
      CommonOps_DDRM.subtract(controlVector, desiredControlVector, tempControlMatrix);
      DiagonalMatrixTools.preMult(R, tempControlMatrix, matrixToPack);
      CommonOps_DDRM.scale(2.0, matrixToPack);
   }

   /** L_xx(X_k, U_k) */
   @Override
   public void getCostStateHessian(DefaultDiscreteState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj constants, DMatrixRMaj matrixToPack)
   {
      CommonOps_DDRM.scale(2.0, Q, matrixToPack);
   }

   /** L_uu(X_k, U_k) */
   @Override
   public void getCostControlHessian(DefaultDiscreteState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj constants, DMatrixRMaj matrixToPack)
   {
      CommonOps_DDRM.scale(2.0, R, matrixToPack);
   }

   /** L_ux(X_k, U_k) */
   @Override
   public void getCostStateGradientOfControlGradient(DefaultDiscreteState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector,
                                                     DMatrixRMaj constants, DMatrixRMaj matrixToPack)
   {
      matrixToPack.reshape(controlVectorSize, stateVectorSize);
   }

   /** L_xu(X_k, U_k) */
   @Override
   public void getCostControlGradientOfStateGradient(DefaultDiscreteState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector,
                                                     DMatrixRMaj constants, DMatrixRMaj matrixToPack)
   {
      matrixToPack.reshape(stateVectorSize, controlVectorSize);
   }
}
