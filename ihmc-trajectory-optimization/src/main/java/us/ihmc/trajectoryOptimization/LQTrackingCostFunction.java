package us.ihmc.trajectoryOptimization;

import org.ejml.data.DMatrixRMaj;

public interface LQTrackingCostFunction<E extends Enum>
{
   /** L(X_k, U_k) */
   double getCost(E state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj desiredControlVector, DMatrixRMaj desiredStateVector, DMatrixRMaj constants);

   /** L_x(X_k, U_k) */
   void getCostStateGradient(E state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj desiredControlVector, DMatrixRMaj desiredStateVector, DMatrixRMaj constants, DMatrixRMaj matrixToPack);
   /** L_u(X_k, U_k) */
   void getCostControlGradient(E state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj desiredControlVector, DMatrixRMaj desiredStateVector, DMatrixRMaj constants, DMatrixRMaj matrixToPack);

   /** L_xx(X_k, U_k) */
   void getCostStateHessian(E state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj constants, DMatrixRMaj matrixToPack);
   /** L_uu(X_k, U_k) */
   void getCostControlHessian(E state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj constants, DMatrixRMaj matrixToPack);
   /** L_ux(X_k, U_k) */
   void getCostStateGradientOfControlGradient(E state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj constants, DMatrixRMaj matrixToPack);
   /** L_xu(X_k, U_k) */
   void getCostControlGradientOfStateGradient(E state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj constants, DMatrixRMaj matrixToPack);
}
