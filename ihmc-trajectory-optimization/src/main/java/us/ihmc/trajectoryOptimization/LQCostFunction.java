package us.ihmc.trajectoryOptimization;

import org.ejml.data.DenseMatrix64F;

public interface LQCostFunction
{
   /** L(X_k, U_k) */
   double getCost(DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVector, DenseMatrix64F desiredStateVector);

   /** L_x(X_k, U_k) */
   void getCostStateGradient(DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVector, DenseMatrix64F desiredStateVector, DenseMatrix64F matrixToPack);
   /** L_u(X_k, U_k) */
   void getCostControlGradient(DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVector, DenseMatrix64F desiredStateVector, DenseMatrix64F matrixToPack);

   /** L_xx(X_k, U_k) */
   void getCostStateHessian(DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack);
   /** L_uu(X_k, U_k) */
   void getCostControlHessian(DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack);
   /** L_ux(X_k, U_k) */
   void getCostStateGradientOfControlGradient(DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack);
   /** L_xu(X_k, U_k) */
   void getCostControlGradientOfStateGradient(DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack);
}
