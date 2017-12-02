package us.ihmc.commonWalkingControlModules.dynamicPlanning;

import org.ejml.data.DenseMatrix64F;

public interface DiscreteHybridDynamics<E extends Enum>
{
   void setTimeStepSize(double deltaT);
   double getStateVectorSize();
   double getControlVectorSize();

   /** f */
   void getDynamics(E hybridState, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F matrixToPack);
   /** f_x */
   void getDynamicsStateGradient(E hybridState, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F matrixToPack);
   /** f_u */
   void getDynamicsControlGradient(E hybridState, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F matrixToPack);
   /** f_xx */
   void getDynamicsStateHessian(E hybridState, int stateVariable, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F matrixToPack);
   /** f_uu */
   void getDynamicsControlHessian(E hybridState, int controlVariable, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F matrixToPack);
   /** f_ux */
   void getDynamicsStateGradientOfControlGradient(E hybridState, int stateVariable, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F matrixToPack);
   /** f_xu */
   void getDynamicsControlGradientOfStateGradient(E hybridState, int controlVariable, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F matrixToPack);
}
