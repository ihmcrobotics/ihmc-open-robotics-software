package us.ihmc.trajectoryOptimization;

import org.ejml.data.DenseMatrix64F;

public interface DiscreteHybridDynamics<E extends Enum>
{
   void setTimeStepSize(double deltaT);
   int getStateVectorSize();
   int getControlVectorSize();
   int getConstantVectorSize();

   /** f */
   void getNextState(E hybridState, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F constants, DenseMatrix64F matrixToPack);
   /** f_x */
   void getDynamicsStateGradient(E hybridState, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F constants, DenseMatrix64F matrixToPack);
   /** f_u */
   void getDynamicsControlGradient(E hybridState, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F constants, DenseMatrix64F matrixToPack);
   /** f_xx */
   void getDynamicsStateHessian(E hybridState, int stateVariable, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F constants, DenseMatrix64F matrixToPack);
   /** f_uu */
   void getDynamicsControlHessian(E hybridState, int controlVariable, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F constants, DenseMatrix64F matrixToPack);
   /** f_ux */
   void getDynamicsStateGradientOfControlGradient(E hybridState, int stateVariable, DenseMatrix64F currentState, DenseMatrix64F currentControl,
         DenseMatrix64F constants, DenseMatrix64F matrixToPack);
   /** f_xu */
   void getDynamicsControlGradientOfStateGradient(E hybridState, int controlVariable, DenseMatrix64F currentState, DenseMatrix64F currentControl,
         DenseMatrix64F constants, DenseMatrix64F matrixToPack);
   void getContinuousAMatrix(DenseMatrix64F A);
   void getContinuousBMatrix(DenseMatrix64F A);
}
