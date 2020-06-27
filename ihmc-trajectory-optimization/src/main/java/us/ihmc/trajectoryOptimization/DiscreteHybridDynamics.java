package us.ihmc.trajectoryOptimization;

import org.ejml.data.DMatrixRMaj;

public interface DiscreteHybridDynamics<E extends Enum>
{
   void setTimeStepSize(double deltaT);
   int getStateVectorSize();
   int getControlVectorSize();
   int getConstantVectorSize();

   /** f */
   void getNextState(E hybridState, DMatrixRMaj currentState, DMatrixRMaj currentControl, DMatrixRMaj constants, DMatrixRMaj matrixToPack);
   /** f_x */
   void getDynamicsStateGradient(E hybridState, DMatrixRMaj currentState, DMatrixRMaj currentControl, DMatrixRMaj constants, DMatrixRMaj matrixToPack);
   /** f_u */
   void getDynamicsControlGradient(E hybridState, DMatrixRMaj currentState, DMatrixRMaj currentControl, DMatrixRMaj constants, DMatrixRMaj matrixToPack);
   /** f_xx */
   void getDynamicsStateHessian(E hybridState, int stateVariable, DMatrixRMaj currentState, DMatrixRMaj currentControl, DMatrixRMaj constants, DMatrixRMaj matrixToPack);
   /** f_uu */
   void getDynamicsControlHessian(E hybridState, int controlVariable, DMatrixRMaj currentState, DMatrixRMaj currentControl, DMatrixRMaj constants, DMatrixRMaj matrixToPack);
   /** f_ux */
   void getDynamicsStateGradientOfControlGradient(E hybridState, int stateVariable, DMatrixRMaj currentState, DMatrixRMaj currentControl,
         DMatrixRMaj constants, DMatrixRMaj matrixToPack);
   /** f_xu */
   void getDynamicsControlGradientOfStateGradient(E hybridState, int controlVariable, DMatrixRMaj currentState, DMatrixRMaj currentControl,
         DMatrixRMaj constants, DMatrixRMaj matrixToPack);
   void getContinuousAMatrix(DMatrixRMaj A);
   void getContinuousBMatrix(DMatrixRMaj A);
}
