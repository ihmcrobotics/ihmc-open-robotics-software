package us.ihmc.trajectoryOptimization;

import org.ejml.data.DMatrixRMaj;

public interface ContinuousHybridDynamics<E extends Enum>
{
   int getStateVectorSize();
   int getControlVectorSize();

   /** f */
   void getDynamics(E hybridState, DMatrixRMaj currentState, DMatrixRMaj currentControl, DMatrixRMaj constants, DMatrixRMaj matrixToPack);
   /** f_x */
   void getDynamicsStateGradient(E hybridState, DMatrixRMaj currentState, DMatrixRMaj currentControl, DMatrixRMaj constants, DMatrixRMaj matrixToPack);
   /** f_u */
   void getDynamicsControlGradient(E hybridState, DMatrixRMaj currentState, DMatrixRMaj currentControl, DMatrixRMaj constants, DMatrixRMaj matrixToPack);
}
