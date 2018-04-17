package us.ihmc.trajectoryOptimization;

import org.ejml.data.DenseMatrix64F;

public interface ContinuousHybridDynamics<E extends Enum>
{
   int getStateVectorSize();
   int getControlVectorSize();

   /** f */
   void getDynamics(E hybridState, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F constants, DenseMatrix64F matrixToPack);
   /** f_x */
   void getDynamicsStateGradient(E hybridState, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F constants, DenseMatrix64F matrixToPack);
   /** f_u */
   void getDynamicsControlGradient(E hybridState, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F constants, DenseMatrix64F matrixToPack);
}
