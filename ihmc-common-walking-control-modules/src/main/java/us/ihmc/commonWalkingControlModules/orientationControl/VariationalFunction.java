package us.ihmc.commonWalkingControlModules.orientationControl;

import org.ejml.data.DMatrixRMaj;

public interface VariationalFunction
{
   void compute(double timeInState, DMatrixRMaj PToPack, DMatrixRMaj KToPack);
}
