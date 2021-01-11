package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DMatrixRMaj;

public interface S1Function
{
   void compute(double timeInState, DMatrixRMaj S1ToPack);
}
