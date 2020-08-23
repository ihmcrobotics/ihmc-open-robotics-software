package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DMatrixRMaj;

public interface S2Function
{
   void compute(double timeInState, DMatrixRMaj s2ToPack);
}
