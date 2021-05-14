package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DMatrixRMaj;

public interface S2Segment
{
   void compute(double timeInSegment, DMatrixRMaj s2ToPack);
}
