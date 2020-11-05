package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DMatrixRMaj;

public interface S2Function extends S2Segment
{
   default void compute(double timeInSegment, DMatrixRMaj s2ToPack)
   {
      compute(0, timeInSegment, s2ToPack);
   }

   void compute(int segmentNumber, double timeInSegment, DMatrixRMaj s2ToPack);
}
