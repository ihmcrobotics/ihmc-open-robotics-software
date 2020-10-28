package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.Trajectory3D;

import java.util.List;

public class DifferentialS2Function implements S2Function
{
   private final DMatrixRMaj endValueLocal = new DMatrixRMaj(6, 1);
   private final RecyclingArrayList<DifferentialS2Segment> s2Segments;

   public DifferentialS2Function(double dt)
   {
      s2Segments = new RecyclingArrayList<>(() -> new DifferentialS2Segment(dt));
   }

   public void set(DifferentialS1Function s1Function,
                   List<Trajectory3D> vrpTrajectories,
                   DMatrixRMaj Q,
                   DMatrixRMaj q2,
                   DMatrixRMaj R1,
                   DMatrixRMaj NTranspose,
                   DMatrixRMaj A,
                   DMatrixRMaj B,
                   DMatrixRMaj D,
                   DMatrixRMaj s2AtEnd)
   {
      int k = vrpTrajectories.size();
      s2Segments.clear();
      for (int i = 0; i < k; i++)
         s2Segments.add();

      endValueLocal.set(s2AtEnd);
      for (int i = k; i >= 0; i--)
      {
         DifferentialS1Segment s1Segment = s1Function.getS1Segment(i);
         DifferentialS2Segment s2Segment = s2Segments.get(i);
         s2Segment.set(s1Segment, vrpTrajectories.get(i), Q, q2, R1, NTranspose, A, B, D, endValueLocal);
         s2Segment.compute(0, endValueLocal);
      }
   }

   @Override
   public void compute(int segmentNumber, double timeInSegment, DMatrixRMaj s2ToPack)
   {
      s2Segments.get(segmentNumber).compute(timeInSegment, s2ToPack);
   }
}
