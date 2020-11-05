package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.Trajectory3D;

import java.util.List;

public class DifferentialS1Function implements S1Function
{
   private final RecyclingArrayList<DifferentialS1Segment> s1Segments;
   private final DMatrixRMaj endValueLocal = new DMatrixRMaj(6, 6);

   public DifferentialS1Function(double dt)
   {
      s1Segments = new RecyclingArrayList<>(() -> new DifferentialS1Segment(dt));
   }

   public void set(DMatrixRMaj Q1, DMatrixRMaj R1, DMatrixRMaj NTranspose, DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj S1AtEnd, List<Trajectory3D> vrpTrajectories)
   {
      s1Segments.clear();
      for (int i = 0; i < vrpTrajectories.size(); i++)
         s1Segments.add();

      endValueLocal.set(S1AtEnd);
      for (int i = vrpTrajectories.size() - 1; i >= 0; i--)
      {
         s1Segments.get(i).set(Q1, R1, NTranspose, A, B, endValueLocal, vrpTrajectories.get(i).getDuration());
         s1Segments.get(i).compute(0, endValueLocal);
      }
   }

   @Override
   public void compute(double timeInState, DMatrixRMaj S1ToPack)
   {
      s1Segments.get(0).compute(timeInState, S1ToPack);
   }

   public DifferentialS1Segment getS1Segment(int i)
   {
      return s1Segments.get(i);
   }

}
