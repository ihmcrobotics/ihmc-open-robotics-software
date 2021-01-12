package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.robotics.math.trajectories.core.Polynomial3D;
import us.ihmc.robotics.math.trajectories.Trajectory3DReadOnly;

import java.util.List;

public interface CoMTrajectoryProvider extends CoMTrajectoryPlannerInterface
{
   List<Polynomial3D> getVRPTrajectories();

   List<? extends Trajectory3DReadOnly> getCoMTrajectories();

   @Override
   default int getSegmentNumber(double time)
   {
      double startTime = 0.0;
      for (int i = 0; i < getVRPTrajectories().size(); i++)
      {
         if (getVRPTrajectories().get(i).timeIntervalContains(time - startTime))
            return i;

         startTime += getVRPTrajectories().get(i).getDuration();
      }

      return -1;
   }

   @Override
   default double getTimeInSegment(int segmentNumber, double time)
   {
      for (int i = 0; i < segmentNumber; i++)
         time -= getVRPTrajectories().get(i).getDuration();

      return time;
   }

}
