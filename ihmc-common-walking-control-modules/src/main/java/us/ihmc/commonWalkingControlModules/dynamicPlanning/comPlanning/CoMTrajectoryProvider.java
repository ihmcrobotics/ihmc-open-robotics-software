package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.robotics.math.trajectories.Trajectory3D;

import java.util.List;

public interface CoMTrajectoryProvider extends CoMTrajectoryPlannerInterface
{
   List<Trajectory3D> getVRPTrajectories();

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
