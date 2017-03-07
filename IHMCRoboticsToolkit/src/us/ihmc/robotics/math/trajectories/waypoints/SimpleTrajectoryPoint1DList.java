package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.OneDoFTrajectoryPointInterface;

public class SimpleTrajectoryPoint1DList extends SimpleTrajectoryPointList<SimpleTrajectoryPoint1D>
{
   public SimpleTrajectoryPoint1DList()
   {
      super(SimpleTrajectoryPoint1D.class);
   }

   public SimpleTrajectoryPoint1DList(int initialCapacity)
   {
      super(SimpleTrajectoryPoint1D.class, initialCapacity);
   }

   public void set(SimpleTrajectoryPoint1DList trajectory)
   {
      clear();
      for (int i = 0; i < trajectory.getNumberOfTrajectoryPoints(); i++)
      {
         addTrajectoryPoint(trajectory.trajectoryPoints.get(i));
      }
   }

   public void addTrajectoryPoint(OneDoFTrajectoryPointInterface<?> trajectoryPoint)
   {
      trajectoryPoints.add().set(trajectoryPoint);
   }

   public void addTrajectoryPoint(double time, double position, double velocity)
   {
      trajectoryPoints.add().set(time, position, velocity);
   }

   public RecyclingArrayList<? extends OneDoFTrajectoryPointInterface<?>> getTrajectoryPoints()
   {
      return trajectoryPoints;
   }

   @Override
   public String toString()
   {
      if (trajectoryPoints != null)
         return "Simple trajectory 1D: number of trajectory points 1D = " + getNumberOfTrajectoryPoints() + ".";
      else
         return "Simple trajectory 1D: no trajectory point 1D.";
   }

}
