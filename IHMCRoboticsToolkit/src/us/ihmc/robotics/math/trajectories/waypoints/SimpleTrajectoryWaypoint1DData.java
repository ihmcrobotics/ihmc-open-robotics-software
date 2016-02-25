package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.robotics.lists.RecyclingArrayList;

public class SimpleTrajectoryWaypoint1DData extends SimpleTrajectoryWaypointData<SimpleWaypoint1D>
{
   public SimpleTrajectoryWaypoint1DData()
   {
      super(SimpleWaypoint1D.class);
   }

   public void set(SimpleTrajectoryWaypoint1DData trajectory)
   {
      clear();
      for (int i = 0; i < trajectory.getNumberOfWaypoints(); i++)
      {
         addWaypoint(trajectory.waypoints.get(i));
      }
   }

   public void addWaypoint(Waypoint1DInterface<?> waypoint)
   {
      waypoints.add().set(waypoint);
   }

   public void addWaypoint(double time, double position, double velocity)
   {
      waypoints.add().set(time, position, velocity);
   }

   public RecyclingArrayList<? extends Waypoint1DInterface<?>> getWaypoints()
   {
      return waypoints;
   }
}
