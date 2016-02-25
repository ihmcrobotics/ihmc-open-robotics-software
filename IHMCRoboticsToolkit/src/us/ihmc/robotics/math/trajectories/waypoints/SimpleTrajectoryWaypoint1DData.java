package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.robotics.lists.RecyclingArrayList;

public class SimpleTrajectoryWaypoint1DData implements TrajectoryWaypointDataInterface
{
   private final RecyclingArrayList<SimpleWaypoint1D> waypoints = new RecyclingArrayList<>(15, SimpleWaypoint1D.class);

   public SimpleTrajectoryWaypoint1DData()
   {
      clear();
   }

   public void clear()
   {
      waypoints.clear();
   }

   public void addWaypoint(Waypoint1DInterface waypoint)
   {
      waypoints.add().set(waypoint);
   }

   public void addWaypoint(double time, double position, double velocity)
   {
      waypoints.add().set(time, position, velocity);
   }

   @Override
   public SimpleWaypoint1D getWaypoint(int waypointIndex)
   {
      return waypoints.get(waypointIndex);
   }

   @Override
   public int getNumberOfWaypoints()
   {
      return waypoints.size();
   }

   @Override
   public SimpleWaypoint1D getLastWaypoint()
   {
      return waypoints.getLast();
   }

   @Override
   public double getTrajectoryTime()
   {
      return getLastWaypoint().getTime();
   }

   public RecyclingArrayList<? extends Waypoint1DInterface> getWaypoints()
   {
      return waypoints;
   }
}
