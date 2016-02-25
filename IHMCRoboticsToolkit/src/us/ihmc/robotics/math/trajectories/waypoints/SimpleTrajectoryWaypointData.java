package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.robotics.lists.RecyclingArrayList;

public class SimpleTrajectoryWaypointData<W extends WaypointInterface<W>> implements TrajectoryWaypointDataInterface<SimpleTrajectoryWaypointData<W>, W>
{
   protected final RecyclingArrayList<W> waypoints;

   public SimpleTrajectoryWaypointData(Class<W> waypointClass)
   {
      waypoints = new RecyclingArrayList<>(waypointClass);
   }

   @Override
   public void clear()
   {
      waypoints.clear();
   }

   @Override
   public void addWaypoint(W waypoint)
   {
      waypoints.add().set(waypoint);
   }

   @Override
   public void set(SimpleTrajectoryWaypointData<W> trajectory)
   {
      clear();
      for (int i = 0; i < trajectory.getNumberOfWaypoints(); i++)
         addWaypoint(trajectory.getWaypoint(i));
   }

   @Override
   public int getNumberOfWaypoints()
   {
      return waypoints.size();
   }

   @Override
   public W getWaypoint(int waypointIndex)
   {
      return waypoints.get(waypointIndex);
   }

   @Override
   public W getLastWaypoint()
   {
      return waypoints.getLast();
   }

   @Override
   public double getTrajectoryTime()
   {
      return getLastWaypoint().getTime();
   }

   @Override
   public boolean epsilonEquals(SimpleTrajectoryWaypointData<W> other, double epsilon)
   {
      if (getNumberOfWaypoints() != other.getNumberOfWaypoints())
         return false;
      for (int i = 0; i < getNumberOfWaypoints(); i++)
      {
         W thisWaypoint = getWaypoint(i);
         W otherWaypoint = other.getWaypoint(i);
         if (!thisWaypoint.epsilonEquals(otherWaypoint, epsilon))
            return false;
      }
      return true;
   }
}
