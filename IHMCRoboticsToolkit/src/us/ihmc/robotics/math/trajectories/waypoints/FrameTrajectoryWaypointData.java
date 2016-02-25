package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.robotics.geometry.ReferenceFrameHolder;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameTrajectoryWaypointData<T extends FrameWaypoint<T>> extends ReferenceFrameHolder implements TrajectoryWaypointDataInterface<T>
{
   protected ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
   protected final RecyclingArrayList<T> waypoints;

   public FrameTrajectoryWaypointData(Class<T> waypointClass)
   {
      waypoints = new RecyclingArrayList<>(waypointClass);
   }

   public void clear()
   {
      waypoints.clear();
   }

   public void clear(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      waypoints.clear();
   }

   public void addWaypoint(T waypoint)
   {
      T newWaypoint = waypoints.add();
      newWaypoint.setToZero(referenceFrame);
      newWaypoint.set(waypoint);
   }

   public void addWaypointAndMatchFrame(T waypoint)
   {
      T newWaypoint = waypoints.add();
      newWaypoint.setIncludingFrame(waypoint);
      newWaypoint.changeFrame(referenceFrame);
   }

   public void set(FrameTrajectoryWaypointData<T> other)
   {
      checkReferenceFrameMatch(other);
      clear();
      for (int i = 0; i < other.getNumberOfWaypoints(); i++)
      {
         T newWaypoint = waypoints.add();
         newWaypoint.setToZero(referenceFrame);
         newWaypoint.set(other.waypoints.get(i));
      }
   }

   public void setIncludingFrame(FrameTrajectoryWaypointData<T> other)
   {
      clear(other.referenceFrame);
      for (int i = 0; i < other.getNumberOfWaypoints(); i++)
      {
         T newWaypoint = waypoints.add();
         newWaypoint.setToZero(referenceFrame);
         // Here we don't want to do setIncludingFrame() in case there is inconsistency in other.
         newWaypoint.set(other.waypoints.get(i));
      }
   }

   public void changeFrame(ReferenceFrame referenceFrame)
   {
      if (this.referenceFrame == referenceFrame)
         return;

      for (int i = 0; i < waypoints.size(); i++)
         waypoints.get(i).changeFrame(referenceFrame);
      this.referenceFrame = referenceFrame;
   }

   @Override
   public int getNumberOfWaypoints()
   {
      return waypoints.size();
   }

   @Override
   public T getWaypoint(int waypointIndex)
   {
      return waypoints.get(waypointIndex);
   }

   @Override
   public T getLastWaypoint()
   {
      return waypoints.getLast();
   }

   @Override
   public double getTrajectoryTime()
   {
      return getLastWaypoint().getTime();
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }
}
