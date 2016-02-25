package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.robotics.geometry.ReferenceFrameHolder;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameTrajectoryWaypointData<T extends FrameTrajectoryWaypointData<T, W>, W extends FrameWaypoint<W>> extends ReferenceFrameHolder
      implements TrajectoryWaypointDataInterface<T, W>
{
   protected ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
   protected final RecyclingArrayList<W> waypoints;

   public FrameTrajectoryWaypointData(Class<W> waypointClass)
   {
      waypoints = new RecyclingArrayList<>(waypointClass);
   }

   @Override
   public void clear()
   {
      waypoints.clear();
   }

   public void clear(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      waypoints.clear();
   }

   @Override
   public void addWaypoint(W waypoint)
   {
      W newWaypoint = addAndInitializeWaypoint();
      newWaypoint.set(waypoint);
   }

   public void addWaypointAndMatchFrame(W waypoint)
   {
      W newWaypoint = addAndInitializeWaypoint();
      newWaypoint.setIncludingFrame(waypoint);
      newWaypoint.changeFrame(referenceFrame);
   }

   @Override
   public void set(T other)
   {
      checkReferenceFrameMatch(other);
      clear();
      for (int i = 0; i < other.getNumberOfWaypoints(); i++)
      {
         W newWaypoint = addAndInitializeWaypoint();
         newWaypoint.set(other.waypoints.get(i));
      }
   }

   public void setIncludingFrame(T other)
   {
      clear(other.referenceFrame);
      for (int i = 0; i < other.getNumberOfWaypoints(); i++)
      {
         W newWaypoint = addAndInitializeWaypoint();
         // Here we don't want to do setIncludingFrame() in case there is inconsistency in other.
         newWaypoint.set(other.waypoints.get(i));
      }
   }

   protected W addAndInitializeWaypoint()
   {
      W newWaypoint = waypoints.add();
      newWaypoint.setToZero(referenceFrame);
      return newWaypoint;
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
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public boolean epsilonEquals(T other, double epsilon)
   {
      if (getNumberOfWaypoints() != other.getNumberOfWaypoints())
         return false;
      if (referenceFrame != other.referenceFrame)
         return false;
      for (int i = 0; i < getNumberOfWaypoints(); i++)
      {
         W thisWaypoint = waypoints.get(i);
         W otherWaypoint = other.waypoints.get(i);
         if (!thisWaypoint.epsilonEquals(otherWaypoint, epsilon))
            return false;
      }
      return true;
   }
}
