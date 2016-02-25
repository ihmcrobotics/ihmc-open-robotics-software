package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.robotics.geometry.ReferenceFrameHolder;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class FrameWaypoint<T extends WaypointInterface> extends ReferenceFrameHolder implements WaypointInterface
{
   public abstract void set(T waypoint);
   public abstract void setIncludingFrame(T waypoint);
   public abstract void changeFrame(ReferenceFrame referenceFrame);
   public abstract void setToZero();
   public abstract void setToZero(ReferenceFrame referenceFrame);
   public abstract void setToNaN();
   public abstract void setToNaN(ReferenceFrame referenceFrame);

   protected void throwFrameInconsistencyException()
   {
      throw new RuntimeException("The reference frames in the " + getClass().getSimpleName() + " are inconsistent.");
   }
}
