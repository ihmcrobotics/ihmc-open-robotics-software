package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.robotics.geometry.ReferenceFrameHolder;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class FrameTrajectoryPoint<T extends FrameTrajectoryPoint<T>> extends ReferenceFrameHolder implements TrajectoryPointInterface<T>
{
   @Override
   public abstract void set(T other);

   public abstract void setIncludingFrame(T other);

   public abstract void changeFrame(ReferenceFrame referenceFrame);

   public abstract void setToZero();

   public abstract void setToZero(ReferenceFrame referenceFrame);

   public abstract void setToNaN();

   public abstract void setToNaN(ReferenceFrame referenceFrame);

   public abstract boolean containsNaN();

   protected void throwFrameInconsistencyException()
   {
      throw new RuntimeException("The reference frames in the " + getClass().getSimpleName() + " are inconsistent.");
   }
}
