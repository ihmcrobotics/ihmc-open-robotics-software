package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.robotics.ComparableDataObject;

public interface WaypointInterface<T extends WaypointInterface<T>> extends ComparableDataObject<T>
{
   public abstract void set(T other);

   public abstract void setToZero();

   public abstract void setToNaN();

   public abstract boolean containsNaN();
}
