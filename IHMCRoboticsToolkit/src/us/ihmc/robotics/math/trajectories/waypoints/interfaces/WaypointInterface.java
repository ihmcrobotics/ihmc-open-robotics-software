package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.robotics.EpsilonComparable;

public interface WaypointInterface<T extends WaypointInterface<T>> extends EpsilonComparable<T>
{
   public abstract void set(T other);

   public abstract void setToZero();

   public abstract void setToNaN();

   public abstract boolean containsNaN();
}
