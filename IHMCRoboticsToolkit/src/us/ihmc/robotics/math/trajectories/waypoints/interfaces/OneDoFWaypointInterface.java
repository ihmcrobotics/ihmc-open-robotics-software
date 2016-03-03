package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.robotics.geometry.interfaces.GeometryObject;

public interface OneDoFWaypointInterface<T extends OneDoFWaypointInterface<T>> extends GeometryObject<T>
{
   public abstract void setPosition(double position);

   public abstract void setVelocity(double velocity);

   public abstract double getPosition();

   public abstract double getVelocity();
}
