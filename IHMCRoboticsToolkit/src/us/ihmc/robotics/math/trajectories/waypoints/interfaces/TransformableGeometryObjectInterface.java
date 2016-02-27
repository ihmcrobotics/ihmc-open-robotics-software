package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public interface TransformableGeometryObjectInterface
{
   public abstract void applyTransform(RigidBodyTransform transform);
}
