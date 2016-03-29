package us.ihmc.robotics.geometry.transformables;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public interface Transformable
{
   public abstract void applyTransform(RigidBodyTransform transform);
}
