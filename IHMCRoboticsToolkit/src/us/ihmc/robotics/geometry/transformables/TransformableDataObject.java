package us.ihmc.robotics.geometry.transformables;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public interface TransformableDataObject
{
   public abstract void transform(RigidBodyTransform transform);
}
