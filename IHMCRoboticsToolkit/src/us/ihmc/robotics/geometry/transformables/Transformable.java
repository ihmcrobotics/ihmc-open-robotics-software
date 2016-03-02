package us.ihmc.robotics.geometry.transformables;

import us.ihmc.robotics.Settable;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public interface Transformable<T> extends Settable<T>
{
   public abstract void applyTransform(RigidBodyTransform transform);
}
