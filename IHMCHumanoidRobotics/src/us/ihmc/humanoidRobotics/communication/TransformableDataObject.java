package us.ihmc.humanoidRobotics.communication;

import us.ihmc.robotics.EpsilonComparable;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public interface TransformableDataObject<T> extends EpsilonComparable<T>
{
	public T transform(RigidBodyTransform transform);
}

