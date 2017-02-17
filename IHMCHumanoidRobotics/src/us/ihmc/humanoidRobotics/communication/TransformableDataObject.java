package us.ihmc.humanoidRobotics.communication;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.EpsilonComparable;

public interface TransformableDataObject<T> extends EpsilonComparable<T>
{
	public T transform(RigidBodyTransform transform);
}

