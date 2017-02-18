package us.ihmc.humanoidRobotics.communication;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.transform.RigidBodyTransform;

public interface TransformableDataObject<T> extends EpsilonComparable<T>
{
	public T transform(RigidBodyTransform transform);
}

