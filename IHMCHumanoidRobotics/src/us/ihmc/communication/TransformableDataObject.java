package us.ihmc.communication;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public interface TransformableDataObject <T> extends ComparableDataObject<T>
{
	public T transform(RigidBodyTransform transform);
}

