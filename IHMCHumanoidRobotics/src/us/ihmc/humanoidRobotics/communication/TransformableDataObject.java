package us.ihmc.humanoidRobotics.communication;

import us.ihmc.communication.ComparableDataObject;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public interface TransformableDataObject <T> extends ComparableDataObject<T>
{
	public T transform(RigidBodyTransform transform);
}

