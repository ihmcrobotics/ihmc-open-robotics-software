package us.ihmc.communication.subscribers;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public interface DetectedObjectListener
{
   public abstract void updatePose(RigidBodyTransform pose, int id);
}
