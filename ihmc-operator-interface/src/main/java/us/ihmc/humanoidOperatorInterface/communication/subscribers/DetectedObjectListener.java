package us.ihmc.humanoidOperatorInterface.communication.subscribers;

import us.ihmc.euclid.transform.RigidBodyTransform;

public interface DetectedObjectListener
{
   public abstract void updatePose(RigidBodyTransform pose, int id);
}
