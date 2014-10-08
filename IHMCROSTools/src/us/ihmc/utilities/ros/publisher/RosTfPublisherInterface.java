package us.ihmc.utilities.ros.publisher;

import us.ihmc.utilities.math.geometry.RigidBodyTransform;

public interface RosTfPublisherInterface
{
   public void publish(RigidBodyTransform transform3d, long timeStamp, String parentFrame, String childFrame);
}
