package us.ihmc.utilities.ros;

import us.ihmc.utilities.math.geometry.Transform3d;

public interface RosTfPublisherInterface
{
   public void publish(Transform3d transform3d, long timeStamp, String parentFrame, String childFrame);
}
