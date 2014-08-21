package us.ihmc.utilities.ros;

import javax.media.j3d.Transform3D;

public interface RosTfPublisherInterface
{
   public void publish(Transform3D transform3d, long timeStamp, String parentFrame, String childFrame);
}
