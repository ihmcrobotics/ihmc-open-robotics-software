package us.ihmc.avatar.ros;

import us.ihmc.avatar.ros.messages.ImageMessage;

public interface ImageListener
{
   public void receivedImage(ImageMessage image, long timestamp);
}
