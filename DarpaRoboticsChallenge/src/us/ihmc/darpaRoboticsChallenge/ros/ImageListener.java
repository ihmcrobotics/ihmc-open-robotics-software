package us.ihmc.darpaRoboticsChallenge.ros;

import us.ihmc.darpaRoboticsChallenge.ros.messages.ImageMessage;

public interface ImageListener
{
   public void receivedImage(ImageMessage image, long timestamp);
}
