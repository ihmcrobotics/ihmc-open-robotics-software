package us.ihmc.darpaRoboticsChallenge.visualSensorProcessor.camera;

import java.awt.image.BufferedImage;

import us.ihmc.darpaRoboticsChallenge.visualSensorProcessor.messages.controller.RobotPoseData;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.net.ObjectConsumer;

public abstract class CameraDataReceiver implements ObjectConsumer<RobotPoseData>
{
   
   public CameraDataReceiver(ObjectCommunicator fieldComputerCommunicator)
   {
      fieldComputerCommunicator.attachListener(RobotPoseData.class, this);
   }

   public void updateImage(BufferedImage bufferedImage, int timeStamp)
   {
   }

   public void consumeObject(RobotPoseData object)
   {
      
   }
}
