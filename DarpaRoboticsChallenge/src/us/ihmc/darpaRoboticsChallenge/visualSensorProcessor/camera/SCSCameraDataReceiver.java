package us.ihmc.darpaRoboticsChallenge.visualSensorProcessor.camera;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;

import us.ihmc.darpaRoboticsChallenge.visualSensorProcessor.state.RobotPoseBuffer;
import us.ihmc.graphics3DAdapter.camera.VideoPacket;
import us.ihmc.graphics3DAdapter.camera.VideoSettings;
import us.ihmc.utilities.net.KryoObjectClient;
import us.ihmc.utilities.net.KryoObjectServer;
import us.ihmc.utilities.net.ObjectConsumer;

public class SCSCameraDataReceiver extends CameraDataReceiver implements ObjectConsumer<VideoPacket> 
{
   private final BufferedImage image;

   
   public SCSCameraDataReceiver(RobotPoseBuffer robotPoseBuffer, VideoSettings videoSettings, KryoObjectClient fieldComputerClient, KryoObjectServer teamServer)
   {
      super(robotPoseBuffer, videoSettings, teamServer);
      this.image = new BufferedImage(videoSettings.getWidth(), videoSettings.getHeight(), BufferedImage.TYPE_3BYTE_BGR);
      
      fieldComputerClient.attachListener(VideoPacket.class, this);
   }

   public void consumeObject(VideoPacket object)
   {
      byte[] data = object.getData();
      
      DataBufferByte databuffer = ((DataBufferByte)image.getRaster().getDataBuffer());
      System.arraycopy(data, 0, databuffer.getData(), 0, data.length);
      
      updateImage(image, object.getTimeStamp(), object.getFieldOfView());
   }

}

