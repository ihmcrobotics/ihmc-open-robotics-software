package us.ihmc.humanoidRobotics.communication.producers;

import java.awt.image.BufferedImage;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.producers.VideoDataServer;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.LocalVideoPacket;

public class RawVideoDataServer implements VideoDataServer
{
   protected final ObjectCommunicator objectCommunicator;
   
   public RawVideoDataServer(ObjectCommunicator objectCommunicator)
   {
      this.objectCommunicator = objectCommunicator;
   }
   
   @Override
   public void updateImage(VideoSource videoSource, BufferedImage bufferedImage, long timeStamp, Point3D cameraPosition, Quaternion cameraOrientation, IntrinsicParameters intrinsicParameters)
   {
      LocalVideoPacket videoPacket = new LocalVideoPacket(timeStamp, bufferedImage, intrinsicParameters);
      objectCommunicator.consumeObject(videoPacket);
   }

   @Override
   public void close()
   {
   }

   @Override
   public boolean isConnected()
   {
      return objectCommunicator.isConnected();
   }
}
