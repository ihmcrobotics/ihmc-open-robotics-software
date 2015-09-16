package us.ihmc.humanoidRobotics.communication.producers;

import java.awt.image.BufferedImage;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.producers.VideoDataServer;
import us.ihmc.humanoidRobotics.communication.packets.LocalVideoPacket;
import us.ihmc.robotics.robotSide.RobotSide;
import boofcv.struct.calib.IntrinsicParameters;

public class RawVideoDataServer implements VideoDataServer
{
   private final ObjectCommunicator objectCommunicator;
   
   public RawVideoDataServer(ObjectCommunicator objectCommunicator)
   {
      this.objectCommunicator = objectCommunicator;
   }
   
   public void updateImage(RobotSide robotSide, BufferedImage bufferedImage, long timeStamp, Point3d cameraPosition, Quat4d cameraOrientation, IntrinsicParameters intrinsicParameters)
   {
      
      LocalVideoPacket videoPacket = new LocalVideoPacket(timeStamp, bufferedImage, intrinsicParameters);
      objectCommunicator.consumeObject(videoPacket);
   }

   public void close()
   {
   }

   public boolean isConnected()
   {
      return objectCommunicator.isConnected();
   }

}
