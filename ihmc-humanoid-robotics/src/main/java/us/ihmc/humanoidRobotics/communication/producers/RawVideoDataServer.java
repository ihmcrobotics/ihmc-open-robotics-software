package us.ihmc.humanoidRobotics.communication.producers;

import java.awt.image.BufferedImage;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.producers.VideoDataServer;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.LocalVideoPacket;

public class RawVideoDataServer implements VideoDataServer
{
   protected final ObjectCommunicator objectCommunicator;
   
   public RawVideoDataServer(ObjectCommunicator objectCommunicator)
   {
      this.objectCommunicator = objectCommunicator;
   }
   
   @Override
   public void onFrame(VideoSource videoSource, BufferedImage bufferedImage, long timeStamp, Point3DReadOnly cameraPosition, QuaternionReadOnly cameraOrientation, IntrinsicParameters intrinsicParameters)
   {
      LocalVideoPacket videoPacket = HumanoidMessageTools.createLocalVideoPacket(timeStamp, bufferedImage, intrinsicParameters);
      objectCommunicator.consumeObject(videoPacket);
   }

   @Override
   public boolean isConnected()
   {
      return objectCommunicator.isConnected();
   }
}
