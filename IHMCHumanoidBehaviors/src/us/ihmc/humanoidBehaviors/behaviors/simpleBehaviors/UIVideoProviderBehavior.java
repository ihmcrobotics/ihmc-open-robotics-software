package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.producers.CompressedVideoDataFactory;
import us.ihmc.communication.producers.CompressedVideoDataServer;
import us.ihmc.communication.producers.CompressedVideoHandler;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.robotics.robotSide.RobotSide;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.awt.image.BufferedImage;

public abstract class UIVideoProviderBehavior extends VideoPacketListenerBehavior
{
   private final CompressedVideoDataServer videoDataServer;
   private final PacketDestination videoPacketDestination;

   public UIVideoProviderBehavior(String namePrefix, final BehaviorCommunicationBridge communicationBridge, PacketDestination videoPacketDestination)
   {
      super(namePrefix, communicationBridge);

      if(videoPacketDestination.equals(PacketDestination.BEHAVIOR_MODULE))
         throw new RuntimeException("Cannot send send packets FROM Behavior module TO Behavior module");
      else
         this.videoPacketDestination = videoPacketDestination;

      videoDataServer = CompressedVideoDataFactory.createCompressedVideoDataServer(new UIVideoHandler());
   }

   public abstract void processImage(BufferedImage bufferedImageToPack, Point3d cameraPositionToPack, Quat4d cameraOrientationToPack, IntrinsicParameters intrinsicParametersToPack);

   @Override
   public void onNewImage(BufferedImage bufferedImage, Point3d cameraPosition, Quat4d cameraOrientation, IntrinsicParameters intrinsicParameters)
   {
      processImage(bufferedImage, cameraPosition, cameraOrientation, intrinsicParameters);

      videoDataServer.updateImage(null, bufferedImage, 0, cameraPosition, cameraOrientation, intrinsicParameters);
   }

   class UIVideoHandler implements CompressedVideoHandler
   {
      @Override
      public void newVideoPacketAvailable(RobotSide robotSide, long timeStamp, byte[] data, Point3d position, Quat4d orientation,
            IntrinsicParameters intrinsicParameters)
      {
         VideoPacket videoPacket = new VideoPacket(robotSide, timeStamp, data, position, orientation, intrinsicParameters);
         videoPacket.setDestination(videoPacketDestination);

         if(videoPacketDestination.equals(PacketDestination.CONTROLLER))
            sendPacketToController(videoPacket);
         else
            sendPacketToNetworkProcessor(videoPacket);
      }

      @Override
      public void addNetStateListener(NetStateListener compressedVideoDataServer)
      {
      }

      @Override
      public boolean isConnected()
      {
         return true;
      }
   }
}
