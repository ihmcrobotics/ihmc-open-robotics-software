package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import java.awt.image.BufferedImage;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.producers.CompressedVideoDataFactory;
import us.ihmc.communication.producers.CompressedVideoDataServer;
import us.ihmc.communication.producers.CompressedVideoHandler;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;

public abstract class ImageProcessingBehavior extends VideoPacketListenerBehavior
{
   private final CompressedVideoDataServer videoDataServer;
   private final PacketDestination videoPacketDestination;

   public ImageProcessingBehavior(String namePrefix, final CommunicationBridge communicationBridge, PacketDestination videoPacketDestination)
   {
      super(namePrefix, communicationBridge);

      if(videoPacketDestination.equals(PacketDestination.BEHAVIOR_MODULE))
         throw new RuntimeException("Cannot send send packets FROM Behavior module TO Behavior module");
      else
         this.videoPacketDestination = videoPacketDestination;

      videoDataServer = CompressedVideoDataFactory.createCompressedVideoDataServer(new UIVideoHandler());
   }

   public abstract void processImageToSend(BufferedImage bufferedImageToPack, Point3d cameraPositionToPack, Quat4d cameraOrientationToPack, IntrinsicParameters intrinsicParametersToPack);

   @Override
   public void updateImage(BufferedImage bufferedImage, Point3d cameraPosition, Quat4d cameraOrientation, IntrinsicParameters intrinsicParameters)
   {
      processImageToSend(bufferedImage, cameraPosition, cameraOrientation, intrinsicParameters);

      videoDataServer.updateImage(VideoSource.IMAGE_PROCESSING_BEHAVIOR, bufferedImage, 0, cameraPosition, cameraOrientation, intrinsicParameters);
   }

   class UIVideoHandler implements CompressedVideoHandler
   {
      @Override
      public void newVideoPacketAvailable(VideoSource videoSource, long timeStamp, byte[] data, Point3d position, Quat4d orientation,
            IntrinsicParameters intrinsicParameters)
      {
         VideoPacket videoPacket = new VideoPacket(videoSource, timeStamp, data, position, orientation, intrinsicParameters, videoPacketDestination);

         if(videoPacketDestination.equals(PacketDestination.CONTROLLER))
            sendPacketToController(videoPacket);
         else
            sendPacket(videoPacket);
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
