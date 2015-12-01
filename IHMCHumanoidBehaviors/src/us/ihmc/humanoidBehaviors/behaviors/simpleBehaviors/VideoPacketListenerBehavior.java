package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.producers.CompressedVideoDataClient;
import us.ihmc.communication.producers.CompressedVideoDataFactory;
import us.ihmc.communication.producers.VideoStreamer;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.awt.image.BufferedImage;

public abstract class VideoPacketListenerBehavior extends BehaviorInterface
{
   private final ConcurrentListeningQueue<VideoPacket> cameraData = new ConcurrentListeningQueue<>();

   private final CompressedVideoDataClient videoDataClient;

   public VideoPacketListenerBehavior(String namePrefix, BehaviorCommunicationBridge communicationBridge)
   {
      super(namePrefix, communicationBridge);

      videoDataClient = CompressedVideoDataFactory.createCompressedVideoDataClient(new VideoStreamer()
      {
         @Override
         public void updateImage(BufferedImage bufferedImage, Point3d cameraPosition, Quat4d cameraOrientation, IntrinsicParameters intrinsicParamaters)
         {
            onNewImage(bufferedImage, cameraPosition, cameraOrientation, intrinsicParamaters);
         }
      });

      communicationBridge.attachGlobalListener(getNetworkProcessorGlobalObjectConsumer());
      attachNetworkProcessorListeningQueue(cameraData, VideoPacket.class);
   }

   public abstract void onNewImage(BufferedImage bufferedImage, Point3d cameraPosition, Quat4d cameraOrientation, IntrinsicParameters intrinsicParamaters);

   @Override
   public void doControl()
   {
      if (cameraData.isNewPacketAvailable())
      {
         VideoPacket packet = cameraData.getNewestPacket();
         videoDataClient.consumeObject(packet.getData(), packet.getPosition(), packet.getOrientation(), packet.getIntrinsicParameters());
      }
   }

}
