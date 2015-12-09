package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import us.ihmc.communication.producers.CompressedVideoDataClient;
import us.ihmc.communication.producers.CompressedVideoDataFactory;
import us.ihmc.communication.producers.VideoStreamer;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;

public abstract class VideoPacketListenerBehavior extends BehaviorInterface implements VideoStreamer
{
   private final ConcurrentListeningQueue<VideoPacket> cameraData = new ConcurrentListeningQueue<>();

   private final CompressedVideoDataClient videoDataClient;

   public VideoPacketListenerBehavior(String namePrefix, BehaviorCommunicationBridge communicationBridge)
   {
      super(namePrefix, communicationBridge);

      videoDataClient = CompressedVideoDataFactory.createCompressedVideoDataClient(this);

      communicationBridge.attachGlobalListener(getNetworkProcessorGlobalObjectConsumer());
      attachNetworkProcessorListeningQueue(cameraData, VideoPacket.class);
   }

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
