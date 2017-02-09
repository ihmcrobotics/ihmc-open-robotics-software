package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import us.ihmc.communication.producers.CompressedVideoDataClient;
import us.ihmc.communication.producers.CompressedVideoDataFactory;
import us.ihmc.communication.producers.VideoStreamer;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;

public abstract class VideoPacketListenerBehavior extends AbstractBehavior implements VideoStreamer
{
   private final ConcurrentListeningQueue<VideoPacket> cameraData = new ConcurrentListeningQueue<>(20);

   private final CompressedVideoDataClient videoDataClient;

   public VideoPacketListenerBehavior(String namePrefix, CommunicationBridge communicationBridge)
   {
      super(namePrefix, communicationBridge);

      videoDataClient = CompressedVideoDataFactory.createCompressedVideoDataClient(this);

      attachNetworkListeningQueue(cameraData, VideoPacket.class);
   }

   @Override
   public void doControl()
   {
      if (cameraData.isNewPacketAvailable())
      {
         VideoPacket packet = cameraData.poll();
         videoDataClient.consumeObject(packet.getData(), packet.getPosition(), packet.getOrientation(), packet.getIntrinsicParameters());
      }
   }
}
