package us.ihmc.humanoidRobotics.communication.subscribers;

import java.util.concurrent.ConcurrentLinkedQueue;

import controller_msgs.msg.dds.LocalizationPacket;
import controller_msgs.msg.dds.PelvisPoseErrorPacket;
import controller_msgs.msg.dds.StampedPosePacket;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.RealtimeRos2Node;

public class PelvisPoseCorrectionCommunicator implements PelvisPoseCorrectionCommunicatorInterface
{
   private final ConcurrentLinkedQueue<StampedPosePacket> packetQueue = new ConcurrentLinkedQueue<StampedPosePacket>();
   private final IHMCRealtimeROS2Publisher<PelvisPoseErrorPacket> poseErrorPublisher;
   private final IHMCRealtimeROS2Publisher<LocalizationPacket> localizationPublisher;

   public PelvisPoseCorrectionCommunicator(RealtimeRos2Node realtimeRos2Node, MessageTopicNameGenerator topicNameGenerator)
   {
      if (realtimeRos2Node != null && topicNameGenerator != null)
      {
         poseErrorPublisher = ROS2Tools.createPublisher(realtimeRos2Node, PelvisPoseErrorPacket.class, topicNameGenerator);
         localizationPublisher = ROS2Tools.createPublisher(realtimeRos2Node, LocalizationPacket.class, topicNameGenerator);
      }
      else
      {
         poseErrorPublisher = null;
         localizationPublisher = null;
      }
   }

   @Override
   public void onNewDataMessage(Subscriber<StampedPosePacket> subscriber)
   {
      receivedPacket(subscriber.takeNextData());
   }

   @Override
   public void receivedPacket(StampedPosePacket newestStampedPosePacket)
   {
      packetQueue.add(newestStampedPosePacket);
   }

   @Override
   public boolean hasNewPose()
   {
      return !packetQueue.isEmpty();
   }

   @Override
   public StampedPosePacket getNewExternalPose()
   {
      return packetQueue.poll();
   }

   @Override
   public void sendPelvisPoseErrorPacket(PelvisPoseErrorPacket pelvisPoseErrorPacket)
   {
      if (poseErrorPublisher != null)
         poseErrorPublisher.publish(pelvisPoseErrorPacket);
   }

   @Override
   public void sendLocalizationResetRequest(LocalizationPacket localizationPacket)
   {
      if (localizationPublisher != null)
         localizationPublisher.publish(localizationPacket);
   }
}
