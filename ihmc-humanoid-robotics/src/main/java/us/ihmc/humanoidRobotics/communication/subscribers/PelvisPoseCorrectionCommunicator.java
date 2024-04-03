package us.ihmc.humanoidRobotics.communication.subscribers;

import java.util.concurrent.ConcurrentLinkedQueue;

import controller_msgs.msg.dds.LocalizationPacket;
import controller_msgs.msg.dds.PelvisPoseErrorPacket;
import ihmc_common_msgs.msg.dds.StampedPosePacket;
import us.ihmc.communication.StateEstimatorAPI;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.RealtimeROS2Node;

public class PelvisPoseCorrectionCommunicator implements PelvisPoseCorrectionCommunicatorInterface
{
   private final ConcurrentLinkedQueue<StampedPosePacket> packetQueue = new ConcurrentLinkedQueue<StampedPosePacket>();
   private final ROS2PublisherBasics<PelvisPoseErrorPacket> poseErrorPublisher;
   private final ROS2PublisherBasics<LocalizationPacket> localizationPublisher;

   public PelvisPoseCorrectionCommunicator(RealtimeROS2Node realtimeROS2Node, String robotName)
   {
      if (realtimeROS2Node != null && robotName != null)
      {
         poseErrorPublisher = realtimeROS2Node.createPublisher(StateEstimatorAPI.getTopic(PelvisPoseErrorPacket.class, robotName));
         localizationPublisher = realtimeROS2Node.createPublisher(StateEstimatorAPI.getTopic(LocalizationPacket.class, robotName));
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
