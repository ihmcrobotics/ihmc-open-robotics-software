package us.ihmc.humanoidRobotics.communication.subscribers;

import controller_msgs.LocalizationPacket;
import controller_msgs.PelvisPoseErrorPacket;
import ihmc_common_msgs.StampedPosePacket;
import us.ihmc.communication.StateEstimatorAPI;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;

import java.util.concurrent.ConcurrentLinkedQueue;

public class PelvisPoseCorrectionCommunicator implements PelvisPoseCorrectionCommunicatorInterface
{
   private final ConcurrentLinkedQueue<StampedPosePacket> packetQueue = new ConcurrentLinkedQueue<StampedPosePacket>();
   private final ROS2Publisher<PelvisPoseErrorPacket> poseErrorPublisher;
   private final ROS2Publisher<LocalizationPacket> localizationPublisher;

   public PelvisPoseCorrectionCommunicator(ROS2Node realtimeROS2Node, String robotName)
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
