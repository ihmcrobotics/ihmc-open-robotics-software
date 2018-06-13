package us.ihmc.humanoidRobotics.communication.subscribers;

import controller_msgs.msg.dds.LocalizationPacket;
import controller_msgs.msg.dds.PelvisPoseErrorPacket;
import controller_msgs.msg.dds.StampedPosePacket;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.NewMessageListener;

public interface PelvisPoseCorrectionCommunicatorInterface extends NewMessageListener<StampedPosePacket>, PacketConsumer<StampedPosePacket>
{
   public boolean hasNewPose();

   public StampedPosePacket getNewExternalPose();
   
   public void sendPelvisPoseErrorPacket(PelvisPoseErrorPacket pelvisPoseErrorPacket);
   
   public void sendLocalizationResetRequest(LocalizationPacket localizationPacket);

   @Override
   default void onNewDataMessage(Subscriber<StampedPosePacket> subscriber)
   {
      receivedPacket(subscriber.readNextData());
   }
}
