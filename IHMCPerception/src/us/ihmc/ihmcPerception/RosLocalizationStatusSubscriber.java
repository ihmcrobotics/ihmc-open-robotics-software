package us.ihmc.ihmcPerception;

import std_msgs.Float64;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.sensing.LocalizationStatusPacket;
import us.ihmc.humanoidRobotics.kryo.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

/**
 * Created by dstephen on 3/20/15.
 */
public class RosLocalizationStatusSubscriber
{
   private double overlap = RosLocalizationConstants.DEFAULT_OVERLAP;

   public RosLocalizationStatusSubscriber(final RosMainNode rosMainNode, final PacketCommunicator packetCommunicator,
         final PPSTimestampOffsetProvider ppsTimeOffsetProvider)
   {
      AbstractRosTopicSubscriber<Float64> overlapSubscriber = new AbstractRosTopicSubscriber<std_msgs.Float64>(std_msgs.Float64._TYPE) {
         @Override
         public void onNewMessage(std_msgs.Float64 message) {
            overlap = message.getData();
            LocalizationStatusPacket localizationOverlapPacket = new LocalizationStatusPacket(overlap,null);
            packetCommunicator.send(localizationOverlapPacket);
         }
      };
      rosMainNode.attachSubscriber(RosLocalizationConstants.OVERLAP_UPDATE_TOPIC, overlapSubscriber);

      AbstractRosTopicSubscriber<std_msgs.String> statusSubscriber = new AbstractRosTopicSubscriber<std_msgs.String>(std_msgs.String._TYPE) {

         @Override
         public void onNewMessage(std_msgs.String message) {
            String status = message.getData();
            LocalizationStatusPacket localizationOverlapPacket = new LocalizationStatusPacket(overlap,status);
            packetCommunicator.send(localizationOverlapPacket);
         }

      };

      rosMainNode.attachSubscriber(RosLocalizationConstants.STATUS_UPDATE_TOPIC, statusSubscriber);
   }
}
