package us.ihmc.robotEnvironmentAwareness.tools;

import perception_msgs.msg.dds.OcTreeKeyListMessage;
import perception_msgs.msg.dds.OcTreeKeyListMessagePubSubType;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.idl.serializers.extra.YAMLSerializer;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.ros2.ROS2Node;

public class OcTreeTopicEcho
{
   public OcTreeTopicEcho()
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, getClass().getSimpleName());
      YAMLSerializer<OcTreeKeyListMessage> serializer = new YAMLSerializer<>(new OcTreeKeyListMessagePubSubType());

      ros2Node.createSubscription(REACommunicationProperties.outputTopic.withTypeName(OcTreeKeyListMessage.class), s ->
      {
         try
         {
            OcTreeKeyListMessage ocTreeKeyListMessage = s.takeNextData();
            String serializedString = serializer.serializeToString(ocTreeKeyListMessage);
            System.out.println(serializedString);
            System.out.println("\n---------------------------------------------------------------------------------------------\n");
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
      });

      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new OcTreeTopicEcho();
   }
}
