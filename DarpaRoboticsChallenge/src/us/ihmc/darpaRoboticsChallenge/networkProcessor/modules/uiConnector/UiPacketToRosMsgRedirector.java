package us.ihmc.darpaRoboticsChallenge.networkProcessor.modules.uiConnector;

import java.net.URI;
import java.util.ArrayList;
import java.util.Map;

import org.ros.internal.message.Message;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

import us.ihmc.communication.PacketRouter;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.GlobalPacketConsumer;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.ros.IHMCPacketToMsgPublisher;
import us.ihmc.darpaRoboticsChallenge.ros.IHMCRosApiMessageMap;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.publisher.RosTopicPublisher;

public class UiPacketToRosMsgRedirector implements GlobalPacketConsumer
{
   private static final Map<String, Class> PACKETS_TO_REDIRECT_TO_ROS = IHMCRosApiMessageMap.INPUT_PACKET_MESSAGE_NAME_MAP;
   private final String ROS_NAMESPACE;
   
   private final RosMainNode rosMainNode;
   private final NodeConfiguration nodeConfiguration;
   private final MessageFactory messageFactory;
   private final ArrayList<RosTopicPublisher<?>> publishers;


   public UiPacketToRosMsgRedirector(DRCRobotModel robotModel, URI rosCoreURI, PacketCommunicator gfe_communicator, PacketRouter<PacketDestination> packetRouter, String namespace)
   {
      ROS_NAMESPACE = namespace;
      rosMainNode = new RosMainNode(rosCoreURI, ROS_NAMESPACE, true);
      this.nodeConfiguration = NodeConfiguration.newPrivate();
      this.messageFactory = nodeConfiguration.getTopicMessageFactory();
      this.publishers = new ArrayList<RosTopicPublisher<?>>();
      packetRouter.setPacketRedirects(PacketDestination.CONTROLLER, PacketDestination.GFE);
      setupMsgTopics(gfe_communicator);
      rosMainNode.execute();
//      gfe_communicator.attachGlobalListener(this);
   }

   @Override
   public void receivedPacket(Packet<?> packet)
   {
//      if (!PACKETS_TO_REDIRECT_TO_ROS.containsKey(packet.getClass()))
//      {
//         packet.setDestination(PacketDestination.CONTROLLER.ordinal());
//         packetCommunicator.send(packet);
//      }
   }

   private void setupMsgTopics(PacketCommunicator gfe_communicator)
   {
      Map<String, Class> outputPacketList = PACKETS_TO_REDIRECT_TO_ROS;

      for (Map.Entry<String, Class> e : outputPacketList.entrySet())
      {
         Message message = messageFactory.newFromType(e.getKey());

         IHMCPacketToMsgPublisher<Message, Packet> publisher = IHMCPacketToMsgPublisher.createIHMCPacketToMsgPublisher(message, false, gfe_communicator,
               e.getValue());
         publishers.add(publisher);
         rosMainNode.attachPublisher(ROS_NAMESPACE + IHMCRosApiMessageMap.PACKET_TO_TOPIC_MAP.get(e.getValue()), publisher);
      }
   }
}
