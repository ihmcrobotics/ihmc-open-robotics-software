package us.ihmc.avatar.networkProcessor.modules.uiConnector;

import java.net.URI;
import java.util.ArrayList;
import java.util.Set;

import org.ros.internal.message.Message;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros.IHMCPacketToMsgPublisher;
import us.ihmc.avatar.ros.IHMCROSTranslationRuntimeTools;
import us.ihmc.communication.PacketRouter;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.GlobalPacketConsumer;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.msgToPacket.converter.GenericROSTranslationTools;
import us.ihmc.utilities.ros.publisher.RosTopicPublisher;

public class UiPacketToRosMsgRedirector implements GlobalPacketConsumer
{
//   private static final Map<String, Class> PACKETS_TO_REDIRECT_TO_ROS = null; //IHMCRosApiMessageMap.INPUT_PACKET_MESSAGE_NAME_MAP;
   private static final Set<Class<?>> PACKETS_TO_REDIRECT_TO_ROS = GenericROSTranslationTools.getCoreInputTopics();
   private final String ROS_NAMESPACE;
   
   private final RosMainNode rosMainNode;
   private final NodeConfiguration nodeConfiguration;
   private final MessageFactory messageFactory;
   private final ArrayList<RosTopicPublisher<?>> publishers;


   public UiPacketToRosMsgRedirector(DRCRobotModel robotModel, URI rosCoreURI, PacketCommunicator rosAPI_communicator, PacketRouter<PacketDestination> packetRouter, String namespace)
   {
      ROS_NAMESPACE = namespace;
      rosMainNode = new RosMainNode(rosCoreURI, ROS_NAMESPACE, true);
      this.nodeConfiguration = NodeConfiguration.newPrivate();
      this.messageFactory = nodeConfiguration.getTopicMessageFactory();
      this.publishers = new ArrayList<RosTopicPublisher<?>>();
      packetRouter.setPacketRedirects(PacketDestination.CONTROLLER, PacketDestination.ROS_API);
      setupMsgTopics(rosAPI_communicator);
      rosMainNode.execute();
//      rosAPI_communicator.attachGlobalListener(this);
   }

   @Override
   public void receivedPacket(Packet<?> packet)
   {

   }

   @SuppressWarnings("unchecked")
   private void setupMsgTopics(PacketCommunicator rosAPI_communicator)
   {
      Set<Class<?>> outputPacketList = PACKETS_TO_REDIRECT_TO_ROS;

      for (Class clazz : outputPacketList)
      {
         RosMessagePacket annotation = (RosMessagePacket) clazz.getAnnotation(RosMessagePacket.class);
         Message message = messageFactory.newFromType(IHMCROSTranslationRuntimeTools.getROSMessageTypeStringFromIHMCMessageClass(clazz));

         IHMCPacketToMsgPublisher<Message, Packet> publisher = IHMCPacketToMsgPublisher.createIHMCPacketToMsgPublisher(message, false, rosAPI_communicator,
               clazz);
         publishers.add(publisher);
         rosMainNode.attachPublisher(ROS_NAMESPACE + annotation.topic(), publisher);
      }
   }
}
