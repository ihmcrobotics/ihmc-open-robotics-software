package us.ihmc.darpaRoboticsChallenge.ros.subscriber;

import org.ros.internal.message.Message;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.darpaRoboticsChallenge.ros.DRCROSMessageConverter;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public class IHMCMsgToPacketSubscriber<T extends Message> extends AbstractRosTopicSubscriber<T>
{
   private final PacketCommunicator controllerCommunicator;
   private int packetDestination;

   public IHMCMsgToPacketSubscriber(String messageType, PacketCommunicator communicator, int packetDestination)
   {
      super(messageType);
      this.controllerCommunicator = communicator;
      this.packetDestination = packetDestination;
   }

   @Override
   public void onNewMessage(T message)
   {
      try
      {
         Packet<?> packet = DRCROSMessageConverter.convertToPacket(message);
         packet.setDestination(packetDestination);
         controllerCommunicator.send(packet);
      }
      catch (IllegalArgumentException | SecurityException e)
      {
         System.out.println(this.getMessageType().getClass().getSimpleName() + " failed to covert message to packet! see exception below.");
         e.printStackTrace();
      }
   }

   public static <U extends Message> IHMCMsgToPacketSubscriber<U> createIHMCMsgToPacketSubscriber(U msg, PacketCommunicator communicator, int packetDestination)
   {
      return new IHMCMsgToPacketSubscriber<U>(msg.toRawMessage().getType(), communicator, packetDestination);
   }
}
