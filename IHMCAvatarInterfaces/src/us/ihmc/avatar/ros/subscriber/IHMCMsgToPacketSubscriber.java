package us.ihmc.avatar.ros.subscriber;

import org.ros.internal.message.Message;

import us.ihmc.avatar.ros.IHMCROSTranslationRuntimeTools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.utilities.ros.msgToPacket.converter.RosEnumConversionException;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.lang.reflect.InvocationTargetException;

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
         Packet<?> packet = IHMCROSTranslationRuntimeTools.convertToIHMCMessage(message);
         packet.setDestination(packetDestination);
         controllerCommunicator.send(packet);
      }
      catch (ClassNotFoundException | InvocationTargetException | RosEnumConversionException | IllegalAccessException | InstantiationException | NoSuchFieldException e)
      {
         System.err.println("Could not convert ROS Message to IHMC Message. Will not publish.");
         System.err.println("Original ROS message: " + message);
         e.printStackTrace();
      }
   }

   public static <U extends Message> IHMCMsgToPacketSubscriber<U> createIHMCMsgToPacketSubscriber(U msg, PacketCommunicator communicator, int packetDestination)
   {
      return new IHMCMsgToPacketSubscriber<U>(msg.toRawMessage().getType(), communicator, packetDestination);
   }
}
