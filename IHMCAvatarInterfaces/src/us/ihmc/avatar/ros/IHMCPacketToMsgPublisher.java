package us.ihmc.avatar.ros;

import java.lang.reflect.InvocationTargetException;

import org.ros.internal.message.Message;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.utilities.ros.publisher.RosTopicPublisher;

public class IHMCPacketToMsgPublisher<T extends Message, U extends Packet> extends RosTopicPublisher<T> implements PacketConsumer<U>
{
   private static final boolean DEBUG = false;
   public IHMCPacketToMsgPublisher(String messageType, boolean latched)
   {
      super(messageType, latched);
   }

   @SuppressWarnings("unchecked")
   @Override
   public void receivedPacket(U packet)
   {
      if(DEBUG)
      {
         System.out.println("converting " + packet.getClass() + " to ros");
      }

      T msg = null;

      try
      {
         msg = (T) IHMCROSTranslationRuntimeTools.convertToRosMessage(packet);
      }
      catch (InvocationTargetException | IllegalAccessException | NoSuchMethodException | ClassNotFoundException e)
      {
         System.err.println("Could not convert IHMC Message to ROS Message. Will not publish.");
         System.err.println("Original packet: " + packet);
         e.printStackTrace();
      }

      if(isConnected() && msg != null)
      {
         publish(msg);
      }
   }

   public static <T extends Message, U extends Packet> IHMCPacketToMsgPublisher<T, U> createIHMCPacketToMsgPublisher(Message message, boolean latched,
         PacketCommunicator rosAPI_communicator, Class<U> clazz)
   {
      IHMCPacketToMsgPublisher<T, U> pub = new IHMCPacketToMsgPublisher<T, U>(message.toRawMessage().getType(), latched);
      rosAPI_communicator.attachListener(clazz, pub);
      return pub;
   }
}
