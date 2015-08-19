package us.ihmc.darpaRoboticsChallenge.ros;

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
      try
      {
         T msg = (T) DRCROSMessageConverter.convertToRosMessage(packet);
         if(isConnected())
         {
            publish(msg);
         }
      }
      catch (IllegalArgumentException | SecurityException e)
      {
         System.out.println("Problem consuming Packet of class: " + packet.getClass().getSimpleName());
         System.out.println(this.getMessageType().getClass().getSimpleName() + " failed to convert packet to message! see exception below.");
         e.printStackTrace();
      }
      catch (NullPointerException e)
      {
         System.err.println("Received a malformed packet of type " + packet.getClass().getSimpleName());
         e.printStackTrace();
      }
   }

   public static <T extends Message, U extends Packet> IHMCPacketToMsgPublisher<T, U> createIHMCPacketToMsgPublisher(Message message, boolean latched,
         PacketCommunicator gfe_communicator, Class<U> clazz)
   {
      IHMCPacketToMsgPublisher<T, U> pub = new IHMCPacketToMsgPublisher<T, U>(message.toRawMessage().getType(), latched);
      gfe_communicator.attachListener(clazz, pub);
      return pub;
   }
}
