package us.ihmc.humanoidRobotics.communication.packets.wholebody;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.communication.packets.Packet;

/**
 * MessageOfMessages provides a generic way to send a collection of messages to the controller.
 */
public class MessageOfMessages extends Packet<MessageOfMessages>
{
   public List<Packet<?>> packets = new ArrayList<>();

   public MessageOfMessages()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(MessageOfMessages other)
   {
      packets.clear();
      packets.addAll(other.packets);
      setPacketInformation(other);
   }

   public void addPacket(Packet<?>... messages)
   {
      for (Packet<?> packet : messages)
      {
         if (packet instanceof MessageOfMessages)
            packets.addAll(((MessageOfMessages) packet).getPackets());
         else
            packets.add(packet);
      }
   }

   public void clear()
   {
      packets.clear();
   }

   public List<Packet<?>> getPackets()
   {
      return packets;
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   @Override
   public boolean epsilonEquals(MessageOfMessages other, double epsilon)
   {
      if (packets.size() != other.packets.size())
      {
         return false;
      }

      for (int i = 0; i < packets.size(); i++)
      {
         Packet packet = packets.get(i);
         Packet otherPacket = other.packets.get(i);
         packet.epsilonEquals(otherPacket, epsilon);
      }

      return true;
   }
}
