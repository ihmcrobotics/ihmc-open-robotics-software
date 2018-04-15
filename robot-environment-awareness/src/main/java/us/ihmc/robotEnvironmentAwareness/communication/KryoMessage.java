package us.ihmc.robotEnvironmentAwareness.communication;

import us.ihmc.communication.packets.Packet;

public class KryoMessage<T> extends Packet<KryoMessage<T>>
{
   public Message<T> message;

   public KryoMessage()
   {
   }

   public KryoMessage(Message<T> message)
   {
      this.message = message;
   }

   @Override
   public void set(KryoMessage<T> other)
   {
      message.set(other.message);
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(KryoMessage<T> other, double epsilon)
   {
      return message.epsilonEquals(other.message, epsilon);
   }
}
