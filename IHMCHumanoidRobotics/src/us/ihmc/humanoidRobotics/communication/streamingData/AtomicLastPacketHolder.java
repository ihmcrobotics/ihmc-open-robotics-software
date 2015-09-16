package us.ihmc.humanoidRobotics.communication.streamingData;

import java.util.concurrent.atomic.AtomicLong;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.Packet;
import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentCopier;

public class AtomicLastPacketHolder implements PacketConsumer
{
   private final AtomicLong robotTime = new AtomicLong();
   private final ConcurrentCopier<LastPacket> copier = new ConcurrentCopier<>(new LastPacketBuilder());


   public LastPacket getLastPacket()
   {
      return copier.getCopyForReading();
   }
   
   public void setRobotTime(long time)
   {
      robotTime.set(time);
   }

   public static class LastPacket
   {

      private LastPacket()
      {

      }

      private Class<? extends Packet> packet;
      private long uniqueId;
      private long receivedTimestamp;

      public Class<? extends Packet> getPacket()
      {
         return packet;
      }

      public long getUniqueId()
      {
         return uniqueId;
      }

      public long getReceivedTimestamp()
      {
         return receivedTimestamp;
      }

   }

   public static class LastPacketBuilder implements Builder<LastPacket>
   {

      @Override
      public LastPacket newInstance()
      {
         return new LastPacket();
      }

   }

   @Override
   public void receivedPacket(Packet packet)
   {
      LastPacket writable = copier.getCopyForWriting();
      writable.packet = packet.getClass();
      writable.uniqueId = packet.getUniqueId();
      writable.receivedTimestamp = robotTime.get();
      copier.commit();      
   }

}
