package us.ihmc.communication.blackoutGenerators;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.BlackoutDurationPacket;

public class UserSettableBlackoutGenerator implements CommunicationBlackoutGenerator, PacketConsumer<BlackoutDurationPacket>
{
   private final TimeUnit timeUnit;
   private final AtomicLong blackoutLength = new AtomicLong();
   
   public UserSettableBlackoutGenerator(PacketCommunicator packetCommunicator, TimeUnit timeUnit)
   {
      this.timeUnit = timeUnit;
      packetCommunicator.attachListener(BlackoutDurationPacket.class, this);
   }

   @Override
   public long calculateNextBlackoutLength(long currentTime, TimeUnit timeUnit)
   {
      return timeUnit.convert(blackoutLength.get(), this.timeUnit);
   }

   @Override
   public void receivedPacket(BlackoutDurationPacket packet)
   {
      blackoutLength.set(timeUnit.convert(packet.getBlackoutDuration(), TimeUnit.SECONDS));
   }

}
