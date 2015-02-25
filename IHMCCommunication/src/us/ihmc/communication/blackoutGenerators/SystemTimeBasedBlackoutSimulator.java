package us.ihmc.communication.blackoutGenerators;

import java.util.concurrent.TimeUnit;

import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;

public class SystemTimeBasedBlackoutSimulator extends StandardBlackoutSimulator
{
   public SystemTimeBasedBlackoutSimulator(CommunicationBlackoutGenerator blackoutGenerator, PacketCommunicator packetCommunicator)
   {
      super(blackoutGenerator, packetCommunicator);
   }

   @Override
   public long getCurrentTime(TimeUnit timeUnit)
   {
      return timeUnit.convert(System.currentTimeMillis(), TimeUnit.MILLISECONDS);
   }
}
