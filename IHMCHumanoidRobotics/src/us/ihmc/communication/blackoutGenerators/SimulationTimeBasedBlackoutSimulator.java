package us.ihmc.communication.blackoutGenerators;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;

public class SimulationTimeBasedBlackoutSimulator extends StandardBlackoutSimulator implements PacketConsumer<RobotConfigurationData>
{
   private volatile AtomicLong currentSimTime = new AtomicLong();
   
   public SimulationTimeBasedBlackoutSimulator(CommunicationBlackoutGenerator blackoutGenerator)
   {
      super(blackoutGenerator);
   }

   @Override
   public long getCurrentTime(TimeUnit timeUnit)
   {
      return timeUnit.convert(currentSimTime.get(), TimeUnit.NANOSECONDS);
   }

   @Override
   public void receivedPacket(RobotConfigurationData packet)
   {
      currentSimTime.set(packet.getTimestamp());
   }
}
