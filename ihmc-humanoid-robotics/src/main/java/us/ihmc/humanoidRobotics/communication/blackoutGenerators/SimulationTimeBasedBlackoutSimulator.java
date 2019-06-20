package us.ihmc.humanoidRobotics.communication.blackoutGenerators;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.communication.blackoutGenerators.CommunicationBlackoutGenerator;
import us.ihmc.communication.blackoutGenerators.StandardBlackoutSimulator;
import us.ihmc.communication.net.PacketConsumer;

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
      currentSimTime.set(packet.getMonotonicTime());
   }
}
