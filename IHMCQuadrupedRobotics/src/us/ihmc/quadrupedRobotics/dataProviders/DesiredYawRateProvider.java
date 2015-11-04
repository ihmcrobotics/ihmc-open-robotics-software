package us.ihmc.quadrupedRobotics.dataProviders;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.quadrupedRobotics.packets.DesiredYawRatePacket;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;

import java.util.concurrent.atomic.AtomicReference;

public class DesiredYawRateProvider implements DoubleProvider, PacketConsumer<DesiredYawRatePacket>
{
   private final AtomicReference<DesiredYawRatePacket> latestPacket = new AtomicReference<>();

   public DesiredYawRateProvider(GlobalDataProducer dataProducer)
   {
      dataProducer.attachListener(DesiredYawRatePacket.class, this);
   }

   @Override public double getValue()
   {
      return latestPacket.get().getYawRate();
   }

   @Override public void receivedPacket(DesiredYawRatePacket packet)
   {
      latestPacket.set(packet);
   }
}
