package us.ihmc.quadrupedRobotics.dataProviders;


import com.google.common.util.concurrent.AtomicDouble;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.quadrupedRobotics.packets.DesiredYawRatePacket;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;


public class DesiredYawRateProvider implements DoubleProvider, PacketConsumer<DesiredYawRatePacket>
{
   private final AtomicDouble lastReceivedYawRate = new AtomicDouble(0.0);

   public DesiredYawRateProvider(GlobalDataProducer dataProducer)
   {
      dataProducer.attachListener(DesiredYawRatePacket.class, this);
   }

   @Override public double getValue()
   {
      return lastReceivedYawRate.get();
   }

   @Override
   public void receivedPacket(DesiredYawRatePacket packet)
   {
      lastReceivedYawRate.set(packet.getYawRate());
   }
}
