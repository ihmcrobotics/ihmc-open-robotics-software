package us.ihmc.quadrupedRobotics.dataProviders;

import us.ihmc.communication.packetCommunicator.ConcurrentPacketQueue;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.concurrent.Builder;
import us.ihmc.quadrupedRobotics.packets.DesiredYawRatePacket;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;

public class DesiredYawRateProvider implements DoubleProvider
{
   private final ConcurrentPacketQueue<DesiredYawRatePacket> packetQueue = new ConcurrentPacketQueue<>();
   private double lastReceivedYawRate;

   public DesiredYawRateProvider(GlobalDataProducer dataProducer)
   {
      dataProducer.attachListener(DesiredYawRatePacket.class, packetQueue);
   }

   @Override public double getValue()
   {
      if(packetQueue.isNewPacketAvailable())
         lastReceivedYawRate = packetQueue.getPacket().getYawRate();

      return lastReceivedYawRate;
   }

   class DesiredYawRatePacketBuilder implements Builder<DesiredYawRatePacket>
   {
      @Override public DesiredYawRatePacket newInstance()
      {
         return new DesiredYawRatePacket(0.0);
      }
   }
}
