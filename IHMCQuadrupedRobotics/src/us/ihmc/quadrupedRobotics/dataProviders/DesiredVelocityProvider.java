package us.ihmc.quadrupedRobotics.dataProviders;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.quadrupedRobotics.packets.DesiredVelocityPacket;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.trajectories.providers.VectorProvider;

import java.util.concurrent.atomic.AtomicReference;

public class DesiredVelocityProvider implements VectorProvider, PacketConsumer<DesiredVelocityPacket>
{
   private final AtomicReference<DesiredVelocityPacket> latestPacket = new AtomicReference<>();

   public DesiredVelocityProvider(GlobalDataProducer dataProducer)
   {
      dataProducer.attachListener(DesiredVelocityPacket.class, this);
   }

   @Override public void get(FrameVector frameVectorToPack)
   {
      latestPacket.get();
   }

   @Override public void receivedPacket(DesiredVelocityPacket packet)
   {
      latestPacket.set(packet);
   }
}
