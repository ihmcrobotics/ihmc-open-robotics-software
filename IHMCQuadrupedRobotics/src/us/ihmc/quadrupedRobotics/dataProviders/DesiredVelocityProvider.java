package us.ihmc.quadrupedRobotics.dataProviders;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.quadrupedRobotics.packets.DesiredVelocityPacket;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.VectorProvider;

public class DesiredVelocityProvider implements VectorProvider, PacketConsumer<DesiredVelocityPacket>
{
   private final AtomicReference<DesiredVelocityPacket> lastReceivedVelocityPacket = new AtomicReference<DesiredVelocityPacket>(new DesiredVelocityPacket());
   
   
   public DesiredVelocityProvider(GlobalDataProducer dataProducer)
   {
      dataProducer.attachListener(DesiredVelocityPacket.class, this);
   }

   @Override public void get(FrameVector frameVectorToPack)
   {

      DesiredVelocityPacket received = lastReceivedVelocityPacket.get();      
      frameVectorToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), received.getVelocity());
   }

   @Override
   public void receivedPacket(DesiredVelocityPacket packet)
   {
      lastReceivedVelocityPacket.set(packet);
   }
}
