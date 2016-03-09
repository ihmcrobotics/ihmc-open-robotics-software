package us.ihmc.aware.communication;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.aware.packets.BodyOrientationPacket;
import us.ihmc.aware.packets.ComPositionPacket;
import us.ihmc.aware.packets.PlanarVelocityPacket;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;

public class QuadrupedControllerInputProvider
{
   private final AtomicReference<ComPositionPacket> comPositionPacket = new AtomicReference<>(new ComPositionPacket());
   private final AtomicReference<BodyOrientationPacket> bodyOrientationPacket = new AtomicReference<>(new BodyOrientationPacket());
   private final AtomicReference<PlanarVelocityPacket> planarVelocityPacket = new AtomicReference<>(new PlanarVelocityPacket());

   public QuadrupedControllerInputProvider(GlobalDataProducer globalDataProducer)
   {
      globalDataProducer.attachListener(ComPositionPacket.class, new PacketConsumer<ComPositionPacket>()
      {
         @Override
         public void receivedPacket(ComPositionPacket packet)
         {
            comPositionPacket.set(packet);
         }
      });

      globalDataProducer.attachListener(BodyOrientationPacket.class, new PacketConsumer<BodyOrientationPacket>()
      {
         @Override
         public void receivedPacket(BodyOrientationPacket packet)
         {
            bodyOrientationPacket.set(packet);
         }
      });

      globalDataProducer.attachListener(PlanarVelocityPacket.class, new PacketConsumer<PlanarVelocityPacket>()
      {
         @Override
         public void receivedPacket(PlanarVelocityPacket packet)
         {
            planarVelocityPacket.set(packet);
         }
      });
   }

   public AtomicReference<ComPositionPacket> getComPositionPacket()
   {
      return comPositionPacket;
   }

   public AtomicReference<BodyOrientationPacket> getBodyOrientationPacket()
   {
      return bodyOrientationPacket;
   }

   public AtomicReference<PlanarVelocityPacket> getPlanarVelocityPacket()
   {
      return planarVelocityPacket;
   }
}
