package us.ihmc.aware.communication;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.aware.packets.BodyPosePacket;
import us.ihmc.aware.packets.BodyTwistPacket;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;

public class QuadrupedControllerInputProvider
{
   private final AtomicReference<BodyPosePacket> bodyPosePacket = new AtomicReference<>(new BodyPosePacket());
   private final AtomicReference<BodyTwistPacket> bodyTwistPacket = new AtomicReference<>(new BodyTwistPacket());

   public QuadrupedControllerInputProvider(GlobalDataProducer globalDataProducer)
   {
      globalDataProducer.attachListener(BodyPosePacket.class, new PacketConsumer<BodyPosePacket>()
      {
         @Override
         public void receivedPacket(BodyPosePacket packet)
         {
            bodyPosePacket.set(packet);
         }
      });

      globalDataProducer.attachListener(BodyTwistPacket.class, new PacketConsumer<BodyTwistPacket>()
      {
         @Override
         public void receivedPacket(BodyTwistPacket packet)
         {
            bodyTwistPacket.set(packet);
         }
      });
   }

   public AtomicReference<BodyPosePacket> getBodyPosePacket()
   {
      return bodyPosePacket;
   }

   public AtomicReference<BodyTwistPacket> getBodyTwistPacket()
   {
      return bodyTwistPacket;
   }
}
