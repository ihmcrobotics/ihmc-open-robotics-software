package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commonWalkingControlModules.packets.ManualHandControlPacket;
import us.ihmc.utilities.net.ObjectConsumer;

public class ManualHandControlProvider implements ObjectConsumer<ManualHandControlPacket>
{
   
   private AtomicReference<ManualHandControlPacket> packet = new AtomicReference<ManualHandControlPacket>();

   public void consumeObject(ManualHandControlPacket packet)
   {
      this.packet.set(packet);
   }

   public ManualHandControlPacket getPacket()
   {
      return packet.getAndSet(null);
   }

   public boolean isNewPacketAvailable()
   {
      return packet.get() != null;
   }
   
}

