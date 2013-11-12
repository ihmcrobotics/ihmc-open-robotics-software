package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commonWalkingControlModules.packets.FingerStatePacket;
import us.ihmc.utilities.net.ObjectConsumer;

/**
 * @author twan
 *         Date: 6/7/13
 */
public class FingerStateProvider implements ObjectConsumer<FingerStatePacket>
{
   private AtomicReference<FingerStatePacket> packet = new AtomicReference<FingerStatePacket>();

   public void consumeObject(FingerStatePacket packet)
   {
      this.packet.set(packet);
   }

   public FingerStatePacket getPacket()
   {
      return packet.getAndSet(null);
   }

   public boolean isNewFingerStateAvailable()
   {
      return packet.get() != null;
   }
}
