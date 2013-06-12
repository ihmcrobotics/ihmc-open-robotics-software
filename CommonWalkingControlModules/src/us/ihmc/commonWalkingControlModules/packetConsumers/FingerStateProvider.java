package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.commonWalkingControlModules.packets.FingerStatePacket;
import us.ihmc.commonWalkingControlModules.packets.HandStatePacket;
import us.ihmc.utilities.net.ObjectConsumer;

/**
 * @author twan
 *         Date: 6/7/13
 */
public class FingerStateProvider implements ObjectConsumer<FingerStatePacket>
{
   private FingerStatePacket packet;
   private boolean isNewFingerStateAvailable;

   public synchronized void consumeObject(FingerStatePacket packet)
   {
      this.packet = packet;
      isNewFingerStateAvailable = true;
   }

   public synchronized FingerStatePacket getPacket()
   {
      isNewFingerStateAvailable = false;
      return packet;
   }

   public synchronized boolean isNewFingerStateAvailable()
   {
      return isNewFingerStateAvailable;
   }
}
