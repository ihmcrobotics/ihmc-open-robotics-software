package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.commonWalkingControlModules.packets.FingerStatePacket;
import us.ihmc.utilities.net.ObjectConsumer;

/**
 * @author twan
 *         Date: 6/7/13
 */
public class FingerStateProvider implements ObjectConsumer<FingerStatePacket>
{
   private FingerStatePacket packet;
   private boolean isNewHandStateAvailable;

   public synchronized void consumeObject(FingerStatePacket packet)
   {
      this.packet = packet;
      isNewHandStateAvailable = true;
   }

   public synchronized FingerStatePacket getPacket()
   {
      isNewHandStateAvailable = false;
      return packet;
   }

   public synchronized boolean isNewHandStateAvailable()
   {
      return isNewHandStateAvailable;
   }
}
