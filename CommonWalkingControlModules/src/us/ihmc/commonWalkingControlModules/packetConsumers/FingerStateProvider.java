package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.commonWalkingControlModules.packets.HandStatePacket;
import us.ihmc.utilities.net.ObjectConsumer;

/**
 * @author twan
 *         Date: 6/7/13
 */
public class FingerStateProvider implements ObjectConsumer<HandStatePacket>
{
   private HandStatePacket packet;
   private boolean isNewHandStateAvailable;

   public synchronized void consumeObject(HandStatePacket packet)
   {
      this.packet = packet;
      isNewHandStateAvailable = true;
   }

   public synchronized HandStatePacket getPacket()
   {
      isNewHandStateAvailable = false;
      return packet;
   }

   public synchronized boolean isNewHandStateAvailable()
   {
      return isNewHandStateAvailable;
   }
}
