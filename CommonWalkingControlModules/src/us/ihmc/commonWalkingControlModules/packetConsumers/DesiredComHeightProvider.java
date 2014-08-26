package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicBoolean;

import com.google.common.util.concurrent.AtomicDouble;

import us.ihmc.communication.packets.walking.ComHeightPacket;
import us.ihmc.utilities.net.ObjectConsumer;

public class DesiredComHeightProvider
{
   private final ComHeightPacketConsumer comHeightPacketConsumer;

   // Do not do it like this, preferably use one atomic
   private final AtomicBoolean newDataAvailable = new AtomicBoolean(false);
   private final AtomicDouble comHeightOffset = new AtomicDouble(0.0);

   public DesiredComHeightProvider()
   {
      comHeightPacketConsumer = new ComHeightPacketConsumer();
   }

   public ObjectConsumer<ComHeightPacket> getComHeightPacketConsumer()
   {
      return comHeightPacketConsumer;
   }


   private class ComHeightPacketConsumer implements ObjectConsumer<ComHeightPacket>
   {
      public ComHeightPacketConsumer()
      {
      }

      public void consumeObject(ComHeightPacket packet)
      {
         newDataAvailable.set(true);
         comHeightOffset.set(packet.getHeightOffset());
      }
   }


   public boolean isNewComHeightInformationAvailable()
   {
      return newDataAvailable.getAndSet(false);
   }



   public double getComHeightOffset()
   {
      return comHeightOffset.get();
   }

}
