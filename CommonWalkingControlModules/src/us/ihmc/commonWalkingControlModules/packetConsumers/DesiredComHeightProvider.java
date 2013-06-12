package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.commonWalkingControlModules.packets.ComHeightPacket;
import us.ihmc.utilities.net.ObjectConsumer;

public class DesiredComHeightProvider
{
   private final Object synchronizationObject = new Object();

   private final ComHeightPacketConsumer comHeightPacketConsumer;

   private boolean isNewComHeightInformationAvailable;
   private double comHeightOffset;

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
         synchronized (synchronizationObject)
         {
            comHeightOffset = packet.getHeightOffset();

            isNewComHeightInformationAvailable = true;
         }
      }
   }


   public boolean isNewComHeightInformationAvailable()
   {
      synchronized (synchronizationObject)
      {
         return isNewComHeightInformationAvailable;
      }
   }



   public double getComHeightOffset()
   {
      synchronized (synchronizationObject)
      {
         isNewComHeightInformationAvailable = false;

         return comHeightOffset;
      }
   }

}
