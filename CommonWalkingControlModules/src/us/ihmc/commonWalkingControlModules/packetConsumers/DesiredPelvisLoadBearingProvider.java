package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.commonWalkingControlModules.packets.BumStatePacket;
import us.ihmc.utilities.net.ObjectConsumer;

public class DesiredPelvisLoadBearingProvider implements ObjectConsumer<BumStatePacket>
{
   private boolean hasLoadBearingBeenRequested = false;
   
   public DesiredPelvisLoadBearingProvider()
   {
   }
   
   public synchronized boolean checkForNewLoadBearingRequest()
   {
      boolean ret = hasLoadBearingBeenRequested;
      hasLoadBearingBeenRequested = false;
      
      return ret;
   }

   public synchronized void consumeObject(BumStatePacket object)
   {
      hasLoadBearingBeenRequested = true;
   }
}
