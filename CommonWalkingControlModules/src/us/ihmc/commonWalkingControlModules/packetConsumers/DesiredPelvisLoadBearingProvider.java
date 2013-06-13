package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.commonWalkingControlModules.packets.BumStatePacket;
import us.ihmc.utilities.net.ObjectConsumer;

public class DesiredPelvisLoadBearingProvider implements ObjectConsumer<BumStatePacket>
{
   private boolean hasNewLoadBearingState = false;
   private boolean loadBearingState = false;
   
   public DesiredPelvisLoadBearingProvider()
   {
   }

   public synchronized boolean checkForNewLoadBearingState()
   {
      return hasNewLoadBearingState;
   }

   public synchronized boolean getDesiredPelvisLoadBearingState()
   {
      hasNewLoadBearingState = false;
      return loadBearingState;
   }

   public synchronized void consumeObject(BumStatePacket object)
   {
      hasNewLoadBearingState = true;
      loadBearingState = object.isLoadBearing();
   }
}
