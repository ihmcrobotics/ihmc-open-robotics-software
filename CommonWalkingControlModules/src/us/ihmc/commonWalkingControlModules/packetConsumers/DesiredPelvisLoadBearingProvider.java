package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicInteger;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.BumStatePacket;

public class DesiredPelvisLoadBearingProvider implements PacketConsumer<BumStatePacket>
{
   private AtomicInteger loadBearingState = new AtomicInteger(-1);

   public DesiredPelvisLoadBearingProvider()
   {
   }

   public boolean checkForNewLoadBearingState()
   {
      return loadBearingState.get() != -1;
   }

   public boolean getDesiredPelvisLoadBearingState()
   {
      return loadBearingState.getAndSet(-1) == 1;
   }

   public void receivedPacket(BumStatePacket object)
   {
      loadBearingState.set(object.isLoadBearing() ? 1 : 0);
   }
}
