package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.communication.packets.walking.BlindWalkingPacket;


public class BlindWalkingPacketConsumer implements ObjectConsumer<BlindWalkingPacket>
{
   private FootstepPathCoordinator footstepPathCoordinator;

   public BlindWalkingPacketConsumer(FootstepPathCoordinator footstepPathCoordinator)
   {
      this.footstepPathCoordinator = footstepPathCoordinator;
   }

   public void consumeObject(BlindWalkingPacket blindWalkingPacket)
   {
      footstepPathCoordinator.setBlindWalking(blindWalkingPacket);
   }
}

