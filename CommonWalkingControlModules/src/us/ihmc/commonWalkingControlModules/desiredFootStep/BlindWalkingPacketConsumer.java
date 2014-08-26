package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.communication.packets.walking.BlindWalkingPacket;
import us.ihmc.utilities.net.ObjectConsumer;


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

