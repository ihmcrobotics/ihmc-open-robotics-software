package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.walking.BlindWalkingPacket;


public class BlindWalkingPacketConsumer implements PacketConsumer<BlindWalkingPacket>
{
   private FootstepPathCoordinator footstepPathCoordinator;

   public BlindWalkingPacketConsumer(FootstepPathCoordinator footstepPathCoordinator)
   {
      this.footstepPathCoordinator = footstepPathCoordinator;
   }

   public void receivedPacket(BlindWalkingPacket blindWalkingPacket)
   {
      footstepPathCoordinator.setBlindWalking(blindWalkingPacket);
   }
}

