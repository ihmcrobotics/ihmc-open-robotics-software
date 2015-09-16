package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.walking.PauseCommand;

/**
 * User: Matt
 * Date: 1/18/13
 */
public class PauseCommandConsumer implements PacketConsumer<PauseCommand>
{
   private FootstepPathCoordinator footstepPathCoordinator;

   public PauseCommandConsumer(FootstepPathCoordinator footstepPathCoordinator)
   {
      this.footstepPathCoordinator = footstepPathCoordinator;
   }

   public void receivedPacket(PauseCommand object)
   {
      footstepPathCoordinator.setPaused(object.isPaused());
   }
}
