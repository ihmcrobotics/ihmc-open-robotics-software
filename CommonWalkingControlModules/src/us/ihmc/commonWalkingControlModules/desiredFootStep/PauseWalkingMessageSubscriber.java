package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.walking.PauseWalkingMessage;

/**
 * User: Matt
 * Date: 1/18/13
 */
public class PauseWalkingMessageSubscriber implements PacketConsumer<PauseWalkingMessage>
{
   private final FootstepPathCoordinator footstepPathCoordinator;

   public PauseWalkingMessageSubscriber(FootstepPathCoordinator footstepPathCoordinator)
   {
      this.footstepPathCoordinator = footstepPathCoordinator;
   }

   @Override
   public void receivedPacket(PauseWalkingMessage pauseWalkingMessage)
   {
      footstepPathCoordinator.setPaused(pauseWalkingMessage.isPaused());
   }
}
