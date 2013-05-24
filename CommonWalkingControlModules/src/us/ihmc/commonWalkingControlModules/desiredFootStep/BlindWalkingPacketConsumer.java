package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.BlindWalkingPacket;
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
      System.out.println("Received aaa blind packet " + blindWalkingPacket);
      footstepPathCoordinator.setBlindWalking(blindWalkingPacket);
   }
}

