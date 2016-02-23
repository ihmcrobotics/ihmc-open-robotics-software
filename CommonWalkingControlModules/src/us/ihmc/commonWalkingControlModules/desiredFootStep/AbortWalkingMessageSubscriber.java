package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.walking.AbortWalkingMessage;

/**
 * Created by agrabertilton on 4/28/15.
 */
public class AbortWalkingMessageSubscriber implements PacketConsumer<AbortWalkingMessage>
{
   private boolean abortWalking;

   public AbortWalkingMessageSubscriber()
   {
   }

   public boolean shouldAbortWalking()
   {
      return abortWalking;
   }

   public void walkingAborted()
   {
      abortWalking = false;
   }

   public void triggerAbort()
   {
      abortWalking = true;
   }

   @Override
   public void receivedPacket(AbortWalkingMessage packet)
   {
      triggerAbort();
   }
}
