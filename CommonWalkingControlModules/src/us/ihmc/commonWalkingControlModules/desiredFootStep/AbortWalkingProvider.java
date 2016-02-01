package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.walking.AbortWalkingPacket;

/**
 * Created by agrabertilton on 4/28/15.
 */
public class AbortWalkingProvider implements PacketConsumer<AbortWalkingPacket>
{
   private boolean abortWalking;

   public boolean shouldAbortWalking(){
      return abortWalking;
   }

   public void walkingAborted(){
      abortWalking = false;
   }

   public void triggerAbort(){
      abortWalking = true;
   }

   @Override
   public void receivedPacket(AbortWalkingPacket packet)
   {
      triggerAbort();
   }
}
