package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packets.Packet;

/**
 * Created by agrabertilton on 4/28/15.
 */
@ClassDocumentation("This message is used to abort walking, forcing the robot to switch back to double support and clear the footstep list.")

public class AbortWalkingMessage extends Packet<AbortWalkingMessage>
{
   @Override
   public boolean epsilonEquals(AbortWalkingMessage other, double epsilon)
   {
      return true;
   }
}
