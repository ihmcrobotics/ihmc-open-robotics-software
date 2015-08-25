package us.ihmc.communication.packets.walking;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;

/**
 * Created by agrabertilton on 4/28/15.
 */
@ClassDocumentation("This message is used to abort walking, forcing the robot to switch back to double support and clear the footstep list.")

public class AbortWalkingPacket extends IHMCRosApiPacket<FootStatePacket>
{
   @Override
   public boolean epsilonEquals(FootStatePacket other, double epsilon)
   {
      return true;
   }
}
