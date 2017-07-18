package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningRequestPacket;

public class RRTPlanningRequestPacket extends Packet<FootstepPlanningRequestPacket>
{

   @Override
   public boolean epsilonEquals(FootstepPlanningRequestPacket other, double epsilon)
   {
      // TODO Auto-generated method stub
      return false;
   }

}
