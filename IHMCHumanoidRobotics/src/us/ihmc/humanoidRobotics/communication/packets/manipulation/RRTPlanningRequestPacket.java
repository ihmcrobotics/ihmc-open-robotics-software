package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.Packet;

public class RRTPlanningRequestPacket extends Packet<RRTPlanningRequestPacket>
{
   
   public RRTPlanningRequestPacket()
   {
      PrintTools.info("CC");
   }

   @Override
   public boolean epsilonEquals(RRTPlanningRequestPacket other, double epsilon)
   {
      PrintTools.info("CC");
      return false;
   }

}
