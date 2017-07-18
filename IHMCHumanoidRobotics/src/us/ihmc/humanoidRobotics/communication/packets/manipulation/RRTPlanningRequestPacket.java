package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.packets.Packet;

public class RRTPlanningRequestPacket extends Packet<RRTPlanningRequestPacket>
{
   
   public RRTPlanningRequestPacket()
   {
      
   }

   @Override
   public boolean epsilonEquals(RRTPlanningRequestPacket other, double epsilon)
   {
      
      return true;
   }

}
