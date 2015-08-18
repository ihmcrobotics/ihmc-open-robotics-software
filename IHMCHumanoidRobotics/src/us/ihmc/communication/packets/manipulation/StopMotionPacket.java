package us.ihmc.communication.packets.manipulation;

import us.ihmc.communication.packets.Packet;

public class StopMotionPacket extends Packet<StopMotionPacket>
{

   public StopMotionPacket()
   {
   }  
   
   @Override
   public boolean epsilonEquals(StopMotionPacket other, double epsilon) 
   {
      return true;
   }
}
