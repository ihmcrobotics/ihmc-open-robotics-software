package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.packets.Packet;

public class RRTPlanningRequestPacket extends Packet<RRTPlanningRequestPacket>
{   
   public double tempInputValue; 
   
   public RRTPlanningRequestPacket()
   {
      
   }
   
   public void setTempValue(double value)
   {
      tempInputValue = value;
   }
   
   public void setTaskSpace()
   {
      
   }
   
   public void setEndEffectorTrajectory()
   {
      
   }

   @Override
   public boolean epsilonEquals(RRTPlanningRequestPacket other, double epsilon)
   {
      return false;
   }

}
