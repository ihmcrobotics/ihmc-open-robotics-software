package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.packets.Packet;

public class ConstrainedWholebodyPlanningRequestPacket extends Packet<ConstrainedWholebodyPlanningRequestPacket>
{   
   public double tempInputValue; 
   
   public ConstrainedWholebodyPlanningRequestPacket()
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
   public boolean epsilonEquals(ConstrainedWholebodyPlanningRequestPacket other, double epsilon)
   {
      return false;
   }

}
