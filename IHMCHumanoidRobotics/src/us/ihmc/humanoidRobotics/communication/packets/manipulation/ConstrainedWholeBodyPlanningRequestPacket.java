package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.packets.Packet;

public class ConstrainedWholeBodyPlanningRequestPacket extends Packet<ConstrainedWholeBodyPlanningRequestPacket>
{   
   public double tempInputValue; 
   
   public ConstrainedWholeBodyPlanningRequestPacket()
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
   public boolean epsilonEquals(ConstrainedWholeBodyPlanningRequestPacket other, double epsilon)
   {
      return false;
   }

}
