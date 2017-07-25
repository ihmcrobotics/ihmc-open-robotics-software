package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.packets.Packet;

public class ConstrainedWholeBodyPlanningRequestPacket extends Packet<ConstrainedWholeBodyPlanningRequestPacket>
{      
   /*
    * input parameters will be set in behavior.
    */
   public double tempInputValue;

   public ConstrainedWholeBodyPlanningRequestPacket()
   {

   }
   


   public void setTempValue(double value)
   {
      tempInputValue = value;
   }

   @Override
   public boolean epsilonEquals(ConstrainedWholeBodyPlanningRequestPacket other, double epsilon)
   {
      return true;
   }

}
