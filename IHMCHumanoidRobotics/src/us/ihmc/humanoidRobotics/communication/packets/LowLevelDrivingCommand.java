package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;

public class LowLevelDrivingCommand extends Packet<LowLevelDrivingCommand>
{
   public LowLevelDrivingAction action;
   public double value;
   
   public LowLevelDrivingCommand()
   {
      
   }

   public LowLevelDrivingCommand(LowLevelDrivingAction action, double value)
   {
      this.action = action;
      this.value = value;
   }
   
   public LowLevelDrivingAction getAction()
   {
      return action;
   }
   
   public double getValue()
   {
      return value;
   }
   
   public void setAction(LowLevelDrivingAction action)
   {
      this.action = action;
   }
   
   public void setValue(double value)
   {
      this.value = value;
   }
   
   
   public boolean epsilonEquals(LowLevelDrivingCommand other, double epsilon)
   {
      return Math.abs(other.getValue() - getValue()) < epsilon && getAction() == other.getAction();
   }
   
}
