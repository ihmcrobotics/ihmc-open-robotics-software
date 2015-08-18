package us.ihmc.communication.packets;

import us.ihmc.communication.packets.Packet;

public class LowLevelDrivingStatus extends Packet<LowLevelDrivingStatus>
{
   public LowLevelDrivingAction action;
   public boolean success;
   
   public LowLevelDrivingStatus()
   {
      
   }
   
   public LowLevelDrivingStatus(LowLevelDrivingAction action, boolean success)
   {
      this.action = action;
      this.success = success;
   }

   public LowLevelDrivingAction getAction()
   {
      return action;
   }

   public boolean isSuccess()
   {
      return success;
   }

   public void setAction(LowLevelDrivingAction action)
   {
      this.action = action;
   }

   public void setSuccess(boolean success)
   {
      this.success = success;
   }
   
   public boolean equals(Object other)
   {
      if(other instanceof LowLevelDrivingStatus)
      {
         return isSuccess() == ((LowLevelDrivingStatus) other).isSuccess() && getAction() == ((LowLevelDrivingStatus) other).getAction();
      }
      else
      {
         return false;
      }
   }

   @Override
   public boolean epsilonEquals(LowLevelDrivingStatus other, double epsilon)
   {
      return equals(other);
   }
   
   
}
