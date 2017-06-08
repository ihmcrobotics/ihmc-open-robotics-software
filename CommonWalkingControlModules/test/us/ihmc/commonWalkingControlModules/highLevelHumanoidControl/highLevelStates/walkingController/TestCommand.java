package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.walking.AbortWalkingMessage;

public class TestCommand implements Command<TestCommand,AbortWalkingMessage>
{
   public double delayTime;
   public long data;
   
   
   public TestCommand()
   {
      
   }
   
   @Override
   public void set(TestCommand other)
   {
      this.data = other.data;
      this.delayTime = other.delayTime;
   }

   @Override
   public void clear()
   {
      delayTime = 0;
      data = 0;
   }

   @Override
   public void set(AbortWalkingMessage message)
   {
      
   }

   @Override
   public Class<AbortWalkingMessage> getMessageClass()
   {
      return AbortWalkingMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public double getExecutionDelayTime()
   {
      return delayTime;
   }

   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      this.delayTime = delayTime;
   }
   
   public long getData()
   {
      return data;
   }
   
   public void setData(long data)
   {
      this.data = data;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (this == obj)
         return true;
      if (obj == null)
         return false;
      if (getClass() != obj.getClass())
         return false;
      TestCommand other = (TestCommand) obj;
      if (data != other.data)
         return false;
      if (Double.doubleToLongBits(delayTime) != Double.doubleToLongBits(other.delayTime))
         return false;
      return true;
   }
}
