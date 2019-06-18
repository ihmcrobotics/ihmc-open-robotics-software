package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController;

import controller_msgs.msg.dds.AbortWalkingMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class TestCommand implements Command<TestCommand,AbortWalkingMessage>
{
   public double delayTime;
   public long data;
   /** the execution time. This number is set if the execution delay is non zero**/
   public double adjustedExecutionTime;
   
   
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
   public void setFromMessage(AbortWalkingMessage message)
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
   
   /**
    * returns the expected execution time of this command. The execution time will be computed when the controller 
    * receives the command using the controllers time plus the execution delay time.
    * This is used when {@code getExecutionDelayTime} is non-zero
    */
   @Override
   public double getExecutionTime()
   {
      return adjustedExecutionTime;
   }

   /**
    * sets the execution time for this command. This is called by the controller when the command is received.
    */
   @Override
   public void setExecutionTime(double adjustedExecutionTime)
   {
      this.adjustedExecutionTime = adjustedExecutionTime;
   }
   
   /**
    * tells the controller if this command supports delayed execution
    * (Spoiler alert: It does)
    * @return
    */
   @Override
   public boolean isDelayedExecutionSupported()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return 0;
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
