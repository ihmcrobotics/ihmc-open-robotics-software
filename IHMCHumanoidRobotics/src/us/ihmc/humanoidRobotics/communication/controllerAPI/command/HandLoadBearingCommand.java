package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandLoadBearingMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandLoadBearingCommand extends AbstractLoadBearingCommand<HandLoadBearingCommand, HandLoadBearingMessage>
{
   private RobotSide robotSide;

   private boolean useJointspaceCommand = false;

   private ArmTrajectoryCommand armTrajectoryCommand = new ArmTrajectoryCommand();
   
   /** the time to delay this command on the controller side before being executed **/
   private double executionDelayTime;
   /** the execution time. This number is set if the execution delay is non zero**/
   public double adjustedExecutionTime;

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   @Override
   public void set(HandLoadBearingCommand other)
   {
      super.set(other);
      robotSide = other.robotSide;
      executionDelayTime = other.getExecutionDelayTime();
      useJointspaceCommand = other.isUseJointspaceCommand();
      armTrajectoryCommand.set(other.getArmTrajectoryCommand());
   }

   @Override
   public void set(HandLoadBearingMessage message)
   {
      super.set(message);
      executionDelayTime = message.executionDelayTime;
      robotSide = message.robotSide;
      useJointspaceCommand = message.isUseJointspaceCommand();
      if (message.getArmTrajectoryMessage() != null)
      {
         armTrajectoryCommand.set(message.getArmTrajectoryMessage());
      }
   }

   public ArmTrajectoryCommand getArmTrajectoryCommand()
   {
      return armTrajectoryCommand;
   }

   public boolean isUseJointspaceCommand()
   {
      return useJointspaceCommand;
   }

   @Override
   public void clear()
   {
      super.clear();
      robotSide = null;
      useJointspaceCommand = false;
      armTrajectoryCommand.clear();
   }

   @Override
   public Class<HandLoadBearingMessage> getMessageClass()
   {
      return HandLoadBearingMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      boolean armTrajectoryValid = true;
      if (useJointspaceCommand && armTrajectoryCommand == null)
      {
         armTrajectoryValid = false;
      }
      else if (useJointspaceCommand)
      {
         armTrajectoryValid = armTrajectoryCommand.isCommandValid();
      }

      return armTrajectoryValid && robotSide != null && super.isCommandValid();
   }
   
   /**
    * returns the amount of time this command is delayed on the controller side before executing
    * @return the time to delay this command in seconds
    */
   @Override
   public double getExecutionDelayTime()
   {
      return executionDelayTime;
   }
   
   /**
    * sets the amount of time this command is delayed on the controller side before executing
    * @param delayTime the time in seconds to delay after receiving the command before executing
    */
   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      this.executionDelayTime = delayTime;
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
}
