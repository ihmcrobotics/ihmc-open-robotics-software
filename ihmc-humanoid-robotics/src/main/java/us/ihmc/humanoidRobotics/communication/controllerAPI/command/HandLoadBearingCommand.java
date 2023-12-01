package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.HandLoadBearingMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandLoadBearingCommand implements Command<HandLoadBearingCommand, HandLoadBearingMessage>
{
   private long sequenceId;
   private RobotSide robotSide;
   private boolean useJointspaceCommand = true;

   private final JointspaceTrajectoryCommand jointspaceTrajectory = new JointspaceTrajectoryCommand();
   private final SO3TrajectoryControllerCommand orientationTrajectory = new SO3TrajectoryControllerCommand();

   /** the time to delay this command on the controller side before being executed **/
   private double executionDelayTime;
   /** the execution time. This number is set if the execution delay is non zero**/
   public double adjustedExecutionTime;

   private final LoadBearingCommand loadBearingCommand = new LoadBearingCommand();

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   @Override
   public void set(HandLoadBearingCommand other)
   {
      sequenceId = other.sequenceId;
      loadBearingCommand.set(other.loadBearingCommand);
      robotSide = other.robotSide;
      executionDelayTime = other.getExecutionDelayTime();
      useJointspaceCommand = other.useJointspaceCommand();
      jointspaceTrajectory.set(other.getJointspaceTrajectory());
      orientationTrajectory.set(other.getOrientationTrajectory());
   }

   @Override
   public void setFromMessage(HandLoadBearingMessage message)
   {
      sequenceId = message.getSequenceId();
      loadBearingCommand.setFromMessage(message.getLoadBearingMessage());
      executionDelayTime = message.getExecutionDelayTime();
      robotSide = RobotSide.fromByte(message.getRobotSide());
      useJointspaceCommand = message.getUseJointspaceCommand();
      jointspaceTrajectory.setFromMessage(message.getJointspaceTrajectory());
      orientationTrajectory.setFromMessage(message.getOrientationTrajectory());
   }

   public JointspaceTrajectoryCommand getJointspaceTrajectory()
   {
      return jointspaceTrajectory;
   }

   public SO3TrajectoryControllerCommand getOrientationTrajectory()
   {
      return orientationTrajectory;
   }

   public boolean useJointspaceCommand()
   {
      return useJointspaceCommand;
   }

   public LoadBearingCommand getLoadBearingCommand()
   {
      return loadBearingCommand;
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      loadBearingCommand.clear();
      robotSide = null;
      useJointspaceCommand = true;
      jointspaceTrajectory.clear();
      orientationTrajectory.clear();
   }

   @Override
   public Class<HandLoadBearingMessage> getMessageClass()
   {
      return HandLoadBearingMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      boolean armTrajectoryValid;

      if (useJointspaceCommand)
      {
         armTrajectoryValid = !jointspaceTrajectory.getTrajectoryPointLists().isEmpty() && jointspaceTrajectory.isCommandValid();
      }
      else
      {
         armTrajectoryValid = orientationTrajectory.getTrajectoryPointList().getNumberOfTrajectoryPoints() > 0 && orientationTrajectory.isCommandValid();
      }

      return armTrajectoryValid && robotSide != null && loadBearingCommand.isCommandValid();
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

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
