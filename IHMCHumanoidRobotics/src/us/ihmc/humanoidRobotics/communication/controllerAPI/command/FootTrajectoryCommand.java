package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootTrajectoryCommand extends SE3TrajectoryControllerCommand<FootTrajectoryCommand, FootTrajectoryMessage>
{
   private long commandId = Packet.VALID_MESSAGE_DEFAULT_ID;
   private RobotSide robotSide;
   private ExecutionMode executionMode;
   private long previousCommandId = Packet.INVALID_MESSAGE_ID;

   public FootTrajectoryCommand()
   {
   }

   @Override
   public void clear()
   {
      super.clear();

      robotSide = null;
      commandId = Packet.VALID_MESSAGE_DEFAULT_ID;
      executionMode = null;
      previousCommandId = Packet.INVALID_MESSAGE_ID;
   }

   @Override
   public void set(FootTrajectoryMessage message)
   {
      super.set(message);
      commandId = message.getUniqueId();
      this.robotSide = message.getRobotSide();
      executionMode = message.getExecutionMode();
      previousCommandId = message.getPreviousMessageId();
   }

   @Override
   public void set(FootTrajectoryCommand other)
   {
      super.set(other);
      setPropertiesOnly(other);
   }

   /**
    * Same as {@link #set(FootTrajectoryCommand)} but does not change the trajectory points.
    * @param other
    */
   public void setPropertiesOnly(FootTrajectoryCommand other)
   {
      commandId = other.commandId;
      robotSide = other.robotSide;
      executionMode = other.executionMode;
      previousCommandId = other.previousCommandId;
   }

   public void setCommandId(long commandId)
   {
      this.commandId = commandId;
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public void setExecutionMode(ExecutionMode executionMode)
   {
      this.executionMode = executionMode;
   }

   public void setPreviousCommandId(long previousCommandId)
   {
      this.previousCommandId = previousCommandId;
   }

   public long getCommandId()
   {
      return commandId;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public ExecutionMode getExecutionMode()
   {
      return executionMode;
   }

   public long getPreviousCommandId()
   {
      return previousCommandId;
   }

   @Override
   public Class<FootTrajectoryMessage> getMessageClass()
   {
      return FootTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotSide != null && executionMode != null && super.isCommandValid();
   }
}
