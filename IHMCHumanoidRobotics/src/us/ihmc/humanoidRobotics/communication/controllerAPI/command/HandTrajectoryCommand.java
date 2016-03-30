package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage.BaseForControl;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandTrajectoryCommand extends SE3TrajectoryControllerCommand<HandTrajectoryCommand, HandTrajectoryMessage>
{
   private long commandId = Packet.VALID_MESSAGE_DEFAULT_ID;
   private RobotSide robotSide;
   private BaseForControl baseForControl;
   private ExecutionMode executionMode;
   private long previousCommandId = Packet.INVALID_MESSAGE_ID;

   public HandTrajectoryCommand()
   {
   }

   public HandTrajectoryCommand(ReferenceFrame referenceFrame, RobotSide robotSide, BaseForControl baseForControl)
   {
      super.clear(referenceFrame);
      this.robotSide = robotSide;
      this.baseForControl = baseForControl;
   }

   @Override
   public void clear()
   {
      super.clear();

      commandId = Packet.VALID_MESSAGE_DEFAULT_ID;
      robotSide = null;
      baseForControl = null;
      executionMode = null;
      previousCommandId = Packet.INVALID_MESSAGE_ID;
   }

   @Override
   public void clear(ReferenceFrame referenceFrame)
   {
      super.clear(referenceFrame);

      commandId = Packet.VALID_MESSAGE_DEFAULT_ID;
      robotSide = null;
      baseForControl = null;
      executionMode = null;
      previousCommandId = Packet.INVALID_MESSAGE_ID;
   }

   @Override
   public void set(HandTrajectoryCommand other)
   {
      super.set(other);
      setPropertiesOnly(other);
   }

   /**
    * Same as {@link #set(HandTrajectoryCommand)} but does not change the trajectory points.
    * @param other
    */
   public void setPropertiesOnly(HandTrajectoryCommand other)
   {
      commandId = other.commandId;
      robotSide = other.robotSide;
      baseForControl = other.baseForControl;
      executionMode = other.executionMode;
      previousCommandId = other.previousCommandId;
   }

   @Override
   public void set(HandTrajectoryMessage message)
   {
      super.set(message);
      commandId = message.getUniqueId();
      this.robotSide = message.getRobotSide();
      this.baseForControl = message.getBase();
      executionMode = message.getExecutionMode();
      previousCommandId = message.getPreviousMessageId();
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public void setBase(BaseForControl baseForControl)
   {
      this.baseForControl = baseForControl;
   }

   public void setCommandId(long commandId)
   {
      this.commandId = commandId;
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

   public BaseForControl getBase()
   {
      return baseForControl;
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
   public Class<HandTrajectoryMessage> getMessageClass()
   {
      return HandTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotSide != null && baseForControl != null && executionMode != null && super.isCommandValid();
   }
}
