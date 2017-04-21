package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class PelvisHeightTrajectoryCommand extends SimpleTrajectoryPoint1DList implements Command<PelvisHeightTrajectoryCommand, PelvisHeightTrajectoryMessage>
{
   private long commandId = Packet.VALID_MESSAGE_DEFAULT_ID;
   private ExecutionMode executionMode = ExecutionMode.OVERRIDE;
   private long previousCommandId = Packet.INVALID_MESSAGE_ID;

   public PelvisHeightTrajectoryCommand()
   {
   }

   @Override
   public void clear()
   {
      super.clear();

      commandId = Packet.VALID_MESSAGE_DEFAULT_ID;
      executionMode = ExecutionMode.OVERRIDE;
      previousCommandId = Packet.INVALID_MESSAGE_ID;
   }

   @Override
   public void set(PelvisHeightTrajectoryCommand other)
   {
      super.set(other);

      setPropertiesOnly(other);
   }

   /**
    * Same as {@link #set(PelvisHeightTrajectoryCommand)} but does not change the trajectory points.
    * 
    * @param other
    */
   public void setPropertiesOnly(PelvisHeightTrajectoryCommand other)
   {
      commandId = other.commandId;
      executionMode = other.executionMode;
      previousCommandId = other.previousCommandId;
   }

   @Override
   public void set(PelvisHeightTrajectoryMessage message)
   {
      message.getTrajectoryPoints(this);
      commandId = message.getUniqueId();
      executionMode = message.getExecutionMode();
      previousCommandId = message.getPreviousMessageId();
   }

   public void set(PelvisTrajectoryCommand command)
   {
      command.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      clear();

      for (int i = 0; i < command.getNumberOfTrajectoryPoints(); i++)
      {
         FrameSE3TrajectoryPoint trajectoryPoint = command.getTrajectoryPoint(i);
         double time = trajectoryPoint.getTime();
         double position = trajectoryPoint.getPositionZ();
         double velocity = trajectoryPoint.getLinearVelocityZ();
         addTrajectoryPoint(time, position, velocity);
      }

      commandId = command.getCommandId();
      executionMode = command.getExecutionMode();
      previousCommandId = command.getPreviousCommandId();
   }

   public void setExecutionMode(ExecutionMode executionMode)
   {
      this.executionMode = executionMode;
   }

   @Override
   public Class<PelvisHeightTrajectoryMessage> getMessageClass()
   {
      return PelvisHeightTrajectoryMessage.class;
   }

   public long getCommandId()
   {
      return commandId;
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
   public boolean isCommandValid()
   {
      return getNumberOfTrajectoryPoints() > 0;
   }
}
