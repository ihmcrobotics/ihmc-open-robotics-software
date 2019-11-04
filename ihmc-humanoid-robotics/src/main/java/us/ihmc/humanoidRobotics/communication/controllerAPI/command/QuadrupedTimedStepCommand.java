package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.QuadrupedTimedStepMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.trajectories.TrajectoryType;

public class QuadrupedTimedStepCommand implements Command<QuadrupedTimedStepCommand, QuadrupedTimedStepMessage>
{
   private long sequenceId;
   private final QuadrupedStepCommand stepCommand = new QuadrupedStepCommand();
   private final TimeIntervalCommand timeIntervalCommand = new TimeIntervalCommand();

   public QuadrupedTimedStepCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      stepCommand.clear();
      timeIntervalCommand.clear();
   }

   @Override
   public void setFromMessage(QuadrupedTimedStepMessage message)
   {
      sequenceId = message.getSequenceId();
      stepCommand.setFromMessage(message.getQuadrupedStepMessage());
      timeIntervalCommand.setFromMessage(message.getTimeInterval());
   }

   @Override
   public void set(QuadrupedTimedStepCommand other)
   {
      sequenceId = other.sequenceId;
      stepCommand.set(other.stepCommand);
      timeIntervalCommand.set(other.timeIntervalCommand);
   }

   public QuadrupedStepCommand getStepCommand()
   {
      return stepCommand;
   }

   public TimeIntervalCommand getTimeIntervalCommand()
   {
      return timeIntervalCommand;
   }

   public RobotQuadrant getRobotQuadrant()
   {
      return stepCommand.getRobotQuadrant();
   }

   public double getGroundClearance()
   {
      return stepCommand.getGroundClearance();
   }

   public FramePoint3D getGoalPosition()
   {
      return stepCommand.getGoalPosition();
   }
   
   public TrajectoryType getTrajectoryType()
   {
      return stepCommand.getTrajectoryType();
   }

   public double getStartTime()
   {
      return timeIntervalCommand.getStartTime();
   }

   public double getEndTime()
   {
      return timeIntervalCommand.getEndTime();
   }

   @Override
   public Class<QuadrupedTimedStepMessage> getMessageClass()
   {
      return QuadrupedTimedStepMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return stepCommand.isCommandValid() && timeIntervalCommand.isCommandValid();
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
