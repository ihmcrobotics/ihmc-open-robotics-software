package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.QuadrupedTimedStepMessage;
import controller_msgs.msg.dds.TimeIntervalMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedTimedStepCommand implements Command<QuadrupedTimedStepCommand, QuadrupedTimedStepMessage>
{
   private final QuadrupedStepCommand stepCommand = new QuadrupedStepCommand();
   private final TimeIntervalCommand timeIntervalCommand = new TimeIntervalCommand();

   public QuadrupedTimedStepCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      stepCommand.clear();
      timeIntervalCommand.clear();
   }

   @Override
   public void set(QuadrupedTimedStepMessage message)
   {
      stepCommand.set(message.getQuadrupedStepMessage());
      timeIntervalCommand.set(message.getTimeInterval());
   }

   @Override
   public void set(QuadrupedTimedStepCommand other)
   {
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
}
