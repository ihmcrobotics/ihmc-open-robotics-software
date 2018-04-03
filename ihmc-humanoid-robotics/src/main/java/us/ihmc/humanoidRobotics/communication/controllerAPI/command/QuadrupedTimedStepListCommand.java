package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.QuadrupedTimedStepListMessage;
import controller_msgs.msg.dds.QuadrupedTimedStepMessage;
import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.robotics.lists.RecyclingArrayList;

import java.util.List;

public class QuadrupedTimedStepListCommand extends QueueableCommand<QuadrupedTimedStepListCommand, QuadrupedTimedStepListMessage>
{
   private boolean isExpressedInAbsoluteTime;
   private final RecyclingArrayList<QuadrupedTimedStepCommand> stepCommands = new RecyclingArrayList<>(30, QuadrupedTimedStepCommand.class);

   public QuadrupedTimedStepListCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      isExpressedInAbsoluteTime = true;
      stepCommands.clear();
      clearQueuableCommandVariables();
   }

   @Override
   public void set(QuadrupedTimedStepListMessage message)
   {
      clear();

      isExpressedInAbsoluteTime = message.getIsExpressedInAbsoluteTime();
      List<QuadrupedTimedStepMessage> stepList = message.getQuadrupedStepList();
      if (stepList != null)
      {
         for (int i = 0; i < stepList.size(); i++)
            stepCommands.add().set(stepList.get(i));
      }

      setQueueableCommandVariables(message.getQueueingProperties());
   }

   @Override
   public void set(QuadrupedTimedStepListCommand other)
   {
      clear();

      isExpressedInAbsoluteTime = other.isExpressedInAbsoluteTime;
      RecyclingArrayList<QuadrupedTimedStepCommand> otherFootsteps = other.getStepCommands();
      if (otherFootsteps != null)
      {
         for (int i = 0; i < otherFootsteps.size(); i++)
            stepCommands.add().set(otherFootsteps.get(i));
      }
      setQueueableCommandVariables(other);
   }

   public RecyclingArrayList<QuadrupedTimedStepCommand> getStepCommands()
   {
      return stepCommands;
   }

   public int getNumberOfSteps()
   {
      return stepCommands.size();
   }

   @Override
   public Class<QuadrupedTimedStepListMessage> getMessageClass()
   {
      return QuadrupedTimedStepListMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return getNumberOfSteps() > 0 && executionModeValid();
   }

   @Override
   public void addTimeOffset(double timeOffset)
   {
      for (int i = 0; i < getNumberOfSteps(); i++)
      {
         stepCommands.get(i).getTimeIntervalCommand().shiftTimeInterval(timeOffset);
      }
   }
}
