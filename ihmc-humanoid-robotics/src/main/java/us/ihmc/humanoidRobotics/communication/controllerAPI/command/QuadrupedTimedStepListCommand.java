package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.QuadrupedTimedStepListMessage;
import controller_msgs.msg.dds.QuadrupedTimedStepMessage;
import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.commons.lists.RecyclingArrayList;

import java.util.List;

public class QuadrupedTimedStepListCommand extends QueueableCommand<QuadrupedTimedStepListCommand, QuadrupedTimedStepListMessage>
{
   private long sequenceId;
   private boolean isExpressedInAbsoluteTime;
   private boolean isStepPlanAdjustable;
   private final RecyclingArrayList<QuadrupedTimedStepCommand> stepCommands = new RecyclingArrayList<>(30, QuadrupedTimedStepCommand.class);

   public QuadrupedTimedStepListCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      isExpressedInAbsoluteTime = true;
      isStepPlanAdjustable = true;
      stepCommands.clear();
      clearQueuableCommandVariables();
   }

   @Override
   public void setFromMessage(QuadrupedTimedStepListMessage message)
   {
      clear();
      sequenceId = message.getSequenceId();

      isExpressedInAbsoluteTime = message.getIsExpressedInAbsoluteTime();
      isStepPlanAdjustable = message.getAreStepsAdjustable();
      List<QuadrupedTimedStepMessage> stepList = message.getQuadrupedStepList();
      if (stepList != null)
      {
         for (int i = 0; i < stepList.size(); i++)
            stepCommands.add().setFromMessage(stepList.get(i));
      }

      setQueueableCommandVariables(message.getQueueingProperties());
   }

   @Override
   public void set(QuadrupedTimedStepListCommand other)
   {
      clear();

      sequenceId = other.sequenceId;
      isExpressedInAbsoluteTime = other.isExpressedInAbsoluteTime;
      isStepPlanAdjustable = other.isStepPlanAdjustable;
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

   public boolean isExpressedInAbsoluteTime()
   {
      return isExpressedInAbsoluteTime;
   }

   public boolean isStepPlanAdjustable()
   {
      return isStepPlanAdjustable;
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

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
