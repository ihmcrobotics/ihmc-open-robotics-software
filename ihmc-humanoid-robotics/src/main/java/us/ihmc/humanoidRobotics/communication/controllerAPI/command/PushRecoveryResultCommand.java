package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.PushRecoveryResultMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.QueueableCommand;

import java.util.List;

public class PushRecoveryResultCommand  extends QueueableCommand<PushRecoveryResultCommand, PushRecoveryResultMessage>
{
   private long sequenceId;
   private boolean isStepRecoverable = false;
   private final FootstepDataListCommand footstepDataListCommand = new FootstepDataListCommand();
   private final RecyclingArrayList<StepConstraintRegionCommand> stepConstraintCommand = new RecyclingArrayList<>(StepConstraintRegionCommand::new);

   @Override
   public void addTimeOffset(double timeOffset)
   {
      throw new RuntimeException("This method should not be used with push recovery result command.");
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      isStepRecoverable = false;
      footstepDataListCommand.clear();
      for (int i = 0; i < stepConstraintCommand.size(); i++)
         stepConstraintCommand.get(i).clear();
      stepConstraintCommand.clear();
      clearQueuableCommandVariables();
   }

   @Override
   public void setFromMessage(PushRecoveryResultMessage message)
   {
      sequenceId = message.getSequenceId();
      isStepRecoverable = message.getIsStepRecoverable();
      footstepDataListCommand.setFromMessage(message.getRecoverySteps());
      stepConstraintCommand.clear();
      for (int i = 0; i < message.getStepConstraintList().size(); i++)
      {
         stepConstraintCommand.add().setFromMessage(message.getStepConstraintList().get(i));
      }
   }

   @Override
   public Class<PushRecoveryResultMessage> getMessageClass()
   {
      return PushRecoveryResultMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      if (isStepRecoverable)
      {
         if (!footstepDataListCommand.isCommandValid())
            return false;
         if (!stepConstraintCommand.isEmpty())
         {
            if (stepConstraintCommand.size() != footstepDataListCommand.getNumberOfFootsteps())
               return false;
            for (int i = 0; i < stepConstraintCommand.size(); i++)
            {
               if (!stepConstraintCommand.get(i).isCommandValid())
                  return false;
            }
         }
      }
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

   @Override
   public void set(PushRecoveryResultCommand other)
   {
      sequenceId = other.sequenceId;
      stepConstraintCommand.clear();
      for (int i = 0; i < other.stepConstraintCommand.size(); i++)
         stepConstraintCommand.add().set(other.stepConstraintCommand.get(i));
      footstepDataListCommand.set(other.footstepDataListCommand);
      isStepRecoverable = other.isStepRecoverable;
   }

   public boolean isStepRecoverable()
   {
      return isStepRecoverable;
   }

   public FootstepDataListCommand getRecoverySteps()
   {
      return footstepDataListCommand;
   }

   public List<StepConstraintRegionCommand> getStepConstraintRegions()
   {
      return stepConstraintCommand;
   }
}
