package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.PushRecoveryResultMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.QueueableCommand;

public class PushRecoveryResultCommand  extends QueueableCommand<PushRecoveryResultCommand, PushRecoveryResultMessage>
{
   private long sequenceId;
   private boolean isStepRecoverable = false;
   private final FootstepDataListCommand footstepDataListCommand = new FootstepDataListCommand();
   private final RecyclingArrayList<PlanarRegionsListCommand> planarRegionsListCommand = new RecyclingArrayList<>(PlanarRegionsListCommand::new);

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
      for (int i = 0; i < planarRegionsListCommand.size(); i++)
         planarRegionsListCommand.get(i).clear();
      planarRegionsListCommand.clear();
      clearQueuableCommandVariables();
   }

   @Override
   public void setFromMessage(PushRecoveryResultMessage message)
   {
//      sequenceId = message.getSequenceId();
      isStepRecoverable = message.getIsStepRecoverable();
      footstepDataListCommand.setFromMessage(message.getRecoverySteps());
      planarRegionsListCommand.clear();
      for (int i = 0; i < message.getStepConstraintList().size(); i++)
      {
         planarRegionsListCommand.add().setFromMessage(message.getStepConstraintList().get(i));
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
         if (!planarRegionsListCommand.isEmpty())
         {
            if (planarRegionsListCommand.size() != footstepDataListCommand.getNumberOfFootsteps())
               return false;
            for (int i = 0; i < planarRegionsListCommand.size(); i++)
            {
               if (planarRegionsListCommand.get(i).getNumberOfPlanarRegions() < 1 || !planarRegionsListCommand.get(i).isCommandValid())
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
      planarRegionsListCommand.clear();
      for (int i = 0; i < other.planarRegionsListCommand.size(); i++)
         planarRegionsListCommand.add().set(other.planarRegionsListCommand.get(i));
      footstepDataListCommand.set(other.footstepDataListCommand);
      isStepRecoverable = other.isStepRecoverable;
   }

   public boolean isStepRecoverable()
   {
      return isStepRecoverable;
   }
}
