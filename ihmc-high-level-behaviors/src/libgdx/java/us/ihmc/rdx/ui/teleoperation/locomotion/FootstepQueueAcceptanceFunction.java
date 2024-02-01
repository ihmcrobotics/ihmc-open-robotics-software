package us.ihmc.rdx.ui.teleoperation.locomotion;

import controller_msgs.msg.dds.FootstepQueueStatusMessage;

import java.util.function.BiFunction;

/**
 * The full epsilonEquals method doesn't work because there are some values like
 * time_in_support_sequence_ that are continuously changing.
 */
public class FootstepQueueAcceptanceFunction implements BiFunction<FootstepQueueStatusMessage, FootstepQueueStatusMessage, Boolean>
{
   @Override
   public Boolean apply(FootstepQueueStatusMessage previousMessage, FootstepQueueStatusMessage newMessage)
   {
      if (newMessage == null)
         return false;
      if (previousMessage == null)
         return true;
      if (previousMessage == newMessage)
         return false;

      // Copying from FootstepQueueStatusMessage#epsilonEquals

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(previousMessage.is_first_step_in_swing_, newMessage.is_first_step_in_swing_, 1e-3))
         return true;

      if (previousMessage.queued_footstep_list_.size() != newMessage.queued_footstep_list_.size())
      {
         return true;
      }
      else
      {
         for (int i = 0; i < previousMessage.queued_footstep_list_.size(); i++)
         {
            if (!previousMessage.queued_footstep_list_.get(i).epsilonEquals(newMessage.queued_footstep_list_.get(i), 1e-3))
               return true;
         }
      }

      return false;
   }
}
