package us.ihmc.rdx.ui.teleoperation.locomotion;

import controller_msgs.FootstepQueueStatusMessage;
import controller_msgs.QueuedFootstepStatusMessage;

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

      if (previousMessage.getIsFirstStepInSwing() != newMessage.getIsFirstStepInSwing())
         return true;

      if (previousMessage.getQueuedFootstepList().size() != newMessage.getQueuedFootstepList().size())
      {
         return true;
      }
      else
      {
         for (int i = 0; i < previousMessage.getQueuedFootstepList().size(); i++)
         {
            if (!queuedFootstepsEqual(previousMessage.getQueuedFootstepList().get(i), newMessage.getQueuedFootstepList().get(i)))
               return true;
         }
      }

      return false;
   }

   private static boolean queuedFootstepsEqual(QueuedFootstepStatusMessage previous, QueuedFootstepStatusMessage current)
   {
      QueuedFootstepStatusMessage temp = new QueuedFootstepStatusMessage(previous);
      return temp.getRobotSide() == current.getRobotSide()
          && Math.abs(temp.getSwingDuration() - current.getSwingDuration()) < 1e-3
          && Math.abs(temp.getTransferDuration() - current.getTransferDuration()) < 1e-3;
   }
}
