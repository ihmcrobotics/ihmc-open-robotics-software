package us.ihmc.communication.crdt;

import ihmc_common_msgs.msg.dds.ConfirmableRequestMessage;
import us.ihmc.commons.thread.Notification;

import java.util.UUID;

/**
 * A freezable node that has a mechanism for the freeze
 * to end early if we know the change has propagated.
 */
public class Confirmable extends Freezable
{
   private final Notification needToSendRequest = new Notification();
   private final Notification needToSendConfirmation = new Notification();

   private UUID requestUUID;

   @Override
   public void freeze()
   {
      super.freeze();
      needToSendRequest.set();

      requestUUID = UUID.randomUUID();
   }

   public void toMessage(ConfirmableRequestMessage message)
   {
      if (needToSendRequest.poll())
      {
         message.setValue(ConfirmableRequestMessage.REQUEST);
      }
      else if (needToSendConfirmation.poll())
      {
         message.setValue(ConfirmableRequestMessage.CONFIRMATION);
      }
      else
      {
         message.setValue(ConfirmableRequestMessage.NOOP);
      }
   }

   public void fromMessage(ConfirmableRequestMessage message)
   {
      if (message.getValue() == ConfirmableRequestMessage.REQUEST)
      {
         needToSendConfirmation.set();
      }
      else if (message.getValue() == ConfirmableRequestMessage.CONFIRMATION)
      {
//         unfreeze();
      }
   }
}
