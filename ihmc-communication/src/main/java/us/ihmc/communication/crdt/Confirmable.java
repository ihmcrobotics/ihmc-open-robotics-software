package us.ihmc.communication.crdt;

import ihmc_common_msgs.msg.dds.ConfirmableRequestMessage;
import org.apache.commons.lang3.mutable.MutableLong;
import us.ihmc.commons.thread.Notification;
import us.ihmc.log.LogTools;

/**
 * A freezable node that has a mechanism for the freeze
 * to end early if we know the change has propagated.
 *
 * We assume messages are not dropped and do not arrive out of order.
 */
public class Confirmable extends Freezable
{
   private static final MutableLong nextRequestID = new MutableLong();

   private final Notification needToSendRequest = new Notification();
   private final Notification needToSendConfirmation = new Notification();
   private final CRDTInfo crdtInfo;

   private long requestNumber;
   private long updateNumberToUnfreeze = 0;

   public Confirmable(CRDTInfo crdtInfo)
   {
      this.crdtInfo = crdtInfo;
   }

   @Override
   public void freeze()
   {
      super.freeze();

      updateNumberToUnfreeze = crdtInfo.getUpdateNumber() + crdtInfo.getMaxFreezeDuration();

      if (!needToSendRequest.peek()) // Avoid sending multiple requests in one tick
      {
         needToSendRequest.set();

         requestNumber = nextRequestID.getAndIncrement();
//         LogTools.info("Request: {}:{} Actor: {}", this.getClass().getSimpleName(), requestNumber, crdtInfo.getActorDesignation().name());
      }
   }

   @Override
   public void unfreeze()
   {
      updateNumberToUnfreeze = crdtInfo.getUpdateNumber();
   }

   @Override
   public boolean isFrozen()
   {
      return crdtInfo.getUpdateNumber() < updateNumberToUnfreeze;
   }

   public void toMessage(ConfirmableRequestMessage message)
   {
      if (needToSendRequest.poll())
      {
         message.setValue(ConfirmableRequestMessage.REQUEST);
         message.setRequestNumber(requestNumber);
      }
      else if (needToSendConfirmation.poll())
      {
         message.setValue(ConfirmableRequestMessage.CONFIRMATION);
         message.setRequestNumber(requestNumber);

//         LogTools.info("Confirming: {}:{} Actor: {}", this.getClass().getSimpleName(), requestNumber, crdtInfo.getActorDesignation().name());
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
         requestNumber = message.getRequestNumber();
      }
      else if (message.getValue() == ConfirmableRequestMessage.CONFIRMATION)
      {
         long confirmationsRequestUUID = message.getRequestNumber();
         if (confirmationsRequestUUID == requestNumber)
         {
//            LogTools.info("Confirmed: {}:{} Actor: {}", this.getClass().getSimpleName(), confirmationsRequestUUID, crdtInfo.getActorDesignation().name());
            unfreeze();
         }
         else
         {
//            LogTools.error("Received a different request ID than sent. Sent: {} Recieved: {}", requestNumber, confirmationsRequestUUID);
         }
      }
   }

   public CRDTInfo getCRDTInfo()
   {
      return crdtInfo;
   }
}
