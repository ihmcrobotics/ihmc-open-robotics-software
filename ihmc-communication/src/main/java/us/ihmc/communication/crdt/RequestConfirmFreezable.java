package us.ihmc.communication.crdt;

import gnu.trove.set.hash.TLongHashSet;
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
public class RequestConfirmFreezable implements Freezable
{
   private final CRDTInfo crdtInfo;
   private final MutableLong nextRequestID = new MutableLong();
   private final Notification needToSendRequest = new Notification();
   private final Notification needToSendConfirmation = new Notification();
   private final TLongHashSet unconfirmedRequests = new TLongHashSet();
   private long confirmationNumber;
   private long updateNumberToUnfreeze = 0;
   private boolean isFrozen = false;

   public RequestConfirmFreezable(CRDTInfo crdtInfo)
   {
      this.crdtInfo = crdtInfo;
   }

   @Override
   public void freeze()
   {
      if (!isFrozen)
         LogTools.debug(1, "Freezing: %s  Actor: %s".formatted(this.getClass().getSimpleName(), crdtInfo.getActorDesignation()));
      isFrozen = true;

      updateNumberToUnfreeze = crdtInfo.getUpdateNumber() + crdtInfo.getMaxFreezeDuration();

      needToSendRequest.set();
   }

   @Override
   public void unfreeze()
   {
      updateNumberToUnfreeze = crdtInfo.getUpdateNumber();
   }

   @Override
   public boolean isFrozen()
   {
      boolean isFrozen = crdtInfo.getUpdateNumber() < updateNumberToUnfreeze;

      if (isFrozen != this.isFrozen)
         LogTools.debug("Frozen %b -> %b %s Actor: %s".formatted(this.isFrozen, isFrozen, this.getClass().getSimpleName(), crdtInfo.getActorDesignation()));

      this.isFrozen = isFrozen;

      return isFrozen;
   }

   public void toMessage(ConfirmableRequestMessage message)
   {
      message.setIsRequest(false);
      message.setIsConfirmation(false);

      if (needToSendRequest.poll())
      {
         long requestNumber = nextRequestID.incrementAndGet();
         LogTools.debug("Request: {}:{} Actor: {}", this.getClass().getSimpleName(), requestNumber, crdtInfo.getActorDesignation().name());
         message.setIsRequest(true);
         message.setRequestNumber(requestNumber);
         unconfirmedRequests.add(requestNumber);
      }

      if (needToSendConfirmation.poll())
      {
         LogTools.debug("Confirming: {}:{} Actor: {}", this.getClass().getSimpleName(), confirmationNumber, crdtInfo.getActorDesignation().name());
         message.setIsConfirmation(true);
         message.setConfirmationNumber(confirmationNumber);
      }
   }

   public void fromMessage(ConfirmableRequestMessage message)
   {
      if (message.getIsRequest())
      {
         confirmationNumber = message.getRequestNumber();
         needToSendConfirmation.set();
      }

      if (message.getIsConfirmation())
      {
         long confirmationsRequestUUID = message.getConfirmationNumber();

         boolean removed = unconfirmedRequests.remove(confirmationsRequestUUID);

         if (removed)
         {
            LogTools.debug("Confirmed: {}:{} Actor: {}", this.getClass().getSimpleName(), confirmationsRequestUUID, crdtInfo.getActorDesignation().name());
            if (!unconfirmedRequests.isEmpty())
               LogTools.debug("Still unconfirmed requests: {}", unconfirmedRequests);

            unfreeze();
         }
         else
         {
            LogTools.error("Received a different request ID than sent. Sent: {} Recieved: {}", unconfirmedRequests, confirmationsRequestUUID);
         }
      }
   }

   public CRDTInfo getCRDTInfo()
   {
      return crdtInfo;
   }
}
