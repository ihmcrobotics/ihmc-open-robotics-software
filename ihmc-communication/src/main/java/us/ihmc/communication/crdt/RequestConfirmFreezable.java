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
 * <p>
 * We assume messages are not dropped and do not arrive out of order.
 * This is to separate the concerns of this algorithm and the
 * underlying communication protocol.
 * </p>
 *
 * <p>
 * A call to {@link #freeze} initiates incrementing and sending out a
 * monotonically increasing
 * request number. Another node receives that and sends back the same
 * number, calling it the confirmation number. The initial sender
 * stores a set of all the unconfirmed request numbers. As it recieves
 * a confirmation number that matches a request it sent out, it removes
 * it from the set and calls {@link #unfreeze}.
 * </p>
 *
 * <p>
 * A singleton CRDTInfo object exists for a node in the CRDT graph
 * which has an update number that is 0 for the first update and
 * monotonically increases on each subsequent update. In this class
 * that update number is used to "timeout" the freeze if a confirmation
 * is not recieved within the max freeze duration.
 * </p>
 *
 * <p>
 * The {@link #unfreeze} method works pulling back that timeout to
 * the current update number.
 * </p>
 *
 * <p>
 * TODO: This class currently doesn't work with more than two nodes
 *   in the CRDT graph.
 * </p>
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
      updateNumberToUnfreeze = crdtInfo.getUpdateNumber() + crdtInfo.getMaxFreezeDuration();

      if (!isFrozen)
         LogTools.debug(1, "%s Update #%d: Freezing: %s until Update #%d".formatted(crdtInfo.getActorDesignation(),
                                                                                    crdtInfo.getUpdateNumber(),
                                                                                    this.getClass().getSimpleName(),
                                                                                    updateNumberToUnfreeze));
      isFrozen = true;

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
         LogTools.debug("%s Update #%d: Frozen %b -> %b %s".formatted(crdtInfo.getActorDesignation(),
                                                                      crdtInfo.getUpdateNumber(),
                                                                      this.isFrozen,
                                                                      isFrozen,
                                                                      this.getClass().getSimpleName()));

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
         LogTools.debug("%s Update #%d: %s requesting. Request #%d".formatted(crdtInfo.getActorDesignation(),
                                                                              crdtInfo.getUpdateNumber(),
                                                                              this.getClass().getSimpleName(),
                                                                              requestNumber));
         message.setIsRequest(true);
         message.setRequestNumber(requestNumber);
         unconfirmedRequests.add(requestNumber);
      }

      if (needToSendConfirmation.poll())
      {
         LogTools.debug("%s Update #%d: %s confirming. Request #%d".formatted(crdtInfo.getActorDesignation(),
                                                                              crdtInfo.getUpdateNumber(),
                                                                              this.getClass().getSimpleName(),
                                                                              confirmationNumber));
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
         nextRequestID.setValue(confirmationNumber + 1); // Avoid duplicate request numbers for clarity
      }

      if (message.getIsConfirmation())
      {
         long confirmedRequestNumber = message.getConfirmationNumber();

         boolean removed = unconfirmedRequests.remove(confirmedRequestNumber);

         if (removed)
         {
            LogTools.debug("%s Update #%d: %s recieved confirmation. Unfreezing. Request #%d".formatted(crdtInfo.getActorDesignation(),
                                                                                                        crdtInfo.getUpdateNumber(),
                                                                                                        this.getClass().getSimpleName(),
                                                                                                        confirmedRequestNumber));
            if (!unconfirmedRequests.isEmpty())
               LogTools.debug("%s Update #%d: Still unconfirmed requests: %s".formatted(crdtInfo.getActorDesignation(),
                                                                                        crdtInfo.getUpdateNumber(),
                                                                                        unconfirmedRequests));

            unfreeze();
         }
         else
         {
            LogTools.error("%s Update #%d: Unrecognized Request #%d. Unconfirmed requests: %s".formatted(crdtInfo.getActorDesignation(),
                                                                                                         crdtInfo.getUpdateNumber(),
                                                                                                         confirmedRequestNumber,
                                                                                                         unconfirmedRequests));
         }
      }
   }

   public CRDTInfo getCRDTInfo()
   {
      return crdtInfo;
   }
}
