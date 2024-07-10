package us.ihmc.communication.crdt;

import gnu.trove.list.array.TLongArrayList;
import gnu.trove.map.hash.TLongLongHashMap;
import ihmc_common_msgs.msg.dds.ConfirmableRequestMessage;
import org.apache.commons.lang3.mutable.MutableLong;
import us.ihmc.commons.thread.Notification;
import us.ihmc.log.LogTools;

/**
 * A freezable node that has a mechanism for the freeze
 * to end early if we know the change has propagated.
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
 * monotonically increases on each subsequent update. This is different
 * than the request number. In this class
 * that update number is used to "timeout" the freeze if a confirmation
 * is not recieved within the max freeze duration.
 * </p>
 *
 * <p>
 * The {@link #unfreeze} method works by rewinding the "update number to unfreeze",
 * which was previously set to a higher number which served as the timeout.
 * </p>
 *
 * <p>
 * This class handles dropped and out of order messages. It does so by
 * resending requests until they are confirmed and resending
 * confirmations until they expire.
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
   /** This reduces the request creation to once per update.
    *  Often, many changes occur in a single tick, like loading the initial state. */
   private final Notification needToSendRequest = new Notification();
   private final TLongArrayList unconfirmedRequests = new TLongArrayList();
   private final TLongLongHashMap requestTimeouts = new TLongLongHashMap();
   private final TLongArrayList recentConfirmations = new TLongArrayList();
   private final TLongLongHashMap confirmationTimeouts = new TLongLongHashMap();
   private transient final TLongArrayList timedOutEntries = new TLongArrayList();
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
      boolean wasFrozen = crdtInfo.getUpdateNumber() < updateNumberToUnfreeze;

      updateNumberToUnfreeze = crdtInfo.getUpdateNumber();

      boolean isFrozen = crdtInfo.getUpdateNumber() < updateNumberToUnfreeze;

      if (wasFrozen != isFrozen)
         LogTools.debug("%s Update #%d: Frozen %b -> %b %s".formatted(crdtInfo.getActorDesignation(),
                                                                      crdtInfo.getUpdateNumber(),
                                                                      wasFrozen,
                                                                      isFrozen,
                                                                      this.getClass().getSimpleName()));
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
      removeTimedOutEntries(unconfirmedRequests, requestTimeouts, true);
      removeTimedOutEntries(recentConfirmations, confirmationTimeouts, false);

      if (needToSendRequest.poll())
      {
         long requestNumber = nextRequestID.getAndIncrement();
         unconfirmedRequests.add(requestNumber);
         requestTimeouts.put(requestNumber, crdtInfo.getUpdateNumber() + crdtInfo.getMaxFreezeDuration());
      }

      message.getRequestNumbers().resetQuick();
      message.getConfirmationNumbers().resetQuick();

      for (int i = 0; i < unconfirmedRequests.size(); i++)
      {
         long requestNumber = unconfirmedRequests.get(i);
         LogTools.debug("%s Update #%d: %s requesting. Request #%d".formatted(crdtInfo.getActorDesignation(),
                                                                              crdtInfo.getUpdateNumber(),
                                                                              this.getClass().getSimpleName(),
                                                                              requestNumber));
         message.getRequestNumbers().add(requestNumber);
      }

      for (int i = 0; i < recentConfirmations.size(); i++)
      {
         long confirmationNumber = recentConfirmations.get(i);
         message.getConfirmationNumbers().add(confirmationNumber);
      }
   }

   public void fromMessage(ConfirmableRequestMessage message)
   {
      for (int i = 0; i < message.getRequestNumbers().size(); i++)
      {
         long confirmationNumber = message.getRequestNumbers().get(i);

         if (!recentConfirmations.contains(confirmationNumber))
         {
            LogTools.debug("%s Update #%d: %s confirming. Request #%d".formatted(crdtInfo.getActorDesignation(),
                                                                                 crdtInfo.getUpdateNumber(),
                                                                                 this.getClass().getSimpleName(),
                                                                                 confirmationNumber));
            recentConfirmations.add(confirmationNumber);
            confirmationTimeouts.put(confirmationNumber, crdtInfo.getUpdateNumber() + crdtInfo.getMaxFreezeDuration());
         }
      }

      for (int i = 0; i < message.getConfirmationNumbers().size(); i++)
      {
         long confirmedRequestNumber = message.getConfirmationNumbers().get(i);

         if (unconfirmedRequests.contains(confirmedRequestNumber))
         {
            unconfirmedRequests.remove(confirmedRequestNumber);
            requestTimeouts.remove(confirmedRequestNumber);

            LogTools.debug("%s Update #%d: %s recieved confirmation for Request #%d. Unfreezing.".formatted(crdtInfo.getActorDesignation(),
                                                                                                            crdtInfo.getUpdateNumber(),
                                                                                                            this.getClass().getSimpleName(),
                                                                                                            confirmedRequestNumber));
            if (unconfirmedRequests.isEmpty())
            {
               unfreeze();
            }
            else
            {
               LogTools.debug("%s Update #%d: Still unconfirmed requests: %s".formatted(crdtInfo.getActorDesignation(),
                                                                                        crdtInfo.getUpdateNumber(),
                                                                                        unconfirmedRequests));
            }
         }
      }
   }

   private void removeTimedOutEntries(TLongArrayList numbers, TLongLongHashMap timeouts, boolean areRequests)
   {
      timedOutEntries.clear();
      for (int i = 0; i < numbers.size(); i++)
      {
         if (crdtInfo.getUpdateNumber() >= timeouts.get(numbers.get(i)))
         {
            timedOutEntries.add(numbers.get(i));
         }
      }

      for (int i = 0; i < timedOutEntries.size(); i++)
      {
         if (areRequests)
         {
            LogTools.error("%s Update #%d: %s never received confirmation for Request #%d".formatted(crdtInfo.getActorDesignation(),
                                                                                                     crdtInfo.getUpdateNumber(),
                                                                                                     this.getClass().getSimpleName(),
                                                                                                     timedOutEntries.get(i)));
         }

         numbers.remove(timedOutEntries.get(i));
         timeouts.remove(timedOutEntries.get(i));
      }
   }

   public CRDTInfo getCRDTInfo()
   {
      return crdtInfo;
   }

   /** for tests only */ long getNextRequestID()
   {
      return nextRequestID.longValue();
   }
}
