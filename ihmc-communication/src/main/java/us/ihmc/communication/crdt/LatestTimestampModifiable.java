package us.ihmc.communication.crdt;

import ihmc_common_msgs.msg.dds.LatestModificationMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimatorPeer;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.common.Guid;

import java.time.Instant;

/**
 * Keeps track of the latest time associated data was modified, for
 * the purpose of network synchronization. The association to data is
 * external to this class.
 */
public class LatestTimestampModifiable
{
   private final CRDTInfo crdtInfo;
   private String ourName = "unamed";

   private String latestModifierName = ourName;
   private final Guid latestModifierGuid = new Guid();
   private Instant latestModificationTime = Instant.MIN;
   private long modificationNumber = -1;
   private boolean modificationIncoming = false;
   private boolean modificationOutgoing = false;

   /**
    * Used by local code to track external modifications to state data.
    * Data can be modified in several ways, such as loading from file,
    * remote processes, etc.
    */
   private long modificationCheckNumber = -1;
   /**
    * Whether the associated data had been modified between the last two
    * calls to {@link #checkModified} or was modified for the first time
    * before the most recent call.
    */
   private boolean isModified = false;

   private transient final Guid incomingModifierGuid = new Guid();

   public LatestTimestampModifiable(CRDTInfo crdtInfo)
   {
      this.crdtInfo = crdtInfo;
   }

   /** To be set only once right after construction. */
   public void setModifierName(String ourName)
   {
      this.ourName = ourName;
      latestModifierName = ourName;
   }

   /**
    * Call to mark this data as holding the latest valid values.
    * These values will propagate to the rest of the system until
    * this method is called again on this data instance here
    * or somewhere else. This method records a timestamp to compare
    * against latest remote modifications.
    */
   public void modify()
   {
      latestModifierGuid.set(crdtInfo.getPeerClockEstimator().getOurGuid());
      latestModificationTime = Instant.now();
      latestModifierName = ourName;
      ++modificationNumber;

      if (!modificationOutgoing)
         LogTools.debug("{}: OUTGOING = true  # {}", ourName, modificationNumber);

      modificationOutgoing = true;
   }

   /**
    * Call once per tick. The {@link #isModified()} will return
    * whether the associated data has been modified since the last time we checked.
    * {@link #isModified()} will return a constant value until this
    * method is called again.
    */
   public void checkModified()
   {
      isModified = modificationNumber > modificationCheckNumber;
      modificationCheckNumber = modificationNumber;
   }

   public boolean isModified()
   {
      return isModified;
   }

   /**
    * Used by the state publisher to determine if the full
    * message data needs to be sent out.
    */
   public boolean pollModificationOutgoing()
   {
      boolean priorValue = modificationOutgoing;
      if (priorValue)
         LogTools.debug("{}: OUTGOING = false", ourName);
      modificationOutgoing = false;
      return priorValue;
   }

   /**
    * @return If the incoming modification is newer than what we have.
    *         If so, we'll be expecting to have our state updated to the incoming one.
    *         To be used after {@link #fromMessage} in subsequent {@link #fromMessage}s.
    */
   public boolean isModificationIncoming()
   {
      return modificationIncoming;
   }

   public void toMessage(LatestModificationMessage message)
   {
      MessageTools.toMessage(latestModifierGuid, message.getLatestModifierId());
      MessageTools.toMessage(latestModificationTime, message.getLatestModificationTimeInModifierFrame());
      message.setLatestModificationNumber(modificationNumber);
      message.setLatestModifierName(latestModifierName);
   }

   public void fromMessage(LatestModificationMessage message)
   {
      if (modificationIncoming)
         LogTools.debug("{}: INCOMING = false", ourName);
      modificationIncoming = false;

      long incomingModificationNumber = message.getLatestModificationNumber();
      String incomingModifierName = message.getLatestModifierNameAsString();
      // Only need to do anything if another peer made the most recent modification
      // and at least one modification has been made
      MessageTools.fromMessage(message.getLatestModifierId(), incomingModifierGuid);
      Guid ourGuid = crdtInfo.getPeerClockEstimator().getOurGuid();
      if (incomingModificationNumber >= 0 && !incomingModifierGuid.equals(ourGuid))
      {
         ROS2PeerClockOffsetEstimatorPeer latestModifierPeer = crdtInfo.getPeerClockEstimator().getPeerMap().get(incomingModifierGuid);
         boolean peerTimeAvailable = latestModifierPeer != null;
         Instant incomingModificationTime;
         if (peerTimeAvailable)
         {
            Instant incomingModificationTimePeerFrame = MessageTools.toInstant(message.getLatestModificationTimeInModifierFrame());
            incomingModificationTime = latestModifierPeer.getPeerTimeInLocalFrame(incomingModificationTimePeerFrame);
         }
         else
         {
            incomingModificationTime = null;
         }

         // If a later modification number is availble then we take it without checking time
         if (incomingModificationNumber > modificationNumber)
         {
            LogTools.debug(() -> "%s: INCOMING = true  Modification # %d -> %d  Modifier: %s"
                  .formatted(ourName, modificationNumber, incomingModificationNumber, incomingModifierName));
            latestModifierGuid.set(incomingModifierGuid);
            modificationNumber = incomingModificationNumber;
            latestModifierName = incomingModifierName;
            modificationIncoming = true;

            // If peer that most recently modified is now offline, we need to become the latest modifier
            // because
            if (peerTimeAvailable)
            {
               latestModificationTime = incomingModificationTime;
            }
            else
            {
               LogTools.warn(() -> "%s: INCOMING = true  Modification # %d -> %d  Peer offline: %s"
                     .formatted(ourName,
                                modificationNumber,
                                incomingModificationNumber,
                                incomingModifierName));
            }
         }
         // If modification number is the same as what we have, but the modification was made by another peer,
         // then it's a race condition, and we need to resolve it
         else if (incomingModificationNumber == modificationNumber && !incomingModifierGuid.equals(latestModifierGuid))
         {
            // We need to increment modification number and mark modified to resolve the race
            // keeping the time
            boolean localWins;

            // We'll resolve the race by a timestamp in hopes to settle it
            if (peerTimeAvailable)
            {
               LogTools.warn(() -> "%s: INCOMING = true  Race! Modification # %d == %d  Local: %s  %s: %s   Peer offset: %s"
                     .formatted(ourName,
                                modificationNumber,
                                incomingModificationNumber,
                                latestModificationTime,
                                incomingModifierName,
                                incomingModificationTime,
                                latestModifierPeer.getPeerClockOffset()));

               localWins = latestModificationTime.isAfter(incomingModificationTime);
            }
            else // Rare case, a now offline peer somehow concurrently modified
            {
               LogTools.error(() -> "%s: INCOMING = true  Race: Modification # %d == %d  Local: %s  Peer offline: %s"
                     .formatted(ourName,
                                modificationNumber,
                                incomingModificationNumber,
                                latestModificationTime,
                                incomingModifierName));
               // We need to win because we don't know the time of modification of the offline peer
               localWins = true;
            }

            if (localWins)
            {
               // We need to modify again to increment the number and send this out
               // to the other peers, resolving the conflict
               modify();
            }
            else
            {
               latestModifierGuid.set(incomingModifierGuid);
               latestModificationTime = incomingModificationTime;
               modificationNumber = incomingModificationNumber;
               latestModifierName = incomingModifierName;
               modificationIncoming = true;
            }
         }
      }
   }

   public CRDTInfo getCRDTInfo()
   {
      return crdtInfo;
   }
}
