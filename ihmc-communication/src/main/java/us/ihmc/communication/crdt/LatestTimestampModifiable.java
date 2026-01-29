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
 *
 * The algorithm uses a monotonically increasing modification number as
 * the primary way to order modifications. A timestamp with nanosecond precision
 * is used to resolve any race conditions.
 *
 * This class also provides infrastructure for sending data over the network
 * only when it is needed to reduce bandwidth.
 */
public class LatestTimestampModifiable
{
   private final CRDTInfo crdtInfo;
   private String ourName = "unnamed";

   private String latestModifierName = ourName;
   private final Guid latestModifierGuid = new Guid();
   private Instant latestModificationTime = Instant.MIN;
   private long modificationNumber = 0;
   private boolean modificationIncoming = false;
   private boolean needSendFullData = false;
   private boolean requestSendFullData = false;

   /**
    * Used by local code to track external modifications to state data.
    * Data can be modified in several ways, such as loading from file,
    * remote processes, etc.
    */
   private long modificationCheckNumber = 0;
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
      this.ourName = crdtInfo.getActorDesignation().name() + ": " + ourName;
      latestModifierName = this.ourName;
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

      if (!needSendFullData)
      {
         ++modificationNumber;
         LogTools.debug("{}: Update # {} Modification # {} Need send full data: false -> true", ourName, crdtInfo.getUpdateNumber(), modificationNumber);
      }

      needSendFullData = true;
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

   /** Used when we missed a message or recently came online and need the full data. */
   public void requestSendFullData()
   {
      if (!requestSendFullData)
         LogTools.debug("{}: Update # {} Request send full data", ourName, crdtInfo.getUpdateNumber());
      requestSendFullData = true;
   }

   /** Call when we receive the full data to stop requesting it. */
   public void confirmReceivedFullData()
   {
      if (requestSendFullData)
         LogTools.debug("{}: Update # {} Full data received", ourName, crdtInfo.getUpdateNumber());

      requestSendFullData = false;
   }

   /**
    * Used by the state publisher to determine if the full
    * message data needs to be sent out.
    */
   public boolean pollNeedSendFullData()
   {
      boolean priorValue = needSendFullData;
      if (priorValue)
         LogTools.debug("{}: Update # {} Need send full data: true -> false", ourName, crdtInfo.getUpdateNumber());
      needSendFullData = false;
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
      message.setFullDataNeeded(requestSendFullData);
      needSendFullData = false; // In case it doesn't get polled
   }

   public void fromMessage(LatestModificationMessage message)
   {
      fromMessage(message, false);
   }

   /**
    * @param checkOnly is used for when partial data is received in order to not clear the modification
    *                  status, so isModificationIncoming == true when receiving the full data, which may
    *                  be several updates later, even using RELIABLE mode, apparently
    */
   public void fromMessage(LatestModificationMessage message, boolean checkOnly)
   {
      if (modificationIncoming)
         LogTools.debug("{}: Update # {} INCOMING true -> false", ourName, crdtInfo.getUpdateNumber());
      modificationIncoming = false;

      if (message.getFullDataNeeded() && !needSendFullData)
      {
         LogTools.debug("{}: Update # {} INCOMING Need send full data: false -> true", ourName, crdtInfo.getUpdateNumber());
         needSendFullData = true;
      }

      long incomingModificationNumber = message.getLatestModificationNumber();
      String incomingModifierName = message.getLatestModifierNameAsString();
      // Only need to do anything if another peer made the most recent modification
      // and at least one modification has been made
      MessageTools.fromMessage(message.getLatestModifierId(), incomingModifierGuid);
      Guid ourGuid = crdtInfo.getPeerClockEstimator().getOurGuid();
      if (incomingModificationNumber > 0 && !incomingModifierGuid.equals(ourGuid))
      {
         ROS2PeerClockOffsetEstimatorPeer latestModifierPeer = crdtInfo.getPeerClockEstimator().getPeer(incomingModifierGuid);
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
            LogTools.debug(() -> "%s: Update # %d INCOMING = true  Modification # %d -> %d  Modifier: %s"
                  .formatted(ourName, crdtInfo.getUpdateNumber(), modificationNumber, incomingModificationNumber, incomingModifierName));
            if (!checkOnly)
            {
               modificationNumber = incomingModificationNumber;

               if (peerTimeAvailable)
               {
                  latestModifierGuid.set(incomingModifierGuid);
                  latestModificationTime = incomingModificationTime;
                  latestModifierName = incomingModifierName;
               }
               else // If peer that most recently modified is now offline, we need to become the latest modifier
               {    // because we don't know when that time was anymore
                  LogTools.debug(() -> "%s: Update # %d INCOMING = true  Modification # %d -> %d  Peer offline: %s"
                        .formatted(ourName,
                                   crdtInfo.getUpdateNumber(),
                                   modificationNumber,
                                   incomingModificationNumber,
                                   incomingModifierName));
                  modify();
               }
            }
            modificationIncoming = true;
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
               LogTools.debug(() -> "%s: Update # %d INCOMING = true  Race! Modification # %d == %d  Local: %s  %s: %s   Peer offset: %s"
                     .formatted(ourName,
                                crdtInfo.getUpdateNumber(),
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
               LogTools.debug(() -> "%s: Update # %d INCOMING = true  Race: Modification # %d == %d  Local: %s  Peer offline: %s"
                     .formatted(ourName,
                                crdtInfo.getUpdateNumber(),
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
               if (!checkOnly)
               {
                  latestModifierGuid.set(incomingModifierGuid);
                  latestModificationTime = incomingModificationTime;
                  modificationNumber = incomingModificationNumber;
                  latestModifierName = incomingModifierName;
               }
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
