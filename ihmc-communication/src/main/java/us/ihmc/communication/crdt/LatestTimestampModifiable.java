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
   private static final Guid ZERO_GUID = new Guid();
   private final CRDTInfo crdtInfo;
   private final Guid latestModifier = new Guid();
   private Instant latestModificationTime = Instant.MIN;
   private boolean modificationIncoming = false;
   private boolean modificationOutgoing = false;
   private String debugName = "";
   /**
    * Used by local code to track external modifications to state data.
    * Data can be modified in several ways, such as loading from file,
    * remote processes, etc.
    */
   private Instant modificationCheckTime = Instant.MIN;
   /**
    * Whether the associated data had been modified between the last two
    * calls to {@link #checkModified} or was modified for the first time
    * before the most recent call.
    */
   private boolean isModified = false;

   private transient final Guid messageModifier = new Guid();

   public LatestTimestampModifiable(CRDTInfo crdtInfo)
   {
      this.crdtInfo = crdtInfo;
   }

   public void setDebugName(String debugName)
   {
      this.debugName = debugName;
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
      if (crdtInfo.isMultiMachineNetwork())
         latestModifier.set(crdtInfo.getPeerClockEstimator().getOurGuid());
      latestModificationTime = Instant.now();

      if (!modificationOutgoing)
         LogTools.debug("{}: OUTGOING = true", debugName);

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
      isModified = latestModificationTime.isAfter(modificationCheckTime);
      modificationCheckTime = latestModificationTime;
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
         LogTools.debug("{}: OUTGOING = false", debugName);
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
      MessageTools.toMessage(latestModifier, message.getLatestModifierId());
      MessageTools.toMessage(latestModificationTime, message.getLatestModificationTimeInModifierFrame());
   }

   public void fromMessage(LatestModificationMessage message)
   {
      if (modificationIncoming)
         LogTools.debug("{}: INCOMING = false", debugName);
      modificationIncoming = false;

      if (crdtInfo.isMultiMachineNetwork())
      {
         MessageTools.fromMessage(message.getLatestModifierId(), messageModifier);

         // Another peer made the most recent modification
         if (!messageModifier.equals(ZERO_GUID) && !messageModifier.equals(crdtInfo.getPeerClockEstimator().getOurGuid()))
         {
            ROS2PeerClockOffsetEstimatorPeer latestModifierPeer = crdtInfo.getPeerClockEstimator().getPeerMap().get(messageModifier);
            if (latestModifierPeer == null)
            {
               LogTools.error("Peer not in peer map: {}", messageModifier);
            }
            else
            {
               Instant timeInPeerFrame = MessageTools.toInstant(message.getLatestModificationTimeInModifierFrame());
               Instant timeInLocalFrame = latestModifierPeer.getPeerTimeInLocalFrame(timeInPeerFrame);

               if (timeInLocalFrame.isAfter(latestModificationTime))
               {
                  latestModifier.set(messageModifier);
                  latestModificationTime = timeInLocalFrame;
                  modificationIncoming = true;
                  LogTools.debug("{}: INCOMING = true", debugName);
               }
            }
         }
      }
      else // Single machine network
      {
         Instant timeInLocalFrame = MessageTools.toInstant(message.getLatestModificationTimeInModifierFrame());
         if (timeInLocalFrame.isAfter(latestModificationTime))
         {
            latestModificationTime = timeInLocalFrame;
            modificationIncoming = true;
            LogTools.debug("{}: INCOMING = true", debugName);
         }
      }
   }

   public CRDTInfo getCRDTInfo()
   {
      return crdtInfo;
   }
}
