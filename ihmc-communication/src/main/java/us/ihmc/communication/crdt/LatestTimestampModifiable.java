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
   private final Guid latestModifier = new Guid();
   private Instant latestModificationTime = Instant.MIN;
   private boolean modificationIncoming = false;
   private boolean modificationOutgoing = false;

   private transient final Guid messageModifier = new Guid();

   public LatestTimestampModifiable(CRDTInfo crdtInfo)
   {
      this.crdtInfo = crdtInfo;
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
      LogTools.debug(1, "MODIFY");

      if (crdtInfo.isMultiMachineNetwork())
         latestModifier.set(crdtInfo.getPeerClockEstimator().getOurGuid());
      latestModificationTime = Instant.now();
      modificationOutgoing = true;
   }

   /**
    * @return If we changed this data locally and it needs to be sent out.
    *         To be used after call to {@link #modify}.
    */
   public boolean isModificationOutgoing()
   {
      return modificationOutgoing;
   }

   /**
    * @return If the incoming modification is newer than what we have.
    *         If so, we'll be expecting to have our state updated to the incoming one.
    *         To be used after {@link #fromMessage}.
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
      modificationOutgoing = false;
      modificationIncoming = false;

      if (crdtInfo.isMultiMachineNetwork())
      {
         MessageTools.fromMessage(message.getLatestModifierId(), messageModifier);

         // Another peer made the most recent modification
         if (!messageModifier.equals(crdtInfo.getPeerClockEstimator().getOurGuid()))
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
         }
      }
   }

   public CRDTInfo getCRDTInfo()
   {
      return crdtInfo;
   }
}
