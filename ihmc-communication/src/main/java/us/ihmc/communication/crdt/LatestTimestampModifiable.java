package us.ihmc.communication.crdt;

import ihmc_common_msgs.msg.dds.LatestModificationMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimatorPeer;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.common.Guid;

import java.time.Instant;

public class LatestTimestampModifiable
{
   private final CRDTInfo crdtInfo;
   private final Guid latestModifier = new Guid();
   private Instant latestModificationTime = Instant.MIN;
   private boolean isOutOfDate = true;

   private transient final Guid messageModifier = new Guid();

   public LatestTimestampModifiable(CRDTInfo crdtInfo)
   {
      this.crdtInfo = crdtInfo;
   }

   public void modify()
   {
      if (crdtInfo.isMultiMachineNetwork())
         latestModifier.set(crdtInfo.getPeerClockEstimator().getOurGuid());
      latestModificationTime = Instant.now();
   }

   /**
    * @return If the incoming modification was older or same time as what we already have.
    *         To be used after {@link #fromMessage}.
    */
   public boolean isUpToDate()
   {
      return !isOutOfDate;
   }

   /**
    * @return If the incoming modification is newer than what we have.
    *         If so, we'll be expecting to have our state updated to the incoming one.
    *         To be used after {@link #fromMessage}.
    */
   public boolean isOutOfDate()
   {
      return isOutOfDate;
   }

   public void toMessage(LatestModificationMessage message)
   {
      MessageTools.toMessage(latestModifier, message.getLatestModifierId());
      MessageTools.toMessage(latestModificationTime, message.getLatestModificationTimeInModifierFrame());
   }

   public void fromMessage(LatestModificationMessage message)
   {
      isOutOfDate = false;

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
                  isOutOfDate = true;
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
            isOutOfDate = true;
         }
      }
   }

   public CRDTInfo getCRDTInfo()
   {
      return crdtInfo;
   }
}
