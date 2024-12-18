package us.ihmc.communication.crdt;

import ihmc_common_msgs.msg.dds.CRDTTimestampedModificationMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimatorPeer;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.common.Guid;

import java.time.Instant;

public class CRDTModificationManager
{
   private final CRDTInfo globalInfo;
   private final Guid latestModifier = new Guid();
   private Instant latestModificationTime = Instant.MIN;
   private boolean isOutOfDate = true;

   private transient final Guid messageModifier = new Guid();

   public CRDTModificationManager(CRDTInfo globalInfo)
   {
      this.globalInfo = globalInfo;
   }

   public void modify()
   {
      if (globalInfo.isMultiMachineNetwork())
         latestModifier.set(globalInfo.getPeerClockEstimator().getOurGuid());
      latestModificationTime = Instant.now();
   }

   public boolean isOutOfDate()
   {
      return isOutOfDate;
   }

   public void toMessage(CRDTTimestampedModificationMessage message)
   {
      MessageTools.toMessage(latestModifier, message.getLatestModifierId());
      MessageTools.toMessage(latestModificationTime, message.getLatestModificationTimeInModifierFrame());
   }

   public void fromMessage(CRDTTimestampedModificationMessage message)
   {
      isOutOfDate = false;

      if (globalInfo.isMultiMachineNetwork())
      {
         MessageTools.fromMessage(message.getLatestModifierId(), messageModifier);

         // Another peer made the most recent modification
         if (!messageModifier.equals(globalInfo.getPeerClockEstimator().getOurGuid()))
         {
            ROS2PeerClockOffsetEstimatorPeer latestModifierPeer = globalInfo.getPeerClockEstimator().getPeerMap().get(messageModifier);
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
}
