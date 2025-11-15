package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;

/**
 * Holds onto this actor's CRDT information, used by the algorithm to sync data.
 * There should only one of these per synchronizing module.
 */
public class CRDTInfo
{
   private final ROS2ActorDesignation actorDesignation;
   private final ROS2PeerClockOffsetEstimator peerClockEstimator;
   private final int maxFreezeDuration;
   private long updateNumber = 0;

   /** For timestamp based CRDTs that use {@link LatestTimestampModifiable}. */
   public CRDTInfo(ROS2ActorDesignation actorDesignation, ROS2PeerClockOffsetEstimator peerClockEstimator)
   {
      this.actorDesignation = actorDesignation;
      this.peerClockEstimator = peerClockEstimator;
      this.maxFreezeDuration = -1;
   }

   /** For freezable CRDTs that use {@link RequestConfirmFreezable}. */
   public CRDTInfo(ROS2ActorDesignation actorDesignation, int maxFreezeDuration)
   {
      this.actorDesignation = actorDesignation;
      this.maxFreezeDuration = maxFreezeDuration;
      this.peerClockEstimator = null;
   }

   public void startNextUpdate()
   {
      ++updateNumber;
   }

   public long getUpdateNumber()
   {
      return updateNumber;
   }

   /** Only used for freezable CRDTs. */
   public int getMaxFreezeDuration()
   {
      return maxFreezeDuration;
   }

   public ROS2ActorDesignation getActorDesignation()
   {
      return actorDesignation;
   }

   /** Only used for timestamp based CRDTs. */
   public ROS2PeerClockOffsetEstimator getPeerClockEstimator()
   {
      return peerClockEstimator;
   }
}
