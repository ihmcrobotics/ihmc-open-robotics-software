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
   private long updateNumber = 0;

   /** For timestamp based CRDTs that use {@link LatestTimestampModifiable}. */
   public CRDTInfo(ROS2ActorDesignation actorDesignation, ROS2PeerClockOffsetEstimator peerClockEstimator)
   {
      this.actorDesignation = actorDesignation;
      this.peerClockEstimator = peerClockEstimator;
   }

   public void startNextUpdate()
   {
      ++updateNumber;
   }

   public long getUpdateNumber()
   {
      return updateNumber;
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
