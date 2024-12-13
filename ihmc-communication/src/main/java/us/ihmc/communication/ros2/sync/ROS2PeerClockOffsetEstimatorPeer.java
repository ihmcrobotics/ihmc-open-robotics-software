package us.ihmc.communication.ros2.sync;

import us.ihmc.pubsub.common.Guid;

import java.time.Duration;
import java.time.Instant;

/**
 * Keeps track of the clock offset of a peer.
 *
 * TODO: Add an alpha filter since the network latency and compute
 *   is noisy, but the clock rates are smooth.
 */
public class ROS2PeerClockOffsetEstimatorPeer
{
   private final Guid guid;
   private Duration peerClockOffset;

   public ROS2PeerClockOffsetEstimatorPeer(Guid guid)
   {
      this.guid = guid;
   }

   public void update(Instant requestSendTime, Instant replyReceiveTime, Instant peerClockTime)
   {
      Duration roundTripTime = Duration.between(requestSendTime, replyReceiveTime);
      Duration halfRoundTripTime = roundTripTime.dividedBy(2);
      Instant peerNow = peerClockTime.plus(halfRoundTripTime);
      peerClockOffset = Duration.between(replyReceiveTime, peerNow);
   }

   public Instant convertPeerTimeToOurTime(Instant peerTime)
   {
      return peerTime.minus(peerClockOffset);
   }

   public Duration getPeerClockOffset()
   {
      return peerClockOffset;
   }

   public Guid getGuid()
   {
      return guid;
   }
}
