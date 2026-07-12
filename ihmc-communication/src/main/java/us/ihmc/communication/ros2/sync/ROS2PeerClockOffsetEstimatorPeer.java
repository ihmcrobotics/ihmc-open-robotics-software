package us.ihmc.communication.ros2.sync;

import us.ihmc.commons.thread.Notification;
import us.ihmc.jros2.Guid;

import java.time.Duration;
import java.time.Instant;
import java.time.temporal.ChronoUnit;

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
   private Instant replyReceiveTime;
   private final Notification updatedNotification = new Notification();

   public ROS2PeerClockOffsetEstimatorPeer(Guid guid)
   {
      this.guid = guid;
   }

   public void update(Instant requestSendTime, Instant replyReceiveTime, Instant peerClockTime)
   {
      this.replyReceiveTime = replyReceiveTime;

      Duration roundTripTime = Duration.between(requestSendTime, replyReceiveTime);
      Duration halfRoundTripTime = roundTripTime.dividedBy(2);
      Instant peerNow = peerClockTime.plus(halfRoundTripTime);
      peerClockOffset = Duration.between(replyReceiveTime, peerNow);

      updatedNotification.set();
   }

   public Instant getPeerTimeInLocalFrame(Instant peerTime)
   {
      return peerTime.minus(peerClockOffset);
   }

   public Instant getPeerTimeInPeerFrame(Instant ourTime)
   {
      return ourTime.plus(peerClockOffset);
   }

   public Notification getUpdatedNotification()
   {
      return updatedNotification;
   }

   public boolean isAlive()
   {
      return isAlive(Instant.now());
   }

   public boolean isAlive(Instant ourTime)
   {
      return replyReceiveTime != null && replyReceiveTime.isAfter(ourTime.minus(1, ChronoUnit.SECONDS));
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
