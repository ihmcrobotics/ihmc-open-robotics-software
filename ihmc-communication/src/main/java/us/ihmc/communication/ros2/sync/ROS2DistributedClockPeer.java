package us.ihmc.communication.ros2.sync;

import us.ihmc.pubsub.common.Guid;

import java.time.Duration;
import java.time.Instant;

public class ROS2DistributedClockPeer
{
   private final Guid guid;
   private Duration peerClockOffset;

   public ROS2DistributedClockPeer(Guid guid)
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

   public Guid getGuid()
   {
      return guid;
   }
}
