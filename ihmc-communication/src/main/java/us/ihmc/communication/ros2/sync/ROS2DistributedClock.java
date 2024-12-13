package us.ihmc.communication.ros2.sync;

import ihmc_common_msgs.msg.dds.DistributedClockMessage;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.common.Guid;
import us.ihmc.pubsub.common.Guid.Entity;
import us.ihmc.pubsub.common.MatchingInfo;
import us.ihmc.pubsub.common.MatchingInfo.MatchingStatus;
import us.ihmc.pubsub.impl.fastRTPS.FastRTPSPublisher;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.SubscriptionMatchedListener;

import java.net.InetAddress;
import java.time.Instant;
import java.util.HashMap;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Assumes symmetric network delay.
 *
 * TODO: Handle intraprocess mode? Are there only FastRTPS pub/subs now?
 */
public class ROS2DistributedClock
{
   private static final AtomicLong INDEX = new AtomicLong();
   private static final ROS2Topic<DistributedClockMessage> TOPIC = ROS2Tools.IHMC_ROOT.withModule("distributed_clock").withType(DistributedClockMessage.class);

   private final String ourName;
   private final HashMap<String, ROS2DistributedClockPeer> peers = new HashMap<>();

   private final DistributedClockMessage receivedClockMessage = new DistributedClockMessage();
//   private final Guid ourPublisherGuid;

   private long requestNumber = 0;
   private final DistributedClockMessage outgoingClockMessage = new DistributedClockMessage();
   private final ROS2Publisher<DistributedClockMessage> publisher;
   private final RepeatingTaskThread requestThread = new RepeatingTaskThread(getClass().getSimpleName(),
                                                                             this::requestThread,
                                                                             DefaultExceptionHandler.MESSAGE_AND_STACKTRACE)
                                                         .setFrequencyLimit(5.0);

   public ROS2DistributedClock(ROS2Node ros2Node)
   {
      AtomicReference<String> hostname = new AtomicReference<>("");
      ExceptionTools.handle(() -> hostname.set("_" + InetAddress.getLocalHost().getHostName()), DefaultExceptionHandler.PROCEED_SILENTLY);

      ourName = "%s%s_%d_%d".formatted(ros2Node.getName(), hostname.get(), ProcessHandle.current().pid(), INDEX.getAndIncrement());


      publisher = ros2Node.createPublisher(TOPIC);

//      if (publisher instanceof FastRTPSPublisher<DistributedClockMessage> fastRTPSPublisher)
//      {
//         ourPublisherGuid = fastRTPSPublisher.getGuid();
//      }

      ros2Node.createSubscription(TOPIC, subscriber ->
      {
         subscriber.takeNextData(receivedClockMessage, null);

         if (receivedClockMessage.getRequesterIdAsString().equals(ourName))
         {
            Instant replyReceptionTime = Instant.now();

            String peerName = receivedClockMessage.getReplierIdAsString();
            ROS2DistributedClockPeer peer = peers.get(peerName);

            if (peer == null)
            {
               peer = new ROS2DistributedClockPeer(peerName);
               peers.put(peerName, peer);
            }

//            peer.
         }
         else
         {

         }
      }, (subscriber, info) ->
      {

         Guid guid = info.getGuid();

         Entity entity = guid.getEntity();
         String entityName = entity.toString();

         String prefix = guid.getGuidPrefix().toString();

         MatchingStatus status = info.getStatus();

         LogTools.info("ent: %s prefix: %s status: %s".formatted(entityName, prefix, status));


      });

      requestThread.start();
   }

   private void requestThread()
   {

//      outgoingClockMessage.set

      ++requestNumber;
   }
}
