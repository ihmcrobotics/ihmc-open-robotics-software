package us.ihmc.communication.ros2.sync;

import ihmc_common_msgs.msg.dds.DistributedClockMessage;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.common.Guid;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Topic;

import java.time.Instant;
import java.util.ArrayList;
import java.util.HashMap;

/**
 * Assumes symmetric network delay.
 */
public class ROS2DistributedClock
{
   private static final ROS2Topic<DistributedClockMessage> TOPIC = ROS2Tools.IHMC_ROOT.withModule("distributed_clock").withType(DistributedClockMessage.class);

   private final HashMap<Guid, ROS2DistributedClockPeer> peerMap = new HashMap<>();
   private final ArrayList<ROS2DistributedClockPeer> peerList = new ArrayList<>();
   private int nextPeerToPing = 0;
   private final ROS2Publisher<DistributedClockMessage> publisher;
   private final Guid ourGuid;
   private final RepeatingTaskThread requestThread = new RepeatingTaskThread(getClass().getSimpleName(),
                                                                             this::requestThread,
                                                                             DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   private final DistributedClockMessage requestMessage = new DistributedClockMessage();
   private final DistributedClockMessage receivedMessage = new DistributedClockMessage();
   private final DistributedClockMessage replyMessage = new DistributedClockMessage();
   private final Guid receivedRequestTarget = new Guid();
   private final Guid receivedReplyTarget = new Guid();

   public ROS2DistributedClock(ROS2Node ros2Node)
   {
      publisher = ros2Node.createPublisher(TOPIC);
      ourGuid = publisher.getPublisher().getGuid();

      ros2Node.createSubscription(TOPIC, subscriber ->
      {
         subscriber.takeNextData(receivedMessage, null);

         MessageTools.fromMessage(receivedMessage.getRequestTarget(), receivedRequestTarget);
         MessageTools.fromMessage(receivedMessage.getReplyTarget(), receivedReplyTarget);

         if (receivedRequestTarget.equals(ourGuid)) // Reply
         {
            replyMessage.set(receivedMessage);
            MessageTools.toMessage(Instant.now(), replyMessage.getReplySendTime());
            synchronized (publisher)
            {
               publisher.publish(replyMessage);
            }
         }
         else if (receivedReplyTarget.equals(ourGuid)) // Update clock offset estimate
         {
            ROS2DistributedClockPeer peer = peerMap.get(receivedReplyTarget);
            if (peer != null)
            {
               peer.update(MessageTools.toInstant(receivedMessage.getRequestSendTime()),
                           Instant.now(),
                           MessageTools.toInstant(receivedMessage.getReplySendTime()));
            }
         }
      }, (subscriber, info) ->
      {
         Guid guid = info.getGuid();
         if (!guid.equals(publisher.getPublisher().getGuid())) // Exclude our publisher
         {
            LogTools.info("ent: %s prefix: %s status: %s".formatted(guid.getEntity(), guid.getGuidPrefix(), info.getStatus()));
            switch (info.getStatus())
            {
               case MATCHED_MATCHING ->
               {
                  if (!peerMap.containsKey(guid))
                  {
                     Guid guidCopy = new Guid();
                     guidCopy.set(guid);

                     LogTools.info("Setting up peer: %s", guidCopy);
                     peerMap.put(guidCopy, new ROS2DistributedClockPeer(guidCopy));
                  }
               }
               case REMOVED_MATCHING ->
               {
                  peerMap.remove(guid);
               }
            }
         }
      });

      requestThread.setFrequencyLimit(5.0);
      requestThread.start();
   }

   private void requestThread()
   {
      if (!peerList.isEmpty())
      {
         if (nextPeerToPing >= peerList.size())
            nextPeerToPing = 0;

         ROS2DistributedClockPeer peer = peerList.get(nextPeerToPing);
         MessageTools.toMessage(Instant.now(), requestMessage.getRequestSendTime());
         MessageTools.toMessage(peer.getGuid(), requestMessage.getRequestTarget());
         MessageTools.toMessage(publisher.getPublisher().getGuid(), requestMessage.getReplyTarget());
         synchronized (publisher)
         {
            publisher.publish(requestMessage);
         }

         ++nextPeerToPing;
      }
   }
}
