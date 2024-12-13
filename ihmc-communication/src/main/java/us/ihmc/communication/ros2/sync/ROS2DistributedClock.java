package us.ihmc.communication.ros2.sync;

import ihmc_common_msgs.msg.dds.DistributedClockMessage;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.pubsub.common.Guid;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Topic;

import java.time.Instant;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * Sends out pings to other instances of this class on the network to estimate
 * the clock offsets on those computers. This can be used to synchronize data
 * modifications. We account for and assume a symmetric network delay.
 */
public class ROS2DistributedClock
{
   private static final ROS2Topic<DistributedClockMessage> TOPIC = ROS2Tools.IHMC_ROOT.withModule("distributed_clock").withType(DistributedClockMessage.class);

   private final HashMap<Guid, ROS2DistributedClockPeer> peerMap = new HashMap<>();
   private final List<ROS2DistributedClockPeer> peerList = new ArrayList<>();
   private int nextPeerToPing = 0;
   private final ROS2Publisher<DistributedClockMessage> publisher;
   private final Guid ourGuid;
   private final RepeatingTaskThread requestThread = new RepeatingTaskThread(getClass().getSimpleName(),
                                                                             this::requestThread,
                                                                             DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   private final DistributedClockMessage requestMessage = new DistributedClockMessage();
   private final DistributedClockMessage receivedMessage = new DistributedClockMessage();
   private final SampleInfo sampleInfo = new SampleInfo();
   private final Guid receivedRequestTarget = new Guid();
   private final Guid receivedReplyTarget = new Guid();

   public ROS2DistributedClock(ROS2Node ros2Node)
   {
      publisher = ros2Node.createPublisher(TOPIC);
      ourGuid = publisher.getPublisher().getGuid();

      ros2Node.createSubscription(TOPIC, subscriber ->
      {
         subscriber.takeNextData(receivedMessage, sampleInfo);

         MessageTools.fromMessage(receivedMessage.getRequestTarget(), receivedRequestTarget);
         MessageTools.fromMessage(receivedMessage.getReplyTarget(), receivedReplyTarget);

         if (receivedMessage.getIsRequest() && receivedRequestTarget.equals(ourGuid)) // Reply
         {
            DistributedClockMessage replyMessage = new DistributedClockMessage();
            replyMessage.set(receivedMessage);
            replyMessage.setIsRequest(false);
            MessageTools.toMessage(Instant.now(), replyMessage.getReplySendTime());

            ThreadTools.startAsDaemon(() ->
            {
               synchronized (publisher)
               {
                  publisher.publish(replyMessage);
               }
            }, getClass().getSimpleName() + "PublishReply");
         }
         else if (!receivedMessage.getIsRequest() && receivedReplyTarget.equals(ourGuid)) // Update clock offset estimate
         {
            ROS2DistributedClockPeer peer = peerMap.get(receivedRequestTarget);
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
            switch (info.getStatus())
            {
               case MATCHED_MATCHING ->
               {
                  if (!peerMap.containsKey(guid))
                  {
                     Guid guidCopy = new Guid();
                     guidCopy.set(guid);

                     ROS2DistributedClockPeer peer = new ROS2DistributedClockPeer(guidCopy);
                     peerMap.put(guidCopy, peer);
                     peerList.add(peer);
                  }
               }
               case REMOVED_MATCHING ->
               {
                  peerMap.remove(guid);
                  peerList.removeIf(peer -> peer.getGuid().equals(guid));
               }
            }
         }
      });

      requestThread.setFrequencyLimit(5.0);
      requestThread.startRepeating();
   }

   private void requestThread()
   {
      if (!peerList.isEmpty())
      {
         if (nextPeerToPing >= peerList.size())
            nextPeerToPing = 0;

         ROS2DistributedClockPeer peer = peerList.get(nextPeerToPing);
         requestMessage.setIsRequest(true);
         MessageTools.toMessage(peer.getGuid(), requestMessage.getRequestTarget());
         MessageTools.toMessage(ourGuid, requestMessage.getReplyTarget());
         MessageTools.toMessage(Instant.now(), requestMessage.getRequestSendTime());
         synchronized (publisher)
         {
            publisher.publish(requestMessage);
         }

         ++nextPeerToPing;
      }
   }

   public void destroy()
   {
      requestThread.kill();
   }

   public List<ROS2DistributedClockPeer> getPeerList()
   {
      return peerList;
   }
}
