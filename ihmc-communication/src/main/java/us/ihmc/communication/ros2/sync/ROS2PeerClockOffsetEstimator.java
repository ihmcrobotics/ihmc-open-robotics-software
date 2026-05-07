package us.ihmc.communication.ros2.sync;

import ihmc_common_msgs.PeerClockOffsetEstimatorPingMessage;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.pubsub.common.Guid;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.jros2.ROS2Topic;

import java.time.Instant;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Sends out pings to other instances of this class on the network to estimate
 * the clock offsets on those computers. This can be used to synchronize data
 * modifications. We account for and assume a symmetric network delay.
 */
public class ROS2PeerClockOffsetEstimator
{
   private static final ROS2Topic<PeerClockOffsetEstimatorPingMessage> TOPIC = ROS2Tools.IHMC_ROOT.appendedWith("peer_clock_offset_estimator")
                                                                                                  .withType(PeerClockOffsetEstimatorPingMessage.class);

   private final HashMap<Guid, ROS2PeerClockOffsetEstimatorPeer> peerMap = new HashMap<>();
   private final List<ROS2PeerClockOffsetEstimatorPeer> peerList = new ArrayList<>();
   private int nextPeerToPing = 0;
   private final ROS2Publisher<PeerClockOffsetEstimatorPingMessage> publisher;
   private final Guid ourGuid;
   private final RepeatingTaskThread requestThread = new RepeatingTaskThread(getClass().getSimpleName(),
                                                                             this::runRequestTask,
                                                                             DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   private final ROS2Subscription<PeerClockOffsetEstimatorPingMessage> subscription;
   private final ExecutorService cachedThreadPool
         = Executors.newCachedThreadPool(ThreadTools.createNamedThreadFactory(getClass().getSimpleName() + "PublishReply", true));
   private final PeerClockOffsetEstimatorPingMessage requestMessage = new PeerClockOffsetEstimatorPingMessage();
   private final PeerClockOffsetEstimatorPingMessage receivedMessage = new PeerClockOffsetEstimatorPingMessage();
   private final SampleInfo sampleInfo = new SampleInfo();
   private final Guid receivedRequestTarget = new Guid();
   private final Guid receivedReplyTarget = new Guid();

   public ROS2PeerClockOffsetEstimator(ROS2Node ros2Node)
   {
      publisher = ros2Node.createPublisher(TOPIC);
      ourGuid = publisher.getPublisher().getGuid();

      subscription = ros2Node.createSubscription(TOPIC, subscriber ->
      {
         subscriber.takeNextData(receivedMessage, sampleInfo);

         MessageTools.fromMessage(receivedMessage.getRequestTarget(), receivedRequestTarget);
         MessageTools.fromMessage(receivedMessage.getReplyTarget(), receivedReplyTarget);

         if (receivedMessage.getIsRequest() && receivedRequestTarget.equals(ourGuid)) // Reply
         {
            PeerClockOffsetEstimatorPingMessage replyMessage = new PeerClockOffsetEstimatorPingMessage();
            replyMessage.set(receivedMessage);
            replyMessage.setIsRequest(false);
            MessageTools.toMessage(Instant.now(), replyMessage.getReplySendTime());

            cachedThreadPool.submit(() -> ExceptionTools.handle(() ->
            {
               synchronized (publisher)
               {
                  publisher.publish(replyMessage);
               }
            }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE));
         }
         else if (!receivedMessage.getIsRequest() && receivedReplyTarget.equals(ourGuid)) // Update clock offset estimate
         {
            ROS2PeerClockOffsetEstimatorPeer peer = peerMap.get(receivedRequestTarget);
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

                     ROS2PeerClockOffsetEstimatorPeer peer = new ROS2PeerClockOffsetEstimatorPeer(guidCopy);
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

   private void runRequestTask()
   {
      if (!peerList.isEmpty())
      {
         if (nextPeerToPing >= peerList.size())
            nextPeerToPing = 0;

         ROS2PeerClockOffsetEstimatorPeer peer = peerList.get(nextPeerToPing);
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
      cachedThreadPool.shutdown();
      requestThread.kill();
      subscription.remove();
      publisher.remove();
   }

   public Guid getOurGuid()
   {
      return ourGuid;
   }

   /** Synchronized so multiple threads can concurrently access peer data when synchronizing on this object. */
   public synchronized ROS2PeerClockOffsetEstimatorPeer getPeer(Guid guid)
   {
      return peerMap.get(guid);
   }

   public List<ROS2PeerClockOffsetEstimatorPeer> getPeerList()
   {
      return peerList;
   }
}
