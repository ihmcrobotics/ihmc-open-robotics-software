package us.ihmc.communication.ros2.sync;

import ihmc_common_msgs.PeerClockOffsetEstimatorPingMessage;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.jros2.Guid;
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
   private static final ROS2Topic<PeerClockOffsetEstimatorPingMessage> TOPIC = ROS2Tools.IHMC_ROOT.withModule("peer_clock_offset_estimator")
                                                                                                  .withType(PeerClockOffsetEstimatorPingMessage.class);

   private final HashMap<Guid, ROS2PeerClockOffsetEstimatorPeer> peerMap = new HashMap<>();
   private final List<ROS2PeerClockOffsetEstimatorPeer> peerList = new ArrayList<>();
   private int nextPeerToPing = 0;
   private final ROS2Node ros2Node;
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
   private final Guid receivedRequestTarget = new Guid();
   private final Guid receivedReplyTarget = new Guid();
   private volatile boolean destroyed = false;

   public ROS2PeerClockOffsetEstimator(ROS2Node ros2Node)
   {
      this.ros2Node = ros2Node;
      publisher = ros2Node.createPublisher(TOPIC);
      ourGuid = new Guid();
      ourGuid.set(publisher.getGuid());

      subscription = ros2Node.createSubscription(TOPIC, reader ->
      {
         if (destroyed)
            return;

         PeerClockOffsetEstimatorPingMessage message = reader.read();
         if (message == null)
            return;

         receivedMessage.set(message);

         MessageTools.fromMessage(receivedMessage.getRequestTarget(), receivedRequestTarget);
         MessageTools.fromMessage(receivedMessage.getReplyTarget(), receivedReplyTarget);

         if (receivedMessage.getIsRequest() && receivedRequestTarget.equals(ourGuid)) // Reply
         {
            if (destroyed)
               return;

            addPeerIfAbsent(receivedReplyTarget);

            PeerClockOffsetEstimatorPingMessage replyMessage = new PeerClockOffsetEstimatorPingMessage();
            replyMessage.set(receivedMessage);
            replyMessage.setIsRequest(false);
            MessageTools.toMessage(Instant.now(), replyMessage.getReplySendTime());

            if (!cachedThreadPool.isShutdown())
            {
               cachedThreadPool.submit(() -> ExceptionTools.handle(() ->
               {
                  if (destroyed)
                     return;

                  synchronized (publisher)
                  {
                     publisher.publish(replyMessage);
                  }
               }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE));
            }
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
      }, (subscriber, publicationGuid, matched) ->
      {
         if (!publicationGuid.equals(ourGuid))
         {
            if (matched)
            {
               addPeerIfAbsent(publicationGuid);
            }
            else
            {
               peerMap.remove(publicationGuid);
               peerList.removeIf(peer -> peer.getGuid().equals(publicationGuid));
            }
         }
      });

      requestThread.setFrequencyLimit(5.0);
      requestThread.startRepeating();
   }

   private void addPeerIfAbsent(Guid peerGuid)
   {
      if (peerGuid.equals(ourGuid) || peerMap.containsKey(peerGuid))
         return;

      Guid guidCopy = new Guid();
      guidCopy.set(peerGuid);

      ROS2PeerClockOffsetEstimatorPeer peer = new ROS2PeerClockOffsetEstimatorPeer(guidCopy);
      peerMap.put(guidCopy, peer);
      peerList.add(peer);
   }

   private void runRequestTask()
   {
      if (destroyed || peerList.isEmpty())
         return;

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

   public void destroy()
   {
      destroyed = true;

      requestThread.blockingKill();
      ros2Node.destroySubscription(subscription);
      cachedThreadPool.shutdown();
      ros2Node.destroyPublisher(publisher);
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
