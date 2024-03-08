package us.ihmc.communication.ros2;

import java.io.IOException;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.attributes.PublisherAttributes;
import us.ihmc.pubsub.attributes.SubscriberAttributes;
import us.ihmc.pubsub.common.Guid;
import us.ihmc.pubsub.common.MatchingInfo;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.QueuedROS2Subscription;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.SubscriptionMatchedListener;

/**
 * This ROS2 node is for testing purposes! Do NOT use for normal operation.
 * <p>
 * This node allows to artificially introduce delay to simulate communication delays on an actual
 * network.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class DelayedROS2Node implements ROS2NodeInterface
{
   private final ROS2NodeInterface ros2Node;
   private final Executor delayedPubExecutor;
   private final Executor delayedSubExecutor;
   private final AtomicLong delayPubInMillis = new AtomicLong(0);
   private final AtomicLong delaySubInMillis = new AtomicLong(0);

   public DelayedROS2Node(ROS2NodeInterface ros2Node)
   {
      this.ros2Node = ros2Node;

      ScheduledExecutorService scheduledPubExecutor = Executors.newSingleThreadScheduledExecutor(ThreadTools.createNamedDaemonThreadFactory(ros2Node.getName()
            + " - PubDelay"));

      delayedPubExecutor = task ->
      {
         if (task == null)
            return;
         if (delayPubInMillis.get() <= 0)
            task.run();
         else
            scheduledPubExecutor.schedule(task, delayPubInMillis.get(), TimeUnit.MILLISECONDS);
      };

      ScheduledExecutorService scheduledSubExecutor = Executors.newSingleThreadScheduledExecutor(ThreadTools.createNamedDaemonThreadFactory(ros2Node.getName()
            + " - SubDelay"));

      delayedSubExecutor = task ->
      {
         if (task == null)
            return;
         if (delaySubInMillis.get() <= 0)
            task.run();
         else
            scheduledSubExecutor.schedule(task, delaySubInMillis.get(), TimeUnit.MILLISECONDS);
      };
   }

   public void setDelay(long delayInMillis)
   {
      setDelays(delayInMillis, delayInMillis);
   }

   public void setDelays(long delayPubInMillis, long delaySubInMillis)
   {
      this.delayPubInMillis.set(delayPubInMillis);
      this.delaySubInMillis.set(delaySubInMillis);
   }

   @Override
   public <T> PublisherAttributes createPublisherAttributes(TopicDataType<T> topicDataType, String topicName, ROS2QosProfile qosProfile)
   {
      return ros2Node.createPublisherAttributes(topicDataType, topicName,qosProfile);
   }

   @Override
   public <T> ROS2PublisherBasics<T> createPublisher(TopicDataType<T> topicDataType, PublisherAttributes publisherAttributes) throws IOException
   {
      return createDelayedPublisher(ros2Node.createPublisher(topicDataType, publisherAttributes));
   }

   @Override
   public <T> ROS2PublisherBasics<T> createPublisher(TopicDataType<T> topicDataType, String topicName) throws IOException
   {
      return createDelayedPublisher(ros2Node.createPublisher(topicDataType, topicName, ROS2QosProfile.BEST_EFFORT()));
   }

   @Override
   public <T> ROS2PublisherBasics<T> createPublisher(TopicDataType<T> topicDataType, String topicName, ROS2QosProfile qosProfile) throws IOException
   {
      return createDelayedPublisher(ros2Node.createPublisher(topicDataType, topicName, qosProfile));
   }

   private <T> DelayedROS2Publisher<T> createDelayedPublisher(ROS2PublisherBasics<T> publisher)
   {
      return new DelayedROS2Publisher<>(publisher, delayedPubExecutor);
   }

   @Override
   public <T> SubscriberAttributes createSubscriberAttributes(String topicName, TopicDataType<T> topicDataType, ROS2QosProfile qosProfile)
   {
      return ros2Node.createSubscriberAttributes(topicName, topicDataType, qosProfile);
   }

   @Override
   public <T> ROS2Subscription<T> createSubscription(TopicDataType<T> topicDataType,
                                                     NewMessageListener<T> subscriberListener,
                                                     SubscriberAttributes subscriberAttributes) throws IOException
   {
      return ros2Node.createSubscription(topicDataType, subscriberListener, subscriberAttributes);
   }

   public <T> QueuedROS2Subscription<T> createQueuedSubscription(TopicDataType<T> topicDataType, SubscriberAttributes subscriberAttributes, int queueSize)
         throws IOException
   {
      return ros2Node.createQueuedSubscription(topicDataType, subscriberAttributes, queueSize);
   }

   @Override
   public <T> QueuedROS2Subscription<T> createQueuedSubscription(TopicDataType<T> topicDataType, String topicName, ROS2QosProfile qosProfile, int queueSize)
         throws IOException
   {
      throw new RuntimeException("This funtionality is so far unimplemented. Implement me!");
   }

   @Override
   public <T> ROS2Subscription<T> createSubscription(TopicDataType<T> topicDataType, NewMessageListener<T> newMessageListener, String topicName)
         throws IOException
   {
      return ros2Node.createSubscription(topicDataType, new DelayedROS2Listener<>(topicDataType, newMessageListener, delayedSubExecutor), topicName, ROS2QosProfile.BEST_EFFORT());
   }

   @Override
   public <T> ROS2Subscription<T> createSubscription(TopicDataType<T> topicDataType,
                                                     NewMessageListener<T> newMessageListener,
                                                     String topicName,
                                                     ROS2QosProfile qosProfile)
         throws IOException
   {
      return ros2Node.createSubscription(topicDataType,
                                         new DelayedROS2Listener<>(topicDataType, newMessageListener, delayedSubExecutor),
                                         topicName,
                                         qosProfile);
   }

   @Override
   public <T> ROS2Subscription<T> createSubscription(TopicDataType<T> topicDataType,
                                                     NewMessageListener<T> newMessageListener,
                                                     SubscriptionMatchedListener<T> subscriptionMatchedListener,
                                                     String topicName,
                                                     ROS2QosProfile qosProfile)
         throws IOException
   {
      DelayedROS2Listener<T> managedListener = new DelayedROS2Listener<T>(topicDataType, newMessageListener, delayedSubExecutor);
      return ros2Node.createSubscription(topicDataType, managedListener, managedListener, topicName, qosProfile);
   }

   @Override
   public String getName()
   {
      return ros2Node.getName();
   }

   @Override
   public String getNamespace()
   {
      return ros2Node.getNamespace();
   }

   public static class DelayedROS2Listener<T> implements NewMessageListener<T>, SubscriptionMatchedListener<T>
   {
      private final NewMessageListener<T> listener;
      private final Executor delayedExecutor;
      private final TopicDataType<T> topicDataType;

      public DelayedROS2Listener(TopicDataType<T> topicDataType, NewMessageListener<T> listener, Executor delayedExecutor)
      {
         this.topicDataType = topicDataType;
         this.listener = listener;
         this.delayedExecutor = delayedExecutor;
      }

      @Override
      public void onNewDataMessage(Subscriber<T> subscriber)
      {
         DelayedSubscriber<T> delayedSubscriber = new DelayedSubscriber<>(topicDataType, subscriber);
         delayedExecutor.execute(() -> listener.onNewDataMessage(delayedSubscriber));
      }

      @Override
      public void onSubscriptionMatched(Subscriber<T> subscriber, MatchingInfo info)
      {
         // do nothing
      }
   }

   private static class DelayedSubscriber<T> implements Subscriber<T>
   {
      private final Subscriber<T> subscriber;
      private final T nextData;
      private final TopicDataType<T> topicDataType;
      private final SampleInfo info = new SampleInfo();
      private boolean dataTaken = false;

      public DelayedSubscriber(TopicDataType<T> topicDataType, Subscriber<T> subscriber)
      {
         this.topicDataType = topicDataType;
         this.subscriber = subscriber;
         nextData = topicDataType.createData();
         subscriber.takeNextData(nextData, info);
      }

      @Override
      public Guid getGuid()
      {
         return subscriber.getGuid();
      }

      @Override
      public void waitForUnreadMessage(int timeoutInMilliseconds) throws InterruptedException
      {

      }

      @Override
      public boolean readNextData(T data, SampleInfo info)
      {
         if (dataTaken)
            return false;
         topicDataType.copy(nextData, data);
         info.set(this.info);
         return true;
      }

      @Override
      public boolean takeNextData(T data, SampleInfo info)
      {
         if (dataTaken)
            return false;
         topicDataType.copy(nextData, data);
         if (info != null)
            info.set(this.info);
         dataTaken = true;
         return true;
      }

      @Override
      public T readNextData()
      {
         return readNextData(null);
      }

      @Override
      public T readNextData(SampleInfo info)
      {
         if (dataTaken)
            return null;
         T ret = topicDataType.createData();
         readNextData(ret, info);
         return ret;
      }

      @Override
      public T takeNextData()
      {
         return takeNextData(info);
      }

      @Override
      public T takeNextData(SampleInfo info)
      {
         if (dataTaken)
            return null;
         T ret = topicDataType.createData();
         takeNextData(ret, info);
         return ret;
      }

      @Override
      public SubscriberAttributes getAttributes()
      {
         return subscriber.getAttributes();
      }

      @Override
      public boolean isInCleanState()
      {
         return subscriber.isInCleanState();
      }

      @Override
      public boolean isAvailable()
      {
         return subscriber.isAvailable() && !dataTaken;
      }
   }

   public static class DelayedROS2Publisher<T> implements ROS2PublisherBasics<T>
   {
      private final ROS2PublisherBasics<T> publisher;
      private final Executor delayedExecutor;

      public DelayedROS2Publisher(ROS2PublisherBasics<T> publisher, Executor delayedExecutor)
      {
         this.publisher = publisher;
         this.delayedExecutor = delayedExecutor;
      }

      @Override
      public boolean publish(T data)
      {
         delayedExecutor.execute(() ->
         {
            publisher.publish(data);
         });
         return true;
      }

      @Override
      public void remove()
      {
         publisher.remove();
      }
   }
}
