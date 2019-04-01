package us.ihmc.communication;

import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.Ros2Subscription;

/**
 * TODO This class is broken and not thread safe.
 * @deprecated Be careful using this class! TODO undeprecate and make thread safe, using ROS2Callback
 * @param <T>
 */
public class Ros2QueuedSubscription<T> implements NewMessageListener<T>
{
   private final T data;
   private final ConcurrentRingBuffer<T> messageQueue;
   private final TopicDataType<T> topicDataTypeForCallback;
   private final TopicDataType<T> topicDataTypeForPoll;
   private Ros2Subscription<T> rosSubscription;

   public Ros2QueuedSubscription(TopicDataType<T> topicDataType, int queueDepth)
   {
      this.data = topicDataType.createData();
      this.messageQueue = new ConcurrentRingBuffer<>(() -> topicDataType.createData(), queueDepth);
      this.topicDataTypeForCallback = topicDataType.newInstance();
      this.topicDataTypeForPoll = topicDataType.newInstance();
   }

   public boolean poll(T data)
   {
      if(messageQueue.poll())
      {
         T next = messageQueue.read();
         topicDataTypeForPoll.copy(next, data);
         messageQueue.flush();
         return true;
      }
      else
      {
         return false;
      }
   }

   public boolean flushAndGetLatest(T data)
   {
      if(messageQueue.poll())
      {
         T latest = null;
         T next = null;
         while((next = messageQueue.read()) != null)
         {
            latest = next;
         }
         topicDataTypeForPoll.copy(latest, data);
         messageQueue.flush();
         return true;
      }
      else
      {
         return false;
      }
   }

   /**
    * DO NOT CALL THIS ONE FROM USER PERSPECTIVE. Use flushAndGetLatest and poll.
    */
   @Override
   @Deprecated  // not really deprecated, just to steer users away
   public void onNewDataMessage(Subscriber<T> subscriber)
   {
      if (subscriber.takeNextData(data, null))
      {
         T next = messageQueue.next();
         if (next != null)
         {
            topicDataTypeForCallback.copy(data, next);
            messageQueue.commit();
         }
      }
   }

   void setRos2Subscription(Ros2Subscription<T> rosSubscription)
   {
      this.rosSubscription = rosSubscription;
   }

   public void remove()
   {
      rosSubscription.remove();
   }
}