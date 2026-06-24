package us.ihmc.communication.ros2;

import org.apache.commons.lang3.mutable.MutableInt;
import std_msgs.Bool;
import std_msgs.Empty;
import std_msgs.Int32;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.ROS2Input;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.log.LogTools;
import us.ihmc.tools.Destroyable;
import us.ihmc.tools.thread.SwapReference;

import java.util.function.Consumer;

/**
 * Supports:
 * - Publishing on the fly without having to first create publishers
 * - Disabling and enabling all the publishers and subscribers created here
 */
public class ROS2Helper
{
   protected final ROS2Node ros2Node;
   protected final ROS2PublisherMap ros2PublisherMap;

   public ROS2Helper(String nodeName)
   {
      this(new ROS2Node(nodeName));
   }

   public ROS2Helper(ROS2Node ros2Node)
   {
      this.ros2Node = ros2Node;
      ros2PublisherMap = new ROS2PublisherMap(ros2Node);
   }

   public <T extends ROS2Message<T>> void subscribeViaCallback(ROS2Topic<T> topic, Consumer<T> callback)
   {
      ros2Node.createSubscription(topic, reader ->
      {
         T message = reader.read();
         if (message != null)
            callback.accept(message);
      });
   }

   /**
    * Allocation free callback where the user only has access to the message in the callback.
    * The user should not take up any significant time in the callback to not slow down the ROS 2
    * thread.
    */
   public <T extends ROS2Message<T>> void subscribeViaVolatileCallback(ROS2Topic<T> topic, Consumer<T> callback)
   {
      ros2Node.createSubscription(topic, reader ->
      {
         T message = reader.read();
         if (message != null)
            callback.accept(message);
      });
   }

   /** Use when you only need the latest message and need allocation free. */
   public <T extends ROS2Message<T>> SwapReference<T> subscribeViaSwapReference(ROS2Topic<T> topic, Consumer<T> callback)
   {
      SwapReference<T> swapReference = new SwapReference<>(() -> ROS2Message.createInstance(topic.getType()));

      ros2Node.createSubscription(topic, reader ->
      {
         T readMessage = reader.read();
         if (readMessage != null)
         {
            T messageToPack = swapReference.getForThreadOne();
            messageToPack.set(readMessage);
            swapReference.swap();
            callback.accept(messageToPack);
         }
      });
      return swapReference;
   }

   /** Use when you only need the latest message and need allocation free. */
   public <T extends ROS2Message<T>> SwapReference<T> subscribeViaSwapReference(ROS2Topic<T> topic, Notification callback)
   {
      return subscribeViaSwapReference(topic, message -> callback.set());
   }

   public static class QueuedSubscription<T extends ROS2Message<T>> implements Destroyable
   {
      private final ConcurrentRingBuffer<T> queue;
      private final ROS2Subscription<T> subscription;
      private final ROS2Node ros2Node;

      private QueuedSubscription(ConcurrentRingBuffer<T> queue, ROS2Subscription<T> subscription, ROS2Node ros2Node)
      {
         this.queue = queue;
         this.subscription = subscription;
         this.ros2Node = ros2Node;
      }

      public ConcurrentRingBuffer<T> getQueue()
      {
         return queue;
      }

      @Override
      public void destroy()
      {
         ros2Node.destroySubscription(subscription);
      }
   }

   public <T extends ROS2Message<T>> QueuedSubscription<T> subscribeViaQueueWithHandle(ROS2Topic<T> topic, int queueSize, Consumer<T> callback)
   {
      ConcurrentRingBuffer<T> concurrentQueue = new ConcurrentRingBuffer<>(() -> ROS2Message.createInstance(topic.getType()), queueSize);
      Throttler warningThrottler = new Throttler().setFrequency(1.0);
      MutableInt droppedMessages = new MutableInt(0);
      ROS2Subscription<T> subscription = ros2Node.createSubscription(topic, reader ->
      {
         // Make sure we are recieving newer data and throw out old data
         T nextData;
         while ((nextData = concurrentQueue.next()) == null)
         {
            droppedMessages.increment();

            if (warningThrottler.run())
            {
               LogTools.warn("Concurrent ring buffer has been full! Queue size: {} Have dropped {} oldest messages...", queueSize, droppedMessages.intValue());
               droppedMessages.setValue(0);
            }

            concurrentQueue.poll();
            concurrentQueue.read();
            concurrentQueue.flush();
         }

         T readMessage = reader.read();
         if (readMessage != null)
         {
            nextData.set(readMessage);
            callback.accept(nextData);
            concurrentQueue.commit();
         }
      });
      return new QueuedSubscription<>(concurrentQueue, subscription, ros2Node);
   }

   public void subscribeViaCallback(ROS2Topic<Empty> topic, Runnable callback)
   {
      ros2Node.createSubscription(topic, reader -> callback.run());
   }

   public <T extends ROS2Message<T>> ROS2Input<T> subscribe(ROS2Topic<T> topic)
   {
      return new ROS2Input<>(ros2Node, topic);
   }

   public <T extends ROS2Message<T>> ROS2Input<T> subscribe(ROS2Topic<T> topic, ROS2Input.MessageFilter<T> messageFilter)
   {
      return new ROS2Input<>(ros2Node, topic, ROS2Message.createInstance(topic.getType()), messageFilter);
   }

   public <T extends ROS2Message<T>> TypedNotification<T> subscribeViaTypedNotification(ROS2Topic<T> topic)
   {
      TypedNotification<T> typedNotification = new TypedNotification<>();
      ros2Node.createSubscription(topic, reader ->
      {
         T message = reader.read();
         if (message != null)
            typedNotification.set(message);
      });
      return typedNotification;
   }

   public <T extends ROS2Message<T>> void createPublisher(ROS2Topic<T> topic)
   {
      ros2PublisherMap.getOrCreatePublisher(topic);
   }

   public <T extends ROS2Message<T>> void publish(ROS2Topic<T> topic, T message)
   {
      ros2PublisherMap.publish(topic, message);
   }

   public void publish(ROS2Topic<std_msgs.String_> topic, String message)
   {
      std_msgs.String_ stringMessage = new std_msgs.String_();
      stringMessage.setData(message);
      ros2PublisherMap.publish(topic, stringMessage);
   }

   public void publish(ROS2Topic<Empty> topic)
   {
      ros2PublisherMap.publish(topic);
   }

   public void publish(ROS2Topic<Int32> topic, int value)
   {
      Int32 message = new Int32();
      message.setData(value);
      ros2PublisherMap.publish(topic, message);
   }

   public void publish(ROS2Topic<Bool> topic, boolean message)
   {
      ros2PublisherMap.publish(topic, message);
   }

   public ROS2Node getROS2Node()
   {
      return ros2Node;
   }
}
