package us.ihmc.communication.ros2;

import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.attributes.PublisherAttributes;
import us.ihmc.pubsub.attributes.SubscriberAttributes;
import us.ihmc.ros2.*;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicBoolean;

public class ManagedROS2Node implements ROS2NodeInterface
{
   private final ROS2NodeInterface ros2Node;
   private final AtomicBoolean enabled = new AtomicBoolean(true);

   public ManagedROS2Node(ROS2NodeInterface ros2Node)
   {
      this.ros2Node = ros2Node;
   }

   /**
    * The whole point of this class. Disable and enable ROS 2 listeners and publishers created elsewhere.
    */
   public void setEnabled(boolean enabled)
   {
      this.enabled.set(enabled);
   }

   @Override
   public <T> PublisherAttributes createPublisherAttributes(TopicDataType<T> topicDataType, String topicName, ROS2QosProfile qosProfile)
   {
      return ros2Node.createPublisherAttributes(topicDataType, topicName,qosProfile);
   }

   @Override
   public <T> ROS2PublisherBasics<T> createPublisher(TopicDataType<T> topicDataType, PublisherAttributes publisherAttributes) throws IOException
   {
      return createManagedPublisher(ros2Node.createPublisher(topicDataType, publisherAttributes));
   }

   @Override
   public <T> ROS2PublisherBasics<T> createPublisher(TopicDataType<T> topicDataType, String topicName) throws IOException
   {
      return createManagedPublisher(ros2Node.createPublisher(topicDataType, topicName, ROS2QosProfile.BEST_EFFORT()));
   }

   @Override
   public <T> ROS2PublisherBasics<T> createPublisher(TopicDataType<T> topicDataType, String topicName, ROS2QosProfile qosProfile) throws IOException
   {
      return createManagedPublisher(ros2Node.createPublisher(topicDataType, topicName, qosProfile));
   }

   private <T> ManagedROS2Publisher<T> createManagedPublisher(ROS2PublisherBasics<T> publisher)
   {
      return new ManagedROS2Publisher<>(publisher, enabled::get);
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
//      return ros2Node.createQueuedSubscription(topicDataType, topicName, qosProfile, queueSize);
   }

   @Override
   public <T> ROS2Subscription<T> createSubscription(TopicDataType<T> topicDataType, NewMessageListener<T> newMessageListener, String topicName)
         throws IOException
   {
      return ros2Node.createSubscription(topicDataType, createManagedListener(newMessageListener), topicName, ROS2QosProfile.BEST_EFFORT());
   }

   @Override
   public <T> ROS2Subscription<T> createSubscription(TopicDataType<T> topicDataType,
                                                     NewMessageListener<T> newMessageListener,
                                                     String topicName,
                                                     ROS2QosProfile qosProfile) throws IOException
   {
      return ros2Node.createSubscription(topicDataType, createManagedListener(newMessageListener), topicName, qosProfile);
   }

   @Override
   public <T> ROS2Subscription<T> createSubscription(TopicDataType<T> topicDataType,
                                                     NewMessageListener<T> newMessageListener,
                                                     SubscriptionMatchedListener<T> subscriptionMatchedListener,
                                                     String topicName,
                                                     ROS2QosProfile qosProfile) throws IOException
   {
      ManagedROS2Listener<T> managedListener = new ManagedROS2Listener<T>(newMessageListener, subscriptionMatchedListener, enabled::get);
      return ros2Node.createSubscription(topicDataType, managedListener, managedListener, topicName, qosProfile);
   }

   private <T> ManagedROS2Listener<T> createManagedListener(NewMessageListener<T> newMessageListener)
   {
      return new ManagedROS2Listener<T>(newMessageListener, enabled::get);
   }

   public boolean isEnabled()
   {
      return enabled.get();
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
}
