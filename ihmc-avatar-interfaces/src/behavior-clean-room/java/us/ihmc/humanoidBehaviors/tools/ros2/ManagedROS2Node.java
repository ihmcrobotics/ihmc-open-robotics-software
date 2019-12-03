package us.ihmc.humanoidBehaviors.tools.ros2;

import us.ihmc.pubsub.TopicDataType;
import us.ihmc.ros2.*;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class ManagedROS2Node implements Ros2NodeInterface
{
   private final List<ManagedROS2Listener> ros2MessageListeners = new ArrayList<>();
   private final List<ManagedROS2Publisher> ros2Publishers = new ArrayList<>();
   private final Ros2Node ros2Node;

   public ManagedROS2Node(Ros2Node ros2Node)
   {
      this.ros2Node = ros2Node;
   }

   /**
    * The whole point of this class. Disable and enable ROS 2 listeners and publishers created elsewhere.
    */
   public void setEnabled(boolean enabled)
   {
      for (ManagedROS2Listener ros2Listener : ros2MessageListeners)
      {
         ros2Listener.setEnabled(enabled);
      }
      for (ManagedROS2Publisher ros2Publisher : ros2Publishers)
      {
         ros2Publisher.setEnabled(enabled);
      }
   }

   @Override
   public <T> Ros2PublisherBasics<T> createPublisher(TopicDataType<T> topicDataType, String topicName) throws IOException
   {
      return createManagedPublisher(ros2Node.createPublisher(topicDataType, topicName));
   }

   @Override
   public <T> Ros2PublisherBasics<T> createPublisher(TopicDataType<T> topicDataType, String topicName, Ros2QosProfile qosProfile) throws IOException
   {
      return createManagedPublisher(ros2Node.createPublisher(topicDataType, topicName, qosProfile));
   }

   private <T> ManagedROS2Publisher<T> createManagedPublisher(Ros2PublisherBasics<T> publisher)
   {
      ManagedROS2Publisher<T> managedPublisher = new ManagedROS2Publisher<T>(publisher);
      ros2Publishers.add(managedPublisher);
      return managedPublisher;
   }

   @Override
   public <T> Ros2Subscription<T> createSubscription(TopicDataType<T> topicDataType, NewMessageListener<T> newMessageListener, String topicName)
         throws IOException
   {
      return ros2Node.createSubscription(topicDataType, createManagedListener(newMessageListener), topicName);
   }

   @Override
   public <T> Ros2Subscription<T> createSubscription(TopicDataType<T> topicDataType,
                                                     NewMessageListener<T> newMessageListener,
                                                     String topicName,
                                                     Ros2QosProfile qosProfile) throws IOException
   {
      return ros2Node.createSubscription(topicDataType, createManagedListener(newMessageListener), topicName, qosProfile);
   }

   @Override
   public <T> Ros2Subscription<T> createSubscription(TopicDataType<T> topicDataType,
                                                     NewMessageListener<T> newMessageListener,
                                                     SubscriptionMatchedListener<T> subscriptionMatchedListener,
                                                     String topicName,
                                                     Ros2QosProfile qosProfile) throws IOException
   {
      ManagedROS2Listener managedListener = new ManagedROS2Listener(newMessageListener, subscriptionMatchedListener);
      ros2MessageListeners.add(managedListener);
      return ros2Node.createSubscription(topicDataType, managedListener, managedListener, topicName, qosProfile);
   }

   private <T> ManagedROS2Listener createManagedListener(NewMessageListener<T> newMessageListener)
   {
      ManagedROS2Listener managedListener = new ManagedROS2Listener(newMessageListener);
      ros2MessageListeners.add(managedListener);
      return managedListener;
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
