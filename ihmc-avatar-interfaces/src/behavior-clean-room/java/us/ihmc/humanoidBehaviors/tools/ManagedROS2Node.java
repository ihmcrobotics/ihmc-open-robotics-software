package us.ihmc.humanoidBehaviors.tools;

import us.ihmc.communication.ROS2Callback;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.ros2.*;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class ManagedROS2Node implements Ros2NodeInterface
{
   private final List<ROS2Callback> ros2Callbacks = new ArrayList<>();
   private final Ros2Node ros2Node;

   public ManagedROS2Node(Ros2Node ros2Node)
   {
      this.ros2Node = ros2Node;
   }

   public void setEnabled(boolean enabled)
   {
      for (ROS2Callback ros2Callback : ros2Callbacks)
      {
         ros2Callback.setEnabled(enabled);
      }
   }

   @Override
   public <T> Ros2Publisher<T> createPublisher(TopicDataType<T> topicDataType, String topicName) throws IOException
   {
      return ros2Node.createPublisher(topicDataType, topicName);
   }

   @Override
   public <T> Ros2Publisher<T> createPublisher(TopicDataType<T> topicDataType, String topicName, Ros2QosProfile qosProfile) throws IOException
   {
      return ros2Node.createPublisher(topicDataType, topicName, qosProfile);
   }

   @Override
   public <T> Ros2Subscription<T> createSubscription(TopicDataType<T> topicDataType, NewMessageListener<T> newMessageListener, String topicName)
         throws IOException
   {
      return ros2Node.createSubscription(topicDataType, newMessageListener, topicName);
   }

   @Override
   public <T> Ros2Subscription<T> createSubscription(TopicDataType<T> topicDataType,
                                                     NewMessageListener<T> newMessageListener,
                                                     String topicName,
                                                     Ros2QosProfile qosProfile) throws IOException
   {
      return ros2Node.createSubscription(topicDataType, newMessageListener, topicName, qosProfile);
   }

   @Override
   public <T> Ros2Subscription<T> createSubscription(TopicDataType<T> topicDataType,
                                                     NewMessageListener<T> newMessageListener,
                                                     SubscriptionMatchedListener<T> subscriptionMatchedListener,
                                                     String topicName,
                                                     Ros2QosProfile qosProfile) throws IOException
   {
      return ros2Node.createSubscription(topicDataType,newMessageListener, subscriptionMatchedListener,topicName, qosProfile);
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
