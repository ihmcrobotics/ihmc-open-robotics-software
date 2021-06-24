package us.ihmc.robotEnvironmentAwareness.ros;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Subscription;

public class REAModuleROS2Subscription<T>
{
   private final String topicName;
   private final Class<T> messageType;
   private final NewMessageListener<T> listener;

   private ROS2Subscription<T> subscription = null;

   public REAModuleROS2Subscription(ROS2NodeInterface ros2Node, Messager messager, REASourceType reaSourceType, Class<T> messageType, NewMessageListener<T> listener)
   {
      this(ros2Node, messager, reaSourceType.getTopicName(), messageType, listener, reaSourceType.getEnableTopic());
   }

   public REAModuleROS2Subscription(ROS2NodeInterface node, Messager messager, String name, Class<T> Type, NewMessageListener<T> listener, Topic<Boolean> enableTopic)
   {
      this.topicName = name;
      this.messageType = Type;
      this.listener = listener;

      messager.registerTopicListener(enableTopic, (enable) -> handle(node, enable));
   }

   public void handle(ROS2NodeInterface ros2Node, boolean enable)
   {
      if (enable)
         create(ros2Node);
      else
         remove();
   }

   public void create(ROS2NodeInterface ros2Node)
   {
      if (subscription == null)
         subscription = ROS2Tools.createCallbackSubscription(ros2Node, messageType, topicName, listener);
   }

   public void remove()
   {
      if (subscription != null)
      {
         subscription.remove();
         subscription = null;
      }
   }
}