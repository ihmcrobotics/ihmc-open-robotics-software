package us.ihmc.robotEnvironmentAwareness.ros;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.ros2.Ros2Subscription;

public class REAModuleROS2Subscription<T>
{
   private final String topicName;
   private final Class<T> messageType;
   private final NewMessageListener<T> listener;

   private Ros2Subscription<T> subscription = null;

   public REAModuleROS2Subscription(String topicName, Class<T> messageType, NewMessageListener<T> listener)
   {
      this.topicName = topicName;
      this.messageType = messageType;
      this.listener = listener;
   }

   public void cerate(Ros2Node ros2Node)
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