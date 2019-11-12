package us.ihmc.robotEnvironmentAwareness.ros;

import java.util.EnumMap;

import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.Ros2Node;

public class REAModuleROS2BridgeManager
{
   private final Ros2Node ros2Node;
   private final EnumMap<REASourceType, REAModuleROS2Subscription<?>> subscriptionMap = new EnumMap<>(REASourceType.class);

   public REAModuleROS2BridgeManager(Ros2Node ros2Node)
   {
      this.ros2Node = ros2Node;
   }

   //   public static void main(String[] args)
   //   {
   //      REAModuleROS2BridgeManager aa = new REAModuleROS2BridgeManager(ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, ROS2Tools.REA.getNodeName()));
   //   }

   public <T> void addSubscription(REASourceType sourceType, Class<T> messageType, NewMessageListener<T> listener)
   {
      subscriptionMap.put(sourceType, new REAModuleROS2Subscription<T>(sourceType.getTopicName(), messageType, listener));
   }

   public void startSubscription(REASourceType sourceType, boolean thisOnly)
   {
      startSubscription(sourceType);
      if (thisOnly)
      {
         for (REASourceType type : REASourceType.values())
         {
            removeSubscription(type);
         }
      }
   }

   private void startSubscription(REASourceType sourceType)
   {
      if (subscriptionMap.get(sourceType) != null)
         subscriptionMap.get(sourceType).cerate(ros2Node);
   }

   private void removeSubscription(REASourceType sourceType)
   {
      if (subscriptionMap.get(sourceType) != null)
         subscriptionMap.get(sourceType).remove();
   }
}
