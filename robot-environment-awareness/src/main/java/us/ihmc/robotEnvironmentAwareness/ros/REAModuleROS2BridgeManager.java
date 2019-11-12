package us.ihmc.robotEnvironmentAwareness.ros;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.Ros2Node;

public class REAModuleROS2BridgeManager
{
   private final Ros2Node ros2Node;
   private final Messager reaMessager;
   private final List<REASourceType> reaSourceTypes = new ArrayList<>();
   private final EnumMap<REASourceType, REAModuleROS2Subscription<?>> subscriptionMap = new EnumMap<>(REASourceType.class);
   private final EnumMap<REASourceType, AtomicReference<Boolean>> togglerMap = new EnumMap<>(REASourceType.class);

   public REAModuleROS2BridgeManager(Ros2Node ros2Node, Messager reaMessager)
   {
      this.ros2Node = ros2Node;
      this.reaMessager = reaMessager;
   }

   public <T> void addSubscription(REASourceType sourceType, Class<T> messageType, NewMessageListener<T> listener, Topic<Boolean> enable)
   {
      reaSourceTypes.add(sourceType);
      subscriptionMap.put(sourceType, new REAModuleROS2Subscription<T>(sourceType.getTopicName(), messageType, listener));
      togglerMap.put(sourceType, reaMessager.createInput(enable, false));
   }

   public void update()
   {
      for (int i = 0; i < reaSourceTypes.size(); i++)
      {
         REASourceType sourceType = reaSourceTypes.get(i);
         if (togglerMap.get(sourceType).get())
         {
            subscriptionMap.get(sourceType).cerate(ros2Node);
         }
         else
         {
            subscriptionMap.get(sourceType).remove();
         }
      }
   }
}
