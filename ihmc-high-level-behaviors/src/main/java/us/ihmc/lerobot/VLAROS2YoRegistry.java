package us.ihmc.lerobot;

import ihmc_common_msgs.YoRegistryMessage;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.tools.thread.SwapReference;
import us.ihmc.yoVariables.registry.YoRegistry;

public class VLAROS2YoRegistry extends VLAYoRegistry
{
   private final Notification notification = new Notification();
   private final SwapReference<YoRegistryMessage> subscription;

   public VLAROS2YoRegistry(ROS2Node ros2Node)
   {
      var yoTopic = VLAUpdateThread.YO;
      SwapReference<YoRegistryMessage> swapReference = new SwapReference<>(() -> ROS2Message.createInstance(yoTopic.getType()));
      ros2Node.createSubscriptionSampler(yoTopic, sample ->
      {
         var messageToPack = swapReference.getForThreadOne();
         messageToPack.set(sample);
         swapReference.swap();
         notification.set();
      });
      subscription = swapReference;
   }

   public void update()
   {
      if (notification.poll())
      {
         synchronized (subscription)
         {
            MessageTools.fromMessage(subscription.getForThreadTwo(), registry);
         }
      }
   }

   public YoRegistry getYoRegistry()
   {
      return registry;
   }
}
