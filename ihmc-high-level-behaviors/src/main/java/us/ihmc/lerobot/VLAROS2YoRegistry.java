package us.ihmc.lerobot;

import ihmc_common_msgs.msg.dds.YoRegistryMessage;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.SwapReference;
import us.ihmc.yoVariables.registry.YoRegistry;

public class VLAROS2YoRegistry extends VLAYoRegistry
{
   private final Notification notification = new Notification();
   private final SwapReference<YoRegistryMessage> subscription;

   public VLAROS2YoRegistry(ROS2Node ros2Node)
   {
      subscription = ROS2Tools.createSwapReferenceSubscription(ros2Node, VLAUpdateThread.YO, notification);
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
