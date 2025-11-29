package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessage;
import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.SwapReference;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

public class ROS2BehaviorTreeYoRegistry
{
   protected final YoRegistry registry = new YoRegistry("BehaviorTreeExecutor");
   private final Notification notification = new Notification();
   private final SwapReference<BehaviorTreeStateMessage> subscription;

   private final YoBoolean automaticExecution = new YoBoolean("automaticExecution", registry);
   private final YoInteger executionNextIndex = new YoInteger("executionNextIndex", registry);
   private final YoBoolean concurrencyEnabled = new YoBoolean("concurrencyEnabled", registry);

   public ROS2BehaviorTreeYoRegistry(ROS2Node ros2Node)
   {
      subscription = ROS2Tools.createSwapReferenceSubscription(ros2Node, AutonomyAPI.BEHAVIOR_TREE.getStatusTopic(), notification);
   }

   public void update()
   {
      if (notification.poll())
      {
         synchronized (subscription)
         {
            BehaviorTreeStateMessage state = subscription.getForThreadTwo();

            if (!state.getRootNodes().isEmpty())
            {
               BehaviorTreeRootNodeStateMessage root = state.getRootNodes().get(0);
               automaticExecution.set(root.getAutomaticExecution());
               concurrencyEnabled.set(root.getConcurrencyEnabled());
               executionNextIndex.set(root.getExecutionNextIndex());
            }
         }
      }
   }

   public YoRegistry getYoRegistry()
   {
      return registry;
   }
}
