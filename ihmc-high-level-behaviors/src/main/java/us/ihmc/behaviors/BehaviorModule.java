package us.ihmc.behaviors;

import behavior_msgs.msg.dds.BehaviorTreeMessage;
import behavior_msgs.msg.dds.StatusLogMessage;

import std_msgs.msg.dds.Empty;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.tools.BehaviorMessageTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.Destroyable;

/**
 * Manages a behavior tree based process on the real robot and syncs it
 * with an operator UI with shared autonomy.
 */
public class BehaviorModule
{
   private final BehaviorTreeMessage behaviorTreeMessage = new BehaviorTreeMessage();
   private final StatusLogger statusLogger;
   private final BehaviorTreeNodeExecutor rootNode;
   private final BehaviorHelper behaviorHelper;

   public BehaviorModule(BehaviorTreeNodeExecutor rootNode, BehaviorHelper behaviorHelper)
   {
      this.rootNode = rootNode;
      this.behaviorHelper = behaviorHelper;

      statusLogger = behaviorHelper.getOrCreateStatusLogger();

      behaviorHelper.subscribeViaCallback(API.SHUTDOWN, message ->
      {
         statusLogger.info("Received SHUTDOWN. Shutting down...");
         ThreadTools.startAsDaemon(this::destroy, "DestroyThread");
      });
   }

   public void update()
   {
      rootNode.tick();
      behaviorTreeMessage.getNodes().clear();
      BehaviorMessageTools.packBehaviorTreeMessage(rootNode, behaviorTreeMessage);
      behaviorHelper.publish(API.BEHAVIOR_TREE_STATUS, behaviorTreeMessage);
   }

   public void destroy()
   {
      statusLogger.info("Shutting down...");
      behaviorHelper.destroy();
      if (rootNode instanceof Destroyable destroyable)
         destroyable.destroy();
   }

   // API created here from build
   public static class API
   {
      public static final ROS2Topic<?> BASE_TOPIC = ROS2Tools.BEHAVIOR_MODULE;
      public static final ROS2Topic<Empty> SHUTDOWN = BASE_TOPIC.withInput().withType(Empty.class).withSuffix("shutdown");
      public static final ROS2Topic<StatusLogMessage> STATUS_LOG
                                            = BASE_TOPIC.withOutput().withType(StatusLogMessage.class).withSuffix("status_log");
      public static final ROS2Topic<BehaviorTreeMessage> BEHAVIOR_TREE_STATUS
                                            = BASE_TOPIC.withOutput().withType(BehaviorTreeMessage.class).withSuffix("behavior_tree_status");
   }
}
