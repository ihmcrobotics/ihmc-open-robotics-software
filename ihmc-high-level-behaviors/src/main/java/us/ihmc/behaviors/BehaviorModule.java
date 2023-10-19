package us.ihmc.behaviors;

import behavior_msgs.msg.dds.BehaviorTreeMessage;
import behavior_msgs.msg.dds.StatusLogMessage;

import std_msgs.msg.dds.Empty;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.tools.BehaviorMessageTools;
import us.ihmc.behaviors.tools.behaviorTree.*;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.PausablePeriodicThread;

/**
 * Manages a behavior tree based process on the real robot and syncs it
 * with an operator UI with shared autonomy.
 */
public class BehaviorModule
{
   private final ROS2Node ros2Node;
   private final BehaviorTreeControlFlowNode rootNode;
   private final BehaviorRegistry behaviorRegistry;
   private final DRCRobotModel robotModel;
   private final BehaviorDefinition highestLevelBehaviorDefinition;
   private final BehaviorTreeMessage behaviorTreeMessage = new BehaviorTreeMessage();
   private StatusLogger statusLogger;
   private BehaviorInterface highestLevelNode;
   private PausablePeriodicThread behaviorTreeTickThread;
   private BehaviorHelper helper;

   public static BehaviorModule createInterprocess(BehaviorRegistry behaviorRegistry, DRCRobotModel robotModel)
   {
      return new BehaviorModule(behaviorRegistry, robotModel, CommunicationMode.INTERPROCESS);
   }

   public static BehaviorModule createIntraprocess(BehaviorRegistry behaviorRegistry, DRCRobotModel robotModel)
   {
      return new BehaviorModule(behaviorRegistry, robotModel, CommunicationMode.INTRAPROCESS);
   }

   public BehaviorModule(BehaviorRegistry behaviorRegistry,
                         DRCRobotModel robotModel,
                         CommunicationMode ros2CommunicationMode)
   {
      this.behaviorRegistry = behaviorRegistry;
      this.robotModel = robotModel;

      rootNode = new BehaviorTreeControlFlowNode()
      {
         @Override
         public BehaviorTreeNodeStatus tickInternal()
         {
            return highestLevelNode.tick();
         }
      };
      rootNode.setName("Behavior Module");

      highestLevelBehaviorDefinition = behaviorRegistry.getHighestLevelNode();

      LogTools.info("Starting behavior module in ROS 2: {} mode", ros2CommunicationMode.name());
      ros2Node = ROS2Tools.createROS2Node(ros2CommunicationMode.getPubSubImplementation(), "behavior_module");

      setupHighLevelBehavior(highestLevelBehaviorDefinition);
   }

   private void setupHighLevelBehavior(BehaviorDefinition highestLevelNodeDefinition)
   {
      if (highestLevelNode != null)
      {
         destroyHighLevelNode();
      }

      helper = new BehaviorHelper(highestLevelNodeDefinition.getName(), robotModel, ros2Node);
      highestLevelNode = highestLevelNodeDefinition.getBehaviorSupplier().build(helper);
      statusLogger = helper.getOrCreateStatusLogger();
      rootNode.addChild(highestLevelNode);

      behaviorTreeTickThread = new PausablePeriodicThread("BehaviorTree", UnitConversions.hertzToSeconds(5.0), () ->
      {
         rootNode.tick();
         behaviorTreeMessage.getNodes().clear();
         BehaviorMessageTools.packBehaviorTreeMessage(rootNode, behaviorTreeMessage);
         helper.publish(API.BEHAVIOR_TREE_STATUS, behaviorTreeMessage);
      });
      behaviorTreeTickThread.start();

      new IHMCROS2Callback<>(ros2Node, API.SHUTDOWN, message ->
      {
         statusLogger.info("Received SHUTDOWN. Shutting down...");
         ThreadTools.startAsDaemon(this::destroy, "DestroyThread");
      });

      new IHMCROS2Callback<>(ros2Node, API.SHUTDOWN, message ->
      {
         statusLogger.info("Received SHUTDOWN. Shutting down...");
         ThreadTools.startAsDaemon(this::destroy, "DestroyThread");
      });

      new IHMCROS2Callback<>(ros2Node, API.SET_HIGHEST_LEVEL_BEHAVIOR, highestLevelBehaviorName ->
      {
         BehaviorDefinition candidateHighestLevelNodeDefinition = null;
         for (BehaviorDefinition definitionEntry : behaviorRegistry.getDefinitionEntries())
         {
            if (definitionEntry.getName().equals(highestLevelBehaviorName.getDataAsString()))
            {
               candidateHighestLevelNodeDefinition = definitionEntry;
            }
         }
         if (candidateHighestLevelNodeDefinition == null)
         {
            statusLogger.info("No behavior definition matches {}. Doing nothing.", highestLevelBehaviorName.getDataAsString());
         }
         else if (highestLevelBehaviorDefinition == candidateHighestLevelNodeDefinition)
         {
            statusLogger.info("Highest level behavior is already set to {}. Doing nothing.", highestLevelBehaviorName.getDataAsString());
         }
         else
         {
            statusLogger.info("Switching highest level behavior to {}...", highestLevelBehaviorName.getDataAsString());
            setupHighLevelBehavior(candidateHighestLevelNodeDefinition);
         }
      });
   }

   private void destroyHighLevelNode()
   {
      statusLogger.info("Shutting down...");
      behaviorTreeTickThread.destroy();
      helper.destroy();
      highestLevelNode.destroy();
   }

   public void destroy()
   {
      destroyHighLevelNode();

      ros2Node.destroy();
   }

   // API created here from build
   public static class API
   {
      public static final ROS2Topic<?> BASE_TOPIC = ROS2Tools.BEHAVIOR_MODULE;
      public static final ROS2Topic<Empty> SHUTDOWN = BASE_TOPIC.withInput().withType(Empty.class).withSuffix("shutdown");
      public static final ROS2Topic<std_msgs.msg.dds.String> SET_HIGHEST_LEVEL_BEHAVIOR
                                            = BASE_TOPIC.withType(std_msgs.msg.dds.String.class).withSuffix("set_highest_level_behavior");
      public static final ROS2Topic<StatusLogMessage> STATUS_LOG
                                            = BASE_TOPIC.withOutput().withType(StatusLogMessage.class).withSuffix("status_log");
      public static final ROS2Topic<BehaviorTreeMessage> BEHAVIOR_TREE_STATUS
                                            = BASE_TOPIC.withOutput().withType(BehaviorTreeMessage.class).withSuffix("behavior_tree_status");
   }
}
