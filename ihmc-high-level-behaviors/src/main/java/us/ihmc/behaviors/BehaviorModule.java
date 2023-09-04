package us.ihmc.behaviors;

import behavior_msgs.msg.dds.BehaviorTreeMessage;
import behavior_msgs.msg.dds.StatusLogMessage;
import org.apache.commons.lang3.mutable.MutableLong;

import std_msgs.msg.dds.Empty;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.tools.BehaviorMessageTools;
import us.ihmc.behaviors.tools.behaviorTree.*;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.time.LocalDateTime;
import java.time.temporal.ChronoUnit;

import static us.ihmc.communication.util.NetworkPorts.BEHAVIOR_MODULE_YOVARIABLESERVER_PORT;

/**
 * Manages a behavior tree based process on the real robot and syncs it
 * with an operator UI with shared autonomy.
 */
public class BehaviorModule
{
   private ROS2Node ros2Node;
   public static final double YO_VARIABLE_SERVER_UPDATE_PERIOD = UnitConversions.hertzToSeconds(100.0);
   private final YoRegistry yoRegistry = new YoRegistry(getClass().getSimpleName());
   private final YoDouble yoTime = new YoDouble("time", yoRegistry);
   private final YoBoolean enabled = new YoBoolean("enabled", yoRegistry);
   private final BehaviorTreeControlFlowNode rootNode;
   private final BehaviorRegistry behaviorRegistry;
   private final DRCRobotModel robotModel;
   private final BehaviorDefinition highestLevelBehaviorDefinition;
   private final BehaviorTreeMessage behaviorTreeMessage = new BehaviorTreeMessage();
   private final ROS2Heartbeat heartbeat;
   private YoVariableServer yoVariableServer;
   private PausablePeriodicThread yoServerUpdateThread;
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
            if (enabled.getValue())
               return highestLevelNode.tick();
            else
               return BehaviorTreeNodeStatus.FAILURE;
         }
      };
      rootNode.setName("Behavior Module");

      highestLevelBehaviorDefinition = behaviorRegistry.getHighestLevelNode();

      LogTools.info("Starting behavior module in ROS 2: {} mode", ros2CommunicationMode.name());
      ros2Node = ROS2Tools.createROS2Node(ros2CommunicationMode.getPubSubImplementation(), "behavior_module");
      heartbeat = new ROS2Heartbeat(ros2Node, API.HEARTBEAT);
      heartbeat.setAlive(true);

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
      if (highestLevelNode.getYoRegistry() != null)
      {
         yoRegistry.addChild(highestLevelNode.getYoRegistry());
      }
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

      int port = BEHAVIOR_MODULE_YOVARIABLESERVER_PORT.getPort();
      DataServerSettings dataServerSettings = new DataServerSettings(false, true, port, null);
      yoVariableServer = new YoVariableServer(getClass().getSimpleName(), null, dataServerSettings, 0.01);
      yoVariableServer.setMainRegistry(yoRegistry, null);
      LogTools.info("Starting YoVariableServer on {}...", port);
      yoVariableServer.start();
      LogTools.info("Starting YoVariableServer update thread for {}...", port);
      MutableLong timestamp = new MutableLong();
      LocalDateTime startTime = LocalDateTime.now();
      yoServerUpdateThread = new PausablePeriodicThread("YoServerUpdate", YO_VARIABLE_SERVER_UPDATE_PERIOD, () ->
      {
         yoTime.set(Conversions.nanosecondsToSeconds(-LocalDateTime.now().until(startTime, ChronoUnit.NANOS)));
         yoVariableServer.update(timestamp.getAndAdd(Conversions.secondsToNanoseconds(YO_VARIABLE_SERVER_UPDATE_PERIOD)));
      });
      yoServerUpdateThread.start();

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
      yoServerUpdateThread.destroy();
      behaviorTreeTickThread.destroy();
      helper.destroy();
      yoVariableServer.close();
      highestLevelNode.destroy();
   }

   public void destroy()
   {
      destroyHighLevelNode();

      heartbeat.destroy();
      ros2Node.destroy();
   }

   // API created here from build
   public static class API
   {
      public static final ROS2Topic<?> BASE_TOPIC = ROS2Tools.BEHAVIOR_MODULE;
      public static final ROS2Topic<Empty> HEARTBEAT = BASE_TOPIC.withOutput().withType(Empty.class).withSuffix("heartbeat");
      public static final ROS2Topic<Empty> SHUTDOWN = BASE_TOPIC.withInput().withType(Empty.class).withSuffix("shutdown");
      public static final ROS2Topic<std_msgs.msg.dds.String> SET_HIGHEST_LEVEL_BEHAVIOR
                                            = BASE_TOPIC.withType(std_msgs.msg.dds.String.class).withSuffix("set_highest_level_behavior");
      public static final ROS2Topic<StatusLogMessage> STATUS_LOG
                                            = BASE_TOPIC.withOutput().withType(StatusLogMessage.class).withSuffix("status_log");
      public static final ROS2Topic<BehaviorTreeMessage> BEHAVIOR_TREE_STATUS
                                            = BASE_TOPIC.withOutput().withType(BehaviorTreeMessage.class).withSuffix("behavior_tree_status");
   }
}
