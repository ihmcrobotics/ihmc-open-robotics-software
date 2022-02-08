package us.ihmc.behaviors;

import org.apache.commons.lang3.mutable.MutableLong;
import org.apache.commons.lang3.tuple.MutablePair;

import std_msgs.msg.dds.Empty;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.tools.behaviorTree.*;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.SharedMemoryMessager;
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

import static us.ihmc.communication.util.NetworkPorts.BEHAVIOR_MODULE_MESSAGER_PORT;
import static us.ihmc.communication.util.NetworkPorts.BEHAVIOR_MODULE_YOVARIABLESERVER_PORT;

public class BehaviorModule
{
   private static SharedMemoryMessager sharedMemoryMessager;

   private ROS2Node ros2Node;
   public static final double YO_VARIABLE_SERVER_UPDATE_PERIOD = UnitConversions.hertzToSeconds(100.0);
   private final YoRegistry yoRegistry = new YoRegistry(getClass().getSimpleName());
   private YoVariableServer yoVariableServer;
   private final YoDouble yoTime = new YoDouble("time", yoRegistry);
   private final YoBoolean enabled = new YoBoolean("enabled", yoRegistry);
   private PausablePeriodicThread yoServerUpdateThread;
   private StatusLogger statusLogger;
   private BehaviorTreeControlFlowNode rootNode;
   private BehaviorInterface highestLevelNode;
   private PausablePeriodicThread behaviorTreeTickThread;
   private BehaviorRegistry behaviorRegistry;
   private DRCRobotModel robotModel;
   private final CommunicationMode ros2CommunicationMode;
   private final CommunicationMode messagerCommunicationMode;
   private final boolean enableROS1;
   private BehaviorDefinition highestLevelBehaviorDefinition;
   private BehaviorHelper helper;

   public static BehaviorModule createInterprocess(BehaviorRegistry behaviorRegistry, DRCRobotModel robotModel)
   {
      return new BehaviorModule(behaviorRegistry, robotModel, CommunicationMode.INTERPROCESS, CommunicationMode.INTERPROCESS, true);
   }

   public static BehaviorModule createIntraprocess(BehaviorRegistry behaviorRegistry, DRCRobotModel robotModel)
   {
      return new BehaviorModule(behaviorRegistry, robotModel, CommunicationMode.INTRAPROCESS, CommunicationMode.INTRAPROCESS, false);
   }

   public BehaviorModule(BehaviorRegistry behaviorRegistry, 
                         DRCRobotModel robotModel,
                         CommunicationMode ros2CommunicationMode,
                         CommunicationMode messagerCommunicationMode,
                         boolean enableROS1)
   {
      this.behaviorRegistry = behaviorRegistry;
      this.robotModel = robotModel;
      this.ros2CommunicationMode = ros2CommunicationMode;
      this.messagerCommunicationMode = messagerCommunicationMode;
      this.enableROS1 = enableROS1;

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
      setupHighLevelBehavior(highestLevelBehaviorDefinition);
   }

   private void setupHighLevelBehavior(BehaviorDefinition highestLevelNodeDefinition)
   {
      if (highestLevelNode != null)
      {
         helper.destroy();
         ros2Node.destroy();
         behaviorTreeTickThread.destroy();
         yoVariableServer.close();
         highestLevelNode.destroy();
      }

      LogTools.info("Starting behavior module in ROS 2: {}, Messager: {} modes", ros2CommunicationMode.name(), messagerCommunicationMode.name());
      ros2Node = ROS2Tools.createROS2Node(ros2CommunicationMode.getPubSubImplementation(), "behavior_module");
      helper = new BehaviorHelper(highestLevelNodeDefinition.getName(), robotModel, ros2Node, enableROS1);
      highestLevelNode = highestLevelNodeDefinition.getBehaviorSupplier().build(helper);
      if (highestLevelNode.getYoRegistry() != null)
      {
         yoRegistry.addChild(highestLevelNode.getYoRegistry());
      }
      if (messagerCommunicationMode == CommunicationMode.INTERPROCESS)
      {
         helper.getMessagerHelper().startServer(BEHAVIOR_MODULE_MESSAGER_PORT.getPort());
      }
      else
      {
         sharedMemoryMessager = new SharedMemoryMessager(behaviorRegistry.getMessagerAPI());
         helper.getMessagerHelper().connectViaSharedMemory(sharedMemoryMessager);
         sharedMemoryMessager.startMessager();
      }
      statusLogger = new StatusLogger(helper.getMessagerHelper().getMessager()::submitMessage);
      rootNode.addChild(highestLevelNode);

      behaviorTreeTickThread = new PausablePeriodicThread("BehaviorTree", UnitConversions.hertzToSeconds(5.0), () ->
      {
         rootNode.tick();
         helper.publish(API.BehaviorTreeStatus, new BehaviorTreeStatus(rootNode));
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

   public Messager getMessager()
   {
      return helper.getMessager();
   }

   public void destroy()
   {
      statusLogger.info("Shutting down...");
      behaviorTreeTickThread.destroy();
      yoVariableServer.close();
      sharedMemoryMessager = null;
      helper.destroy();
      ros2Node.destroy();
      highestLevelNode.destroy();
   }

   public static SharedMemoryMessager getSharedMemoryMessager()
   {
      return sharedMemoryMessager;
   }

   // API created here from build
   public static class API
   {
      public static final ROS2Topic<Empty> SHUTDOWN = ROS2Tools.BEHAVIOR_MODULE.withOutput().withType(Empty.class).withSuffix("shutdown");
      public static final ROS2Topic<std_msgs.msg.dds.String> SET_HIGHEST_LEVEL_BEHAVIOR
            = ROS2Tools.BEHAVIOR_MODULE.withType(std_msgs.msg.dds.String.class).withSuffix("set_highest_level_behavior");

      private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      private static final MessagerAPIFactory.Category RootCategory = apiFactory.createRootCategory("Root");
      private static final MessagerAPIFactory.CategoryTheme BehaviorModuleTheme = apiFactory.createCategoryTheme("BehaviorModule");

      public static final MessagerAPIFactory.Topic<String> BehaviorSelection = topic("BehaviorSelection");
      public static final MessagerAPIFactory.Topic<MutablePair<Integer, String>> StatusLog = topic("StatusLog");
      public static final MessagerAPIFactory.Topic<BehaviorTreeControlFlowNode> BehaviorTreeStatus = topic("BehaviorTreeStatus");

      private static <T> MessagerAPIFactory.Topic<T> topic(String name)
      {
         return RootCategory.child(BehaviorModuleTheme).topic(apiFactory.createTypedTopicTheme(name));
      }

      public static synchronized MessagerAPI create(MessagerAPI... behaviorAPIs) // TODO check threading
      {
         apiFactory.includeMessagerAPIs(behaviorAPIs);

         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
