package us.ihmc.behaviors;

import org.apache.commons.lang3.mutable.MutableLong;
import org.apache.commons.lang3.tuple.MutablePair;

import std_msgs.msg.dds.Empty;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.tools.behaviorTree.*;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.BehaviorMessagerUpdateThread;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.messager.kryo.KryoMessager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
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

   private final ROS2Node ros2Node;
   private final boolean manageROS2Node;
   private final Messager messager;
   private final boolean manageMessager;
   public static final double YO_VARIABLE_SERVER_UPDATE_PERIOD = UnitConversions.hertzToSeconds(100.0);
   private final YoRegistry yoRegistry = new YoRegistry(getClass().getSimpleName());
   private YoVariableServer yoVariableServer;
   private final YoDouble yoTime = new YoDouble("time", yoRegistry);
   private final YoBoolean enabled = new YoBoolean("enabled", yoRegistry);
   private PausablePeriodicThread yoServerUpdateThread;
   private StatusLogger statusLogger;
   private final FallbackNode rootNode = new FallbackNode();
   private BehaviorInterface highestLevelNode;
   private PausablePeriodicThread behaviorTreeTickThread;

   public static BehaviorModule createInterprocess(BehaviorRegistry behaviorRegistry, DRCRobotModel robotModel)
   {
      return new BehaviorModule(behaviorRegistry, robotModel, CommunicationMode.INTERPROCESS, CommunicationMode.INTERPROCESS);
   }

   public static BehaviorModule createIntraprocess(BehaviorRegistry behaviorRegistry, DRCRobotModel robotModel)
   {
      return new BehaviorModule(behaviorRegistry, robotModel, CommunicationMode.INTRAPROCESS, CommunicationMode.INTRAPROCESS);
   }

   public BehaviorModule(BehaviorRegistry behaviorRegistry, 
                         DRCRobotModel robotModel,
                         CommunicationMode ros2CommunicationMode,
                         CommunicationMode messagerCommunicationMode)
   {
      this.manageROS2Node = true;
      this.manageMessager = true;

      LogTools.info("Starting behavior module in ROS 2: {}, Messager: {} modes", ros2CommunicationMode.name(), messagerCommunicationMode.name());

      MessagerAPI messagerAPI = behaviorRegistry.getMessagerAPI();

      PubSubImplementation pubSubImplementation;
      if (ros2CommunicationMode == CommunicationMode.INTERPROCESS)
      {
         pubSubImplementation = PubSubImplementation.FAST_RTPS;
      }
      else // intraprocess
      {
         pubSubImplementation = PubSubImplementation.INTRAPROCESS;
      }
      if (messagerCommunicationMode == CommunicationMode.INTERPROCESS)
      {
         messager = KryoMessager.createServer(messagerAPI,
                                              BEHAVIOR_MODULE_MESSAGER_PORT.getPort(),
                                              new BehaviorMessagerUpdateThread(BehaviorModule.class.getSimpleName(), 5));
      }
      else // intraprocess
      {
         messager = new SharedMemoryMessager(messagerAPI);
      }
      ThreadTools.startAThread(() -> ExceptionTools.handle(messager::startMessager, DefaultExceptionHandler.RUNTIME_EXCEPTION), "KryoStarter");

      ros2Node = ROS2Tools.createROS2Node(pubSubImplementation, "behavior_backpack");

      init(behaviorRegistry, robotModel, ros2Node, messager);
   }

   public BehaviorModule(BehaviorRegistry behaviorRegistry,
                         DRCRobotModel robotModel,
                         ROS2Node ros2Node,
                         Messager messager)
   {
      this.ros2Node = ros2Node;
      this.manageROS2Node = false;
      this.messager = messager;
      this.manageMessager = false;

      init(behaviorRegistry, robotModel, ros2Node, messager);
   }

   private void init(BehaviorRegistry behaviorRegistry, DRCRobotModel robotModel, ROS2Node ros2Node, Messager messager)
   {
      if (messager instanceof SharedMemoryMessager)
         sharedMemoryMessager = (SharedMemoryMessager) messager;

      statusLogger = new StatusLogger(messager::submitMessage);

      rootNode.setName("Behavior Module");
      BehaviorTreeCondition disabledNode = new BehaviorTreeCondition(() -> !enabled.getValue());
      disabledNode.setName("Disabled");
      rootNode.addChild(disabledNode);

      BehaviorDefinition highestLevelNodeDefinition = behaviorRegistry.getHighestLevelNode();
      BehaviorHelper helper = new BehaviorHelper(highestLevelNodeDefinition.getName(), robotModel, ros2Node);
      highestLevelNode = highestLevelNodeDefinition.getBehaviorSupplier().build(helper);
      if (highestLevelNode.getYoRegistry() != null)
      {
         yoRegistry.addChild(highestLevelNode.getYoRegistry());
      }
      helper.getMessagerHelper().setExternallyStartedMessager(messager);
      highestLevelNode.setEnabled(true);
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

      DataServerSettings dataServerSettings = new DataServerSettings(false, true, BEHAVIOR_MODULE_YOVARIABLESERVER_PORT.getPort(), null);
      yoVariableServer = new YoVariableServer(getClass().getSimpleName(), null, dataServerSettings, 0.01);
      yoVariableServer.setMainRegistry(yoRegistry, null);
      LogTools.info("Starting YoVariableServer...");
      yoVariableServer.start();
      LogTools.info("Starting YoVariableServer update thread...");
      MutableLong timestamp = new MutableLong();
      LocalDateTime startTime = LocalDateTime.now();
      yoServerUpdateThread = new PausablePeriodicThread("YoServerUpdate", YO_VARIABLE_SERVER_UPDATE_PERIOD, () ->
      {
         yoTime.set(Conversions.nanosecondsToSeconds(-LocalDateTime.now().until(startTime, ChronoUnit.NANOS)));
         yoVariableServer.update(timestamp.getAndAdd(Conversions.secondsToNanoseconds(YO_VARIABLE_SERVER_UPDATE_PERIOD)));
      });
      yoServerUpdateThread.start();
   }

   public Messager getMessager()
   {
      return messager;
   }

   public void destroy()
   {
      statusLogger.info("Shutting down...");
      behaviorTreeTickThread.destroy();
      yoVariableServer.close();
      if (manageROS2Node)
      {
         ros2Node.destroy();
      }
      if (manageMessager)
      {
         ExceptionTools.handle(() -> getMessager().closeMessager(), DefaultExceptionHandler.PRINT_STACKTRACE);
      }

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
