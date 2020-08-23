package us.ihmc.humanoidBehaviors;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.MutablePair;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.BehaviorMessagerUpdateThread;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.messager.kryo.KryoMessager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.lists.PairList;

import static us.ihmc.humanoidBehaviors.BehaviorModule.API.BehaviorSelection;

public class BehaviorModule
{
   private final MessagerAPI messagerAPI;
   private final Messager messager;
   private final PairList<BehaviorDefinition, BehaviorInterface> constructedBehaviors = new PairList<>();
   private final Ros2Node ros2Node;

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
      LogTools.info("Starting behavior module in ROS 2: {}, Messager: {} modes", ros2CommunicationMode.name(), messagerCommunicationMode.name());

      messagerAPI = behaviorRegistry.getMessagerAPI();

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
                                              NetworkPorts.BEHAVIOUR_MODULE_PORT.getPort(),
                                              new BehaviorMessagerUpdateThread(BehaviorModule.class.getSimpleName(), 5));
      }
      else // intraprocess
      {
         messager = new SharedMemoryMessager(messagerAPI);
      }

      ThreadTools.startAThread(this::kryoStarter, "KryoStarter");

      ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, "behavior_backpack");

      for (BehaviorDefinition behaviorDefinition : behaviorRegistry.getDefinitionEntries())
      {
         BehaviorHelper helper = new BehaviorHelper(robotModel, messager, ros2Node);
         BehaviorInterface constructedBehavior = behaviorDefinition.getBehaviorSupplier().build(helper);
         constructedBehaviors.add(behaviorDefinition, constructedBehavior);
      }

      messager.registerTopicListener(BehaviorSelection, selection -> // simple string based selection
      {
         for (ImmutablePair<BehaviorDefinition, BehaviorInterface> behavior : constructedBehaviors)
         {
            behavior.getRight().setEnabled(behavior.getLeft().getName().equals(selection));
         }
      });
   }

   private void kryoStarter()
   {
      ExceptionTools.handle(messager::startMessager, DefaultExceptionHandler.RUNTIME_EXCEPTION);
   }

   public Messager getMessager()
   {
      return messager;
   }

   public void destroy()
   {
      LogTools.info("Shutting down...");
      for (ImmutablePair<BehaviorDefinition, BehaviorInterface> behavior : constructedBehaviors)
      {
         behavior.getRight().setEnabled(false);
      }

      ExceptionTools.handle(() -> getMessager().closeMessager(), DefaultExceptionHandler.PRINT_STACKTRACE);
      ros2Node.destroy();
   }

   // API created here from build
   public static class API
   {
      private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      private static final MessagerAPIFactory.Category RootCategory = apiFactory.createRootCategory("Root");
      private static final MessagerAPIFactory.CategoryTheme BehaviorModuleTheme = apiFactory.createCategoryTheme("BehaviorModule");

      public static final MessagerAPIFactory.Topic<String> BehaviorSelection = topic("BehaviorSelection");
      public static final MessagerAPIFactory.Topic<MutablePair<Integer, String>> StatusLog = topic("StatusLog");

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
