package us.ihmc.humanoidBehaviors;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidBehaviors.tools.BehaviorMessagerUpdateThread;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.messager.kryo.KryoMessager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;

import static us.ihmc.humanoidBehaviors.BehaviorModule.API.BehaviorSelection;

public class BehaviorModule
{
   public static final MessagerAPI MessagerAPI = API.create(); // Assuming MessagerAPI is thread safe

   public static BehaviorModule createForBackpack(DRCRobotModel robotModel)
   {
      KryoMessager messager = KryoMessager.createServer(MessagerAPI,
                                                        NetworkPorts.BEHAVIOUR_MODULE_PORT.getPort(),
                                                        new BehaviorMessagerUpdateThread(BehaviorModule.class.getSimpleName(), 5));
      ExceptionTools.handle(() -> messager.startMessager(), DefaultExceptionHandler.RUNTIME_EXCEPTION);
      return new BehaviorModule(robotModel, messager);
   }

   public static BehaviorModule createForTest(DRCRobotModel robotModel, Messager messager)
   {
      return new BehaviorModule(robotModel, messager);
   }

   private BehaviorModule(DRCRobotModel robotModel, Messager messager)
   {
      LogTools.info("Starting behavior backpack");

      PubSubImplementation pubSubImplementation = messager instanceof SharedMemoryMessager ? PubSubImplementation.INTRAPROCESS : PubSubImplementation.FAST_RTPS;
      Ros2Node ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, "behavior_backpack");

      for (BehaviorRegistry behavior : BehaviorRegistry.values())
      {
         behavior.build(robotModel, messager, ros2Node);
      }

      messager.registerTopicListener(BehaviorSelection, selection -> // simple string based selection
      {
         for (BehaviorRegistry behaviorEntry : BehaviorRegistry.values)
         {
            behaviorEntry.getConstructedBehavior().setEnabled(behaviorEntry.name().equals(selection));
         }
      });
   }

   public static class API
   {
      private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      private static final MessagerAPIFactory.Category RootCategory = apiFactory.createRootCategory("Root");
      private static final MessagerAPIFactory.CategoryTheme BehaviorModuleTheme = apiFactory.createCategoryTheme("BehaviorModule");

      public static final Topic<String> BehaviorSelection = topic("BehaviorSelection");

      private static final <T> Topic<T> topic(String name)
      {
         return RootCategory.child(BehaviorModuleTheme).topic(apiFactory.createTypedTopicTheme(name));
      }

      private static synchronized final MessagerAPI create() // TODO check threading
      {
         for (BehaviorRegistry behavior : BehaviorRegistry.values)
         {
            apiFactory.includeMessagerAPIs(behavior.getBehaviorAPI());
         }

         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
