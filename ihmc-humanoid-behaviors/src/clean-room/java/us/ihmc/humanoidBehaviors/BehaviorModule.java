package us.ihmc.humanoidBehaviors;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.kryo.KryoMessager;
import us.ihmc.messager.kryo.MessagerUpdateThread;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;

import java.util.concurrent.TimeUnit;

public class BehaviorModule
{
   private static int UPDATE_PERIOD_MILLIS = 5;
   private final PeriodicNonRealtimeThreadScheduler thread;
   private final KryoMessager messager;

   public static BehaviorModule createForTest(DRCRobotModel robotModel)
   {
      return new BehaviorModule(robotModel, PubSubImplementation.INTRAPROCESS);
   }

   public static BehaviorModule createForTeleop(DRCRobotModel robotModel)
   {
      return new BehaviorModule(robotModel, PubSubImplementation.FAST_RTPS);
   }

   private BehaviorModule(DRCRobotModel robotModel, PubSubImplementation pubSubImplementation)
   {
      thread = new PeriodicNonRealtimeThreadScheduler(getClass().getSimpleName());
      messager = KryoMessager.createServer(API.create(), NetworkPorts.BEHAVIOUR_MODULE_PORT.getPort(), new UpdateThread());

      MessageTopicNameGenerator controllerTopicNameGenerator = ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotModel.getSimpleRobotName());
      Ros2Node ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, "humanoid_behavior_module");

      new StepInPlaceBehavior(messager, ros2Node, controllerTopicNameGenerator, robotModel);
   }

   private class UpdateThread implements MessagerUpdateThread
   {
      @Override
      public void start(Runnable runnable)
      {
         UPDATE_PERIOD_MILLIS = 1;
         thread.schedule(runnable, UPDATE_PERIOD_MILLIS, TimeUnit.MILLISECONDS);
      }

      @Override
      public void stop()
      {
         thread.shutdown();
      }
   }

   public static class API
   {
      private static final FXUIMessagerAPIFactory apiFactory = new FXUIMessagerAPIFactory(BehaviorModule.class);

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
