package us.ihmc.humanoidBehaviors;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidBehaviors.StepInPlaceBehavior.API;
import us.ihmc.humanoidBehaviors.tools.FXUIMessagerAPIFactory;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.messager.kryo.KryoMessager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;

public class BehaviorModule
{
   private final Messager messager;

   public static BehaviorModule createForBackpack(DRCRobotModel robotModel)
   {
      KryoMessager messager = KryoMessager.createServer(FXUIMessagerAPIFactory.createAPIFor(BehaviorModule.class, StepInPlaceBehavior.API.create()),
                                                      NetworkPorts.BEHAVIOUR_MODULE_PORT.getPort(), BehaviorModule.class.getSimpleName(), 5);
      ExceptionTools.handle(() -> messager.startMessager(), DefaultExceptionHandler.RUNTIME_EXCEPTION);
      return new BehaviorModule(robotModel, messager);
   }

   public static BehaviorModule createForTest(DRCRobotModel robotModel, Messager messager)
   {
      return new BehaviorModule(robotModel, messager);
   }

   private BehaviorModule(DRCRobotModel robotModel, Messager messager)
   {
      this.messager = messager;

      PubSubImplementation pubSubImplementation = messager instanceof SharedMemoryMessager ? PubSubImplementation.INTRAPROCESS : PubSubImplementation.FAST_RTPS;
      Ros2Node ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, "humanoid_behavior_module");

      new StepInPlaceBehavior(messager, ros2Node, robotModel);
   }

   public static class API
   {
      private static final FXUIMessagerAPIFactory apiFactory = new FXUIMessagerAPIFactory(BehaviorModule.class);

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }

   public static MessagerAPI getBehaviorAPI()
   {
      return FXUIMessagerAPIFactory.createAPIFor(BehaviorModule.class, StepInPlaceBehavior.API.create());
   }
}
