package us.ihmc.humanoidBehaviors;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.messager.kryo.KryoMessager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;

public class BehaviorBackpack
{
   private final Messager messager;

   public static BehaviorBackpack createForBackpack(DRCRobotModel robotModel)
   {
      KryoMessager messager = KryoMessager.createServer(getBehaviorAPI(),
                                                        NetworkPorts.BEHAVIOUR_MODULE_PORT.getPort(), BehaviorBackpack.class.getSimpleName(), 5);
      ExceptionTools.handle(() -> messager.startMessager(), DefaultExceptionHandler.RUNTIME_EXCEPTION);
      return new BehaviorBackpack(robotModel, messager);
   }

   public static BehaviorBackpack createForTest(DRCRobotModel robotModel, Messager messager)
   {
      return new BehaviorBackpack(robotModel, messager);
   }

   private BehaviorBackpack(DRCRobotModel robotModel, Messager messager)
   {
      this.messager = messager;

      LogTools.info("Starting behavior backpack");
      
      PubSubImplementation pubSubImplementation = messager instanceof SharedMemoryMessager ? PubSubImplementation.INTRAPROCESS : PubSubImplementation.FAST_RTPS;
      Ros2Node ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, "behavior_backpack");

      new StepInPlaceBehavior(messager, ros2Node, robotModel);
   }

   public static MessagerAPI getBehaviorAPI()
   {
      return StepInPlaceBehavior.API.create();
   }
}
