package us.ihmc.humanoidBehaviors;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehavior;
import us.ihmc.humanoidBehaviors.fancyPoses.FancyPosesBehavior;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehavior;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehaviorAPI;
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

public class BehaviorModule
{
   private final Messager messager;

   public static BehaviorModule createForBackpack(DRCRobotModel robotModel)
   {
      KryoMessager messager = KryoMessager.createServer(getBehaviorAPI(),
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
      this.messager = messager;

      LogTools.info("Starting behavior backpack");

      PubSubImplementation pubSubImplementation = messager instanceof SharedMemoryMessager ? PubSubImplementation.INTRAPROCESS : PubSubImplementation.FAST_RTPS;
      Ros2Node ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, "behavior_backpack");

      BehaviorHelper behaviorHelper = new BehaviorHelper(messager, robotModel, ros2Node);

      new StepInPlaceBehavior(behaviorHelper, messager, robotModel);  // TODO don't start threads on construction, but right now not hurting anything
      new PatrolBehavior(behaviorHelper, messager, robotModel);
      new FancyPosesBehavior(behaviorHelper, messager, robotModel);
      new ExploreAreaBehavior(behaviorHelper, messager, robotModel);
   }

   public static MessagerAPI getBehaviorAPI()
   {
      MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      apiFactory.createRootCategory("Root");

      apiFactory.includeMessagerAPIs(StepInPlaceBehavior.API.create());
      apiFactory.includeMessagerAPIs(PatrolBehaviorAPI.create());
      apiFactory.includeMessagerAPIs(FancyPosesBehavior.API.create());
      apiFactory.includeMessagerAPIs(ExploreAreaBehavior.ExploreAreaBehaviorAPI.create());

      return apiFactory.getAPIAndCloseFactory();
   }
}
