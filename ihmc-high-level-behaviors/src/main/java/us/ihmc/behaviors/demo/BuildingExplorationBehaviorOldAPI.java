package us.ihmc.behaviors.demo;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.messager.MessagerAPIFactory;

public class BuildingExplorationBehaviorOldAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final MessagerAPIFactory.Category Root = apiFactory.createRootCategory("DemoCoordinatorRoot");
   private static final MessagerAPIFactory.CategoryTheme DemoCoordinator = apiFactory.createCategoryTheme("DemoCoordinator");

   public static final MessagerAPIFactory.Topic<RobotConfigurationData> RobotConfigurationData = topic("RobotConfigurationData");

   public static final MessagerAPIFactory.Topic<BuildingExplorationStateName> RequestedState = topic("RequestedState");
   public static final MessagerAPIFactory.Topic<BuildingExplorationStateName> CurrentState = topic("CurrentState");

   public static final MessagerAPIFactory.Topic<Boolean> DebrisDetected = topic("DebrisDetected");
   public static final MessagerAPIFactory.Topic<Boolean> StairsDetected = topic("StairsDetected");
   public static final MessagerAPIFactory.Topic<Boolean> DoorDetected = topic("DoorDetected");

   public static final MessagerAPIFactory.Topic<Boolean> IgnoreDebris = topic("IgnoreDebris");
   public static final MessagerAPIFactory.Topic<Object> ConfirmDoor = topic("ConfirmDoor");

   public static final MessagerAPIFactory.Topic<Object> Start = topic("Start");
   public static final MessagerAPIFactory.Topic<Object> Stop = topic("Stop");
   public static final MessagerAPIFactory.Topic<Pose3D> Goal = topic("Goal");
   public static final MessagerAPIFactory.Topic<Pose3D> GoalForUI = topic("GoalForUI");

   public static final MessagerAPIFactory.MessagerAPI API = apiFactory.getAPIAndCloseFactory();

   private static <T> MessagerAPIFactory.Topic<T> topic(String name)
   {
      return Root.child(DemoCoordinator).topic(apiFactory.createTypedTopicTheme(name));
   }
}
