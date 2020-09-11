package us.ihmc.humanoidBehaviors.ui.behaviors.coordinator;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.demo.BuildingExplorationStateName;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class BuildingExplorationBehaviorAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final MessagerAPIFactory.Category Root = apiFactory.createRootCategory("DemoCoordinatorRoot");
   private static final MessagerAPIFactory.CategoryTheme DemoCoordinator = apiFactory.createCategoryTheme("DemoCoordinator");

   public static final MessagerAPIFactory.Topic<RobotConfigurationData> RobotConfigurationData = topic("RobotConfigurationData");
   public static final MessagerAPIFactory.Topic<PlanarRegionsList> PlanarRegions = topic("PlanarRegions");
   public static final MessagerAPIFactory.Topic<Boolean> ShowRegions = topic("ShowRegions");

   public static final MessagerAPIFactory.Topic<BuildingExplorationStateName> RequestedState = topic("RequestedState");
   public static final MessagerAPIFactory.Topic<BuildingExplorationStateName> CurrentState = topic("CurrentState");

   public static final MessagerAPIFactory.Topic<Boolean> DebrisDetected = topic("DebrisDetected");
   public static final MessagerAPIFactory.Topic<Boolean> StairsDetected = topic("StairsDetected");
   public static final MessagerAPIFactory.Topic<Boolean> DoorDetected = topic("DoorDetected");
   public static final MessagerAPIFactory.Topic<Boolean> IgnoreDebris = topic("IgnoreDebris");

   public static final MessagerAPIFactory.Topic<Boolean> Start = topic("Start");
   public static final MessagerAPIFactory.Topic<Boolean> Stop = topic("Stop");
   public static final MessagerAPIFactory.Topic<Point3D> Goal = topic("Goal");

   public static final MessagerAPIFactory.MessagerAPI API = apiFactory.getAPIAndCloseFactory();

   private static <T> MessagerAPIFactory.Topic<T> topic(String name)
   {
      return Root.child(DemoCoordinator).topic(apiFactory.createTypedTopicTheme(name));
   }
}
