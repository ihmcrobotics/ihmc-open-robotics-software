package us.ihmc.atlas.behaviors.coordinator;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.BehaviorRegistry;
import us.ihmc.humanoidBehaviors.RemoteBehaviorInterface;
import us.ihmc.humanoidBehaviors.demo.BuildingExplorationBehaviorAPI;
import us.ihmc.humanoidBehaviors.ui.behaviors.coordinator.BuildingExplorationBehaviorUI;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.ROS2Node;

public class AtlasBuildingExplorationBehaviorUI
{
   public static void start()
   {
      start(new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT, false),
            CommunicationMode.INTERPROCESS,
            BehaviorRegistry.DEFAULT_BEHAVIORS,
            CommunicationMode.INTERPROCESS);
   }

   public static void start(DRCRobotModel robotModel,
                            CommunicationMode ros2CommunicationMode,
                            BehaviorRegistry behaviorRegistry,
                            CommunicationMode behaviorMessagerCommunicationMode)
   {
      BehaviorModule behaviorModule = new BehaviorModule(behaviorRegistry, robotModel, ros2CommunicationMode, behaviorMessagerCommunicationMode);
      ROS2Node ros2Node = ROS2Tools.createROS2Node(ros2CommunicationMode.getPubSubImplementation(), "building_exploration");
      Messager behaviorMessager = behaviorMessagerCommunicationMode == CommunicationMode.INTRAPROCESS
            ? behaviorModule.getMessager() : RemoteBehaviorInterface.createForUI(behaviorRegistry, "localhost");

      JavaFXApplicationCreator.buildJavaFXApplication(stage ->
      {
         SharedMemoryJavaFXMessager messager = new SharedMemoryJavaFXMessager(BuildingExplorationBehaviorAPI.API);
         messager.startMessager();

         ExceptionTools.handle(() ->
         {
            BuildingExplorationBehaviorUI ui = new BuildingExplorationBehaviorUI(stage, messager, robotModel, ros2Node, behaviorMessager);
            ui.show();
         }, DefaultExceptionHandler.RUNTIME_EXCEPTION);
      });
   }

   public static void main(String[] args)
   {
      AtlasBuildingExplorationBehaviorUI.start();
   }
}
