package us.ihmc.atlas.behaviors.coordinator;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.behaviors.AtlasBehaviorModule;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.BehaviorRegistry;
import us.ihmc.humanoidBehaviors.ui.behaviors.coordinator.BuildingExplorationBehaviorAPI;
import us.ihmc.humanoidBehaviors.ui.behaviors.coordinator.BuildingExplorationBehaviorUI;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.tools.processManagement.JavaProcessSpawner;

public class AtlasBuildingExplorationBehaviorUI
{
   public static void start()
   {
      start(new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT, false),
            DomainFactory.PubSubImplementation.FAST_RTPS,
            BehaviorRegistry.DEFAULT_BEHAVIORS);
   }

   public static void start(DRCRobotModel robotModel, DomainFactory.PubSubImplementation pubSubImplementation, BehaviorRegistry behaviorRegistry)
   {
      JavaFXApplicationCreator.buildJavaFXApplication(stage ->
      {
         SharedMemoryJavaFXMessager messager = new SharedMemoryJavaFXMessager(BuildingExplorationBehaviorAPI.API);
         messager.startMessager();

         ExceptionTools.handle(() ->
         {
            BuildingExplorationBehaviorUI ui = new BuildingExplorationBehaviorUI(stage, messager, robotModel, pubSubImplementation, behaviorRegistry);
            ui.show();
         }, DefaultExceptionHandler.RUNTIME_EXCEPTION);
      });
   }

   public static void main(String[] args)
   {
      new JavaProcessSpawner(true).spawn(AtlasBehaviorModule.class);
//      new BehaviorModule(BehaviorRegistry.DEFAULT_BEHAVIORS, robotModel, CommunicationMode.INTRAPROCESS, CommunicationMode.INTERPROCESS);

      AtlasBuildingExplorationBehaviorUI.start();
   }
}
