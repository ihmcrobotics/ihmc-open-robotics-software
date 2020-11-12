package us.ihmc.atlas.behaviors.coordinator;

import javafx.application.Application;
import javafx.stage.Stage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.behaviors.AtlasBehaviorModule;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.BehaviorRegistry;
import us.ihmc.humanoidBehaviors.ui.behaviors.coordinator.BuildingExplorationBehaviorAPI;
import us.ihmc.humanoidBehaviors.ui.behaviors.coordinator.BuildingExplorationBehaviorUI;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.tools.processManagement.JavaProcessSpawner;

public class AtlasBuildingExplorationBehaviorUI extends Application
{
   @Override
   public void start(Stage stage) throws Exception
   {
      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT, false);
      SharedMemoryJavaFXMessager messager = new SharedMemoryJavaFXMessager(BuildingExplorationBehaviorAPI.API);

      messager.startMessager();

      new JavaProcessSpawner(true).spawn(AtlasBehaviorModule.class);
//      new BehaviorModule(BehaviorRegistry.DEFAULT_BEHAVIORS, robotModel, CommunicationMode.INTRAPROCESS, CommunicationMode.INTERPROCESS);

      BuildingExplorationBehaviorUI ui = new BuildingExplorationBehaviorUI(stage, messager, robotModel);

      ui.show();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
