package us.ihmc.robotEnvironmentAwareness;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.robotEnvironmentAwareness.ui.LIDARBasedEnvironmentAwarenessUI;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotEnvironmentAwareness.updaters.StereoBasedREAModule;

public class LidarBasedREAStandaloneLauncher extends Application
{
   private static final String MODULE_CONFIGURATION_FILE_NAME = "./Configurations/defaultREAModuleConfiguration.txt";

   private LIDARBasedEnvironmentAwarenessUI ui;
   private LIDARBasedREAModule lidarBasedModule;
   private StereoBasedREAModule stereoBasedModule;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      ui = LIDARBasedEnvironmentAwarenessUI.creatIntraprocessUI(primaryStage);
      lidarBasedModule = LIDARBasedREAModule.createIntraprocessModule(MODULE_CONFIGURATION_FILE_NAME);
      stereoBasedModule = StereoBasedREAModule.createIntraprocessModule(MODULE_CONFIGURATION_FILE_NAME);

      ui.show();
      lidarBasedModule.start();
      stereoBasedModule.start();
   }

   @Override
   public void stop() throws Exception
   {
      ui.stop();
      lidarBasedModule.stop();
      stereoBasedModule.stop();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
