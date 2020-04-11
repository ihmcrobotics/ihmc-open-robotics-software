package us.ihmc.robotEnvironmentAwareness;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMModule;
import us.ihmc.robotEnvironmentAwareness.ui.SLAMBasedEnvironmentAwarenessUI;

public class SLAMBasedREAStandaloneLauncher extends Application
{
   private static final String MODULE_CONFIGURATION_FILE_NAME = "./Configurations/defaultREAModuleConfiguration.txt";

   private SLAMBasedEnvironmentAwarenessUI ui;
   private SLAMModule module;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      ui = SLAMBasedEnvironmentAwarenessUI.creatIntraprocessUI(primaryStage);
      module = SLAMModule.createIntraprocessModule(MODULE_CONFIGURATION_FILE_NAME);

      ui.show();
      module.start();
   }

   @Override
   public void stop() throws Exception
   {
      ui.stop();
      module.stop();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
