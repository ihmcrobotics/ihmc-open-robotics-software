package us.ihmc.robotEnvironmentAwareness;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.robotEnvironmentAwareness.ui.PlanarSegmentationUI;
import us.ihmc.robotEnvironmentAwareness.updaters.PlanarSegmentationModule;

public class PlanarSegmentationStandaloneLauncher extends Application
{
   private static final String MODULE_CONFIGURATION_FILE_NAME = "./Configurations/defaultREAModuleConfiguration.txt";

   private PlanarSegmentationUI ui;
   private PlanarSegmentationModule module;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      ui = PlanarSegmentationUI.creatIntraprocessUI(primaryStage);
      module = PlanarSegmentationModule.createIntraprocessModule(MODULE_CONFIGURATION_FILE_NAME);

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
