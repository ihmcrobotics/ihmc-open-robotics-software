package us.ihmc.robotEnvironmentAwareness;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.SegmentationModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.PlanarSegmentationUI;
import us.ihmc.robotEnvironmentAwareness.updaters.PlanarSegmentationModule;

public class PlanarSegmentationStandaloneLauncher extends Application
{
   private static final String MODULE_CONFIGURATION_FILE_NAME = "./Configurations/defaultREAModuleConfiguration.txt";

   private Messager messager;
   private PlanarSegmentationUI ui;
   private PlanarSegmentationModule module;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      messager = new SharedMemoryJavaFXMessager(SegmentationModuleAPI.API);
      messager.startMessager();

      ui = PlanarSegmentationUI.createIntraprocessUI(messager, primaryStage);
      module = PlanarSegmentationModule.createIntraprocessModule(MODULE_CONFIGURATION_FILE_NAME, messager);

      ui.show();
      module.start();
   }

   @Override
   public void stop() throws Exception
   {
      ui.stop();
      module.stop();

      messager.closeMessager();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
