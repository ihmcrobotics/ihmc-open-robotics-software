package us.ihmc.robotEnvironmentAwareness;

import java.nio.file.Path;
import java.nio.file.Paths;

import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.javafx.ApplicationNoModule;
import us.ihmc.messager.Messager;
import us.ihmc.messager.javafx.SharedMemoryJavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.communication.SegmentationModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.PlanarSegmentationUI;
import us.ihmc.robotEnvironmentAwareness.updaters.PlanarSegmentationModule;
import us.ihmc.tools.io.WorkspacePathTools;

public class PlanarSegmentationStandaloneLauncher extends ApplicationNoModule
{
   private static final String SEGMENTATION_CONFIGURATION_FILE_NAME = "atlasSLAMSegmentationModuleConfiguration.txt";

   private Messager messager;
   private PlanarSegmentationUI ui;
   private PlanarSegmentationModule module;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      messager = new SharedMemoryJavaFXMessager(SegmentationModuleAPI.API);
      messager.startMessager();

      Path segmentationConfigurationFilePath = WorkspacePathTools.handleWorkingDirectoryFuzziness("ihmc-open-robotics-software/atlas");
      segmentationConfigurationFilePath = Paths.get(segmentationConfigurationFilePath.toString(), "/src/main/resources/" + SEGMENTATION_CONFIGURATION_FILE_NAME);

      ui = PlanarSegmentationUI.createIntraprocessUI(messager, primaryStage);
      module = PlanarSegmentationModule.createIntraprocessModule(segmentationConfigurationFilePath.toFile(), messager);

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
