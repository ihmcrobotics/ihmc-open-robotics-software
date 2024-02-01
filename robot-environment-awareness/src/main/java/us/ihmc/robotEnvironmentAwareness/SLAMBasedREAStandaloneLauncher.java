package us.ihmc.robotEnvironmentAwareness;

import java.nio.file.Path;
import java.nio.file.Paths;

import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.javafx.ApplicationNoModule;
import us.ihmc.messager.Messager;
import us.ihmc.messager.javafx.SharedMemoryJavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.SegmentationModuleAPI;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMModule;
import us.ihmc.robotEnvironmentAwareness.ui.PlanarSegmentationUI;
import us.ihmc.robotEnvironmentAwareness.ui.SLAMBasedEnvironmentAwarenessUI;
import us.ihmc.robotEnvironmentAwareness.updaters.PlanarSegmentationModule;
import us.ihmc.tools.io.WorkspacePathTools;

public class SLAMBasedREAStandaloneLauncher extends ApplicationNoModule
{
   private static final String SEGMENTATION_CONFIGURATION_FILE_NAME = "defaultSLAMSegmentationModuleConfiguration.txt";

   private Messager slamMessager;
   private SLAMModule slamModule;
   private SLAMBasedEnvironmentAwarenessUI slamUI;

   private Messager segmentationMessager;
   private PlanarSegmentationModule segmentationModule;
   private PlanarSegmentationUI planarSegmentationUI;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      slamMessager = new SharedMemoryJavaFXMessager(SLAMModuleAPI.API);
      slamUI = SLAMBasedEnvironmentAwarenessUI.creatIntraprocessUI(slamMessager, primaryStage);
      slamModule = SLAMModule.createIntraprocessModule(slamMessager);

      segmentationMessager = new SharedMemoryJavaFXMessager(SegmentationModuleAPI.API);
      segmentationMessager.startMessager();

      Path segmentationConfigurationFilePath = WorkspacePathTools.handleWorkingDirectoryFuzziness("ihmc-open-robotics-software");
      segmentationConfigurationFilePath = Paths.get(segmentationConfigurationFilePath.toString(), "/robot-environment-awareness/src/main/resources/" + SEGMENTATION_CONFIGURATION_FILE_NAME);

      Stage secondStage = new Stage();
      planarSegmentationUI = PlanarSegmentationUI.createIntraprocessUI(segmentationMessager, secondStage);
      segmentationModule = PlanarSegmentationModule.createIntraprocessModule(segmentationConfigurationFilePath.toFile(), segmentationMessager);

      slamModule.attachOcTreeConsumer(segmentationModule);

      slamUI.show();
      planarSegmentationUI.show();
      slamModule.start();
      segmentationModule.start();
   }

   @Override
   public void stop() throws Exception
   {
      slamUI.stop();
      slamModule.stop();

      planarSegmentationUI.stop();
      segmentationModule.stop();

      slamMessager.closeMessager();
      segmentationMessager.closeMessager();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
