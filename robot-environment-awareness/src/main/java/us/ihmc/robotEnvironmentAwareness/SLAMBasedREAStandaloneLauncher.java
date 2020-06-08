package us.ihmc.robotEnvironmentAwareness;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMModule;
import us.ihmc.robotEnvironmentAwareness.ui.PlanarSegmentationUI;
import us.ihmc.robotEnvironmentAwareness.ui.SLAMBasedEnvironmentAwarenessUI;
import us.ihmc.robotEnvironmentAwareness.updaters.PlanarSegmentationModule;

public class SLAMBasedREAStandaloneLauncher extends Application
{
   private static final String MODULE_CONFIGURATION_FILE_NAME = "./Configurations/defaultREAModuleConfiguration.txt";

   private SLAMBasedEnvironmentAwarenessUI slamUI;
   private PlanarSegmentationUI planarSegmentationUI;
   private SLAMModule slamModule;
   private PlanarSegmentationModule segmentationModule;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      slamUI = SLAMBasedEnvironmentAwarenessUI.creatIntraprocessUI(primaryStage);
      planarSegmentationUI = PlanarSegmentationUI.creatIntraprocessUI(primaryStage);

      slamModule = SLAMModule.createIntraprocessModule();
      segmentationModule = PlanarSegmentationModule.createIntraprocessModule(MODULE_CONFIGURATION_FILE_NAME);

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
      planarSegmentationUI.stop();
      slamModule.stop();
      segmentationModule.stop();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
