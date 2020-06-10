package us.ihmc.atlas.sensors;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.robotEnvironmentAwareness.ui.AtlasPerceptionSuiteUI;
import us.ihmc.robotEnvironmentAwareness.ui.PlanarSegmentationUI;
import us.ihmc.robotEnvironmentAwareness.ui.SLAMBasedEnvironmentAwarenessUI;

public class AltasPerceptionSuiteLauncher extends Application
{
   private AtlasPerceptionSuite module;

   private AtlasPerceptionSuiteUI ui;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      module = AtlasPerceptionSuite.createIntraprocess();
      ui = AtlasPerceptionSuiteUI.createIntraprocessUI(primaryStage);

      module.start();
      ui.show();
   }

   @Override
   public void stop() throws Exception
   {
      module.stop();
      ui.stop();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
