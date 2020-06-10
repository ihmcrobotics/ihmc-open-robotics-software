package us.ihmc.atlas.sensors;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.robotEnvironmentAwareness.ui.PerceptionSuiteUI;

public class AltasPerceptionSuiteLauncher extends Application
{
   private AtlasPerceptionSuite module;

   private PerceptionSuiteUI ui;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      module = AtlasPerceptionSuite.createIntraprocess();
      ui = PerceptionSuiteUI.createIntraprocessUI(primaryStage);

      module.start();
      ui.show();

      primaryStage.setOnCloseRequest(event -> stop());
   }

   @Override
   public void stop()
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
