package us.ihmc.pathPlanning.visibilityGraphs.ui;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;

public class SimpleUILauncher extends Application
{
   private SimpleVisibilityGraphsUI ui;
   

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      ui = new SimpleVisibilityGraphsUI(primaryStage);
      ui.show();
   }

   @Override
   public void stop() throws Exception
   {
      super.stop();
      ui.stop();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch();
   }
}
