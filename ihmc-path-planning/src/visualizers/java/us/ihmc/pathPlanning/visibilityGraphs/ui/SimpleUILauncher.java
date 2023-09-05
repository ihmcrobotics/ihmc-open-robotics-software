package us.ihmc.pathPlanning.visibilityGraphs.ui;

import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.javafx.ApplicationNoModule;

public class SimpleUILauncher extends ApplicationNoModule
{
   private static final boolean SHOW_FILE_CHOOSER_ON_START = false;

   private SimpleVisibilityGraphsUI ui;
   

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      ui = new SimpleVisibilityGraphsUI(primaryStage);
      ui.show(SHOW_FILE_CHOOSER_ON_START);
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
