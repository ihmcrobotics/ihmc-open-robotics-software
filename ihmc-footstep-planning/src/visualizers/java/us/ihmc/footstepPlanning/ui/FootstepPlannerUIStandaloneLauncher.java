package us.ihmc.footstepPlanning.ui;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;

public class FootstepPlannerUIStandaloneLauncher extends Application
{
   private FootstepPlannerUI ui;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      ui = new FootstepPlannerUI(primaryStage);
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
