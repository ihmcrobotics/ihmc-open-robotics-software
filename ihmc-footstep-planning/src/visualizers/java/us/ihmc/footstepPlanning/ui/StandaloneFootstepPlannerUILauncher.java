package us.ihmc.footstepPlanning.ui;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;

public class StandaloneFootstepPlannerUILauncher extends Application
{
   private final boolean visualize;
   private StandaloneFootstepPlannerUI ui;

   public StandaloneFootstepPlannerUILauncher()
   {
      this(true);
   }

   public StandaloneFootstepPlannerUILauncher(boolean visualize)
   {
      this.visualize = visualize;
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      ui = new StandaloneFootstepPlannerUI(primaryStage);
      if (visualize)
         ui.show();
   }

   @Override
   public void stop() throws Exception
   {
      super.stop();
      ui.stop();

      Platform.exit();
   }

   public StandaloneFootstepPlannerUI getUI()
   {
      return ui;
   }

   public static void main(String[] args)
   {
      launch();
   }
}
