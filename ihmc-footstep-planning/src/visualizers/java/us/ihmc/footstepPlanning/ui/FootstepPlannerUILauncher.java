package us.ihmc.footstepPlanning.ui;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;

public class FootstepPlannerUILauncher extends Application
{
   private final boolean visualize;
   private FootstepPlannerUI ui;

   public FootstepPlannerUILauncher()
   {
      this(true);
   }

   public FootstepPlannerUILauncher(boolean visualize)
   {
      this.visualize = visualize;
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      ui = new FootstepPlannerUI(primaryStage);
      if (visualize)
         ui.show();
   }

   @Override
   public void stop() throws Exception
   {
      super.stop();
      ui.stop();

//      Platform.exit();
   }

   public FootstepPlannerUI getUI()
   {
      return ui;
   }

   public static void main(String[] args)
   {
      launch();
   }
}
