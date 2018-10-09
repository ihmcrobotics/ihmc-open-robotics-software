package us.ihmc.footstepPlanning.ui;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.footstepPlanning.ui.components.FootstepPathCalculatorModule;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;

/**
 * This class provides a fully remote footstep planner. It includes the UI and the footstep planning modules, which are
 * available using JavaFX shared memory messages.
 */
public class SharedMemoryStandaloneFootstepPlannerUI extends Application
{
   private final boolean visualize;

   private JavaFXMessager messager;
   private FootstepPathCalculatorModule module;

   private FootstepPlannerUI ui;

   public SharedMemoryStandaloneFootstepPlannerUI(boolean visualize)
   {
      this.visualize = visualize;
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      messager = new SharedMemoryJavaFXMessager(FootstepPlannerUserInterfaceAPI.API);
      messager.startMessager();

      ui = FootstepPlannerUI.createMessagerUI(primaryStage, messager);
      module = FootstepPathCalculatorModule.createMessagerModule(messager);

      module.start();
      if (visualize)
         ui.show();
   }

   @Override
   public void stop() throws Exception
   {
      super.stop();

      ui.stop();
      module.stop();

      Platform.exit();
   }

   public JavaFXMessager getMessager()
   {
      while (!isLaunched())
         ThreadTools.sleep(100);

      return messager;
   }

   private boolean isLaunched()
   {
      return ui != null && module != null && messager != null;
   }


   public static void main(String[] args)
   {
      launch();
   }
}
