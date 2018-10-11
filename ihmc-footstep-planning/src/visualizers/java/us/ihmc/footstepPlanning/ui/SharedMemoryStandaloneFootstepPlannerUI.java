package us.ihmc.footstepPlanning.ui;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.footstepPlanning.communication.FootstepPlannerSharedMemoryAPI;
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

   public SharedMemoryStandaloneFootstepPlannerUI()
   {
      this(true);
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      messager = new SharedMemoryJavaFXMessager(FootstepPlannerSharedMemoryAPI.API);
      messager.startMessager();

      module = FootstepPathCalculatorModule.createMessagerModule(messager);

      module.start();
      if (visualize)
      {
         ui = FootstepPlannerUI.createMessagerUI(primaryStage, messager);
         ui.show();
      }
   }

   @Override
   public void stop() throws Exception
   {
      super.stop();

      module.stop();
      if (visualize)
         ui.stop();

      Platform.exit();
   }

   public JavaFXMessager getMessager()
   {
      double maxTimeForStartUp = 5.0;
      double currentTime = 0.0;
      long sleepDuration = 100;

      while (!isRunning())
      {
         if (currentTime > maxTimeForStartUp)
            throw new RuntimeException("Failed to start.");

         currentTime += Conversions.millisecondsToSeconds(sleepDuration);
         ThreadTools.sleep(sleepDuration);
      }

      return messager;
   }

   private boolean isRunning()
   {
      return module != null && messager != null && (!visualize || ui != null);
   }

   public static void main(String[] args)
   {
      launch();
   }
}
