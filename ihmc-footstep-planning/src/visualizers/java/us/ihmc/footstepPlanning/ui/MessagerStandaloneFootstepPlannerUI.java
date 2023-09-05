package us.ihmc.footstepPlanning.ui;

import javafx.stage.Stage;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.ui.components.FootstepPathCalculatorModule;
import us.ihmc.javafx.ApplicationNoModule;
import us.ihmc.messager.javafx.SharedMemoryJavaFXMessager;

/**
 * This class provides a fully remote footstep planner. It includes the UI and the footstep planning modules, which are
 * available using JavaFX shared memory messages.
 */
public class MessagerStandaloneFootstepPlannerUI extends ApplicationNoModule
{
   private SharedMemoryJavaFXMessager messager;
   private FootstepPathCalculatorModule module;

   private FootstepPlannerUI ui;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      messager = new SharedMemoryJavaFXMessager(FootstepPlannerMessagerAPI.API);
      messager.startMessager();

      module = FootstepPathCalculatorModule.createMessagerModule(messager);
      module.start();

      ui = FootstepPlannerUI.createUI(primaryStage, messager);
      ui.show();

   }

   @Override
   public void stop() throws Exception
   {
      super.stop();

      messager.closeMessager();
      module.stop();
      ui.stop();
   }

   public static void main(String[] args)
   {
      launch();
   }
}
