package us.ihmc.footstepPlanning.ui;

import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.ui.components.FootstepPathCalculatorModule;
import us.ihmc.javafx.ApplicationNoModule;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.messager.javafx.SharedMemoryJavaFXMessager;
import us.ihmc.pubsub.DomainFactory;

/**
 * This class provides a fully remote footstep planner. It includes the UI and the footstep planning modules, which are
 * published out using RTPS messages. They are converted from JavaFX messages internally and then broadcast out.
 */
public class RemoteStandaloneFootstepPlannerUI extends ApplicationNoModule
{
   private FootstepPathCalculatorModule module;
   private JavaFXMessager messager;
   private RemotePlannerMessageConverter messageConverter;

   private FootstepPlannerUI ui;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      messager = new SharedMemoryJavaFXMessager(FootstepPlannerMessagerAPI.API);
      messageConverter = RemotePlannerMessageConverter.createConverter(messager, "", DomainFactory.PubSubImplementation.INTRAPROCESS);
      module = new FootstepPathCalculatorModule(messager);

      messager.startMessager();
      module.start();

      ui = FootstepPlannerUI.createUI(primaryStage, messager);
      ui.show();
   }

   @Override
   public void stop() throws Exception
   {
      super.stop();

      module.stop();
      messager.closeMessager();
      messageConverter.destroy();
      ui.stop();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
