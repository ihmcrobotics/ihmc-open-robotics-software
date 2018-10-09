package us.ihmc.footstepPlanning.ui;

import javafx.application.Application;
import javafx.stage.Stage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.footstepPlanning.communication.FootstepPlannerSharedMemoryAPI;
import us.ihmc.footstepPlanning.ui.components.FootstepPathCalculatorModule;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.pubsub.DomainFactory;

/**
 * This class provides a fully remote footstep planner. It includes the UI and the footstep planning modules, which are
 * published out using RTPS messages. They are converted from JavaFX messages internally and then broadcast out.
 */
public class RemoteStandaloneFootstepPlannerUI extends Application
{
   private final boolean visualize;

   private FootstepPathCalculatorModule module;
   private final JavaFXMessager messager;
   private final RemotePlannerMessageConverter messageConverter;

   private FootstepPlannerUI ui;


   public static RemoteStandaloneFootstepPlannerUI createIntraprocessUI(String robotName)
   {
      return createUI(robotName, DomainFactory.PubSubImplementation.INTRAPROCESS);
   }

   public static RemoteStandaloneFootstepPlannerUI createFastRTPSUI(String robotName)
   {
      return createUI(robotName, DomainFactory.PubSubImplementation.FAST_RTPS);
   }

   public static RemoteStandaloneFootstepPlannerUI createUI(String robotName, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      return createUI(robotName, pubSubImplementation, true);
   }

   public static RemoteStandaloneFootstepPlannerUI createUI(String robotName, DomainFactory.PubSubImplementation pubSubImplementation, boolean visualize)
   {
      JavaFXMessager messager = new SharedMemoryJavaFXMessager(FootstepPlannerSharedMemoryAPI.API);
      RemotePlannerMessageConverter messageConverter = RemotePlannerMessageConverter.createConverter(messager, robotName, pubSubImplementation);
      return new RemoteStandaloneFootstepPlannerUI(messager, messageConverter, visualize);
   }

   public RemoteStandaloneFootstepPlannerUI(JavaFXMessager messager, RemotePlannerMessageConverter messageConverter)
   {
      this(messager, messageConverter, true);
   }

   public RemoteStandaloneFootstepPlannerUI(JavaFXMessager messager, RemotePlannerMessageConverter messageConverter, boolean visualize)
   {
      this.messager = messager;
      this.messageConverter = messageConverter;
      this.module = new FootstepPathCalculatorModule(messager);
      this.visualize = visualize;
   }


   @Override
   public void start(Stage primaryStage) throws Exception
   {
      messager.startMessager();

      module.start();

      ui = FootstepPlannerUI.createMessagerUI(primaryStage, messager);

      if (visualize)
         ui.show();
   }

   @Override
   public void stop() throws Exception
   {
      super.stop();

      ui.stop();
      module.stop();
      messager.closeMessager();

      messageConverter.destroy();
   }

   public boolean isRunning()
   {
      return ui != null && messager != null && messageConverter != null;
   }

   public JavaFXMessager getMessager()
   {
      while (!isRunning())
         ThreadTools.sleep(100);

      return messager;
   }
}
