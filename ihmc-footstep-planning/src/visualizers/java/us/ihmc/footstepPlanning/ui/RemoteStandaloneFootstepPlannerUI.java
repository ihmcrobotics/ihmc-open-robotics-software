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
import us.ihmc.pubsub.DomainFactory;

/**
 * This class provides a fully remote footstep planner. It includes the UI and the footstep planning modules, which are
 * published out using RTPS messages. They are converted from JavaFX messages internally and then broadcast out.
 */
public class RemoteStandaloneFootstepPlannerUI extends Application
{
   private final boolean visualize;

   private FootstepPathCalculatorModule module;
   private JavaFXMessager messager;
   private RemotePlannerMessageConverter messageConverter;

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

   public RemoteStandaloneFootstepPlannerUI()
   {
      this.visualize = true;
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {

      if (messager == null)
         messager = new SharedMemoryJavaFXMessager(FootstepPlannerSharedMemoryAPI.API);
      if (messageConverter == null)
         messageConverter = RemotePlannerMessageConverter.createConverter(messager, "", DomainFactory.PubSubImplementation.INTRAPROCESS);
      if (module == null)
         module = new FootstepPathCalculatorModule(messager);

      messager.startMessager();

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

      if (module != null)
         module.stop();

      if (messager != null)
         messager.closeMessager();

      if (messageConverter != null)
         messageConverter.destroy();

      if (ui != null)
         ui.stop();

      Platform.exit();
   }

   public boolean isRunning()
   {
      return messager != null && messageConverter != null && (!visualize || ui != null);
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

   public static void main(String[] args)
   {
      launch(args);
   }
}
