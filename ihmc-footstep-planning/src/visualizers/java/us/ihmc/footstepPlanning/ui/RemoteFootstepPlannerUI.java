package us.ihmc.footstepPlanning.ui;

import com.sun.javafx.application.PlatformImpl;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.footstepPlanning.communication.FootstepPlannerSharedMemoryAPI;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.pubsub.DomainFactory;

/**
 * This class provides a visualizer for the remote footstep planner found in the footstep planner toolbox.
 * It allows users to view the resulting plans calculated by the toolbox. It also allows the user to tune
 * the planner parameters, and request a new plan from the planning toolbox.
 */
public class RemoteFootstepPlannerUI extends Application
{
   private final boolean visualize;

   private JavaFXMessager messager;
   private RemoteUIMessageConverter messageConverter;

   private FootstepPlannerUI ui;

   public static RemoteFootstepPlannerUI createIntraprocessUI(String robotName)
   {
      return createUI(robotName, DomainFactory.PubSubImplementation.INTRAPROCESS);
   }

   public static RemoteFootstepPlannerUI createFastRTPSUI(String robotName)
   {
      return createUI(robotName, DomainFactory.PubSubImplementation.FAST_RTPS);
   }

   public static RemoteFootstepPlannerUI createUI(String robotName, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      return createUI(robotName, pubSubImplementation, true);
   }

   public static RemoteFootstepPlannerUI createUI(String robotName, DomainFactory.PubSubImplementation pubSubImplementation, boolean visualize)
   {
      JavaFXMessager messager = new SharedMemoryJavaFXMessager(FootstepPlannerSharedMemoryAPI.API);
      RemoteUIMessageConverter messageConverter = RemoteUIMessageConverter.createConverter(messager, robotName, pubSubImplementation);
      return new RemoteFootstepPlannerUI(messager, messageConverter, visualize);
   }

   public RemoteFootstepPlannerUI(JavaFXMessager messager, RemoteUIMessageConverter messageConverter)
   {
      this(messager, messageConverter, true);
   }

   public RemoteFootstepPlannerUI(JavaFXMessager messager, RemoteUIMessageConverter messageConverter, boolean visualize)
   {
      this.messager = messager;
      this.messageConverter = messageConverter;
      this.visualize = visualize;
   }

   public RemoteFootstepPlannerUI()
   {
      this.visualize = true;
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      if (messager == null)
         messager = new SharedMemoryJavaFXMessager(FootstepPlannerSharedMemoryAPI.API);
      if (messageConverter == null)
         messageConverter = RemoteUIMessageConverter.createConverter(messager, "", DomainFactory.PubSubImplementation.INTRAPROCESS);

      messager.startMessager();

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
