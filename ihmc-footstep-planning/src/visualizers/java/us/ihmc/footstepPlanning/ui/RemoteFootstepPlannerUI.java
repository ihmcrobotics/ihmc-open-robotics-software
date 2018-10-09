package us.ihmc.footstepPlanning.ui;

import javafx.application.Application;
import javafx.stage.Stage;
import us.ihmc.commons.thread.ThreadTools;
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

   private final JavaFXMessager messager;
   private final FootstepPlannerUIMessageConverter messageConverter;

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
      JavaFXMessager messager = new SharedMemoryJavaFXMessager(FootstepPlannerUserInterfaceAPI.API);
      FootstepPlannerUIMessageConverter messageConverter = FootstepPlannerUIMessageConverter.createConverter(messager, robotName, pubSubImplementation);
      return new RemoteFootstepPlannerUI(messager, messageConverter, visualize);
   }

   public RemoteFootstepPlannerUI(JavaFXMessager messager, FootstepPlannerUIMessageConverter messageConverter)
   {
      this(messager, messageConverter, true);
   }

   public RemoteFootstepPlannerUI(JavaFXMessager messager, FootstepPlannerUIMessageConverter messageConverter, boolean visualize)
   {
      this.messager = messager;
      this.messageConverter = messageConverter;
      this.visualize = visualize;
   }


   @Override
   public void start(Stage primaryStage) throws Exception
   {
      messager.startMessager();

      ui = FootstepPlannerUI.createMessagerUI(primaryStage, messager);

      if (visualize)
         ui.show();
   }

   @Override
   public void stop() throws Exception
   {
      super.stop();

      ui.stop();
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
