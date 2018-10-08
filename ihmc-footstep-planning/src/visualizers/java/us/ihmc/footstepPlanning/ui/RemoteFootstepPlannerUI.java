package us.ihmc.footstepPlanning.ui;

import javafx.application.Application;
import javafx.stage.Stage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;

public class RemoteFootstepPlannerUI extends Application
{
   private final boolean visualize;
   private final String robotName;

   private FootstepPathCalculatorModule module;
   private JavaFXMessager messager;
   private FootstepPlannerMessageConverter messageConverter;

   private FootstepPlannerUI ui;

   public RemoteFootstepPlannerUI(String robotName)
   {
      this(robotName, false);
   }

   public RemoteFootstepPlannerUI(String robotName, boolean visualize)
   {
      this.robotName = robotName;
      this.visualize = visualize;

   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      messager = new SharedMemoryJavaFXMessager(FootstepPlannerUserInterfaceAPI.API);
      messager.startMessager();

      messageConverter = FootstepPlannerMessageConverter.createRemoteConverter(messager, robotName);

      ui = FootstepPlannerUI.createMessagerUI(primaryStage, messager);
//      module = FootstepPathCalculatorModule.createMessagerModule(messager);

//      module.start();
      if (visualize)
         ui.show();
   }

   @Override
   public void stop() throws Exception
   {
      super.stop();

      ui.stop();
//      module.stop();

      messageConverter.destroy();
   }

   public boolean isRunning()
   {
      return ui != null && messager != null && messageConverter != null;
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
}
