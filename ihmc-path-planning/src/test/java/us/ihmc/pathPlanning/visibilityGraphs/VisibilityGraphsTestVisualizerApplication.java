package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.concurrent.atomic.AtomicBoolean;

import javafx.application.Application;
import javafx.stage.Stage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.pathPlanning.visibilityGraphs.visualizer.VisibilityGraphsTestVisualizer;

public class VisibilityGraphsTestVisualizerApplication extends Application
{
   //TODO: +++JEP: Why or why do these have to be static? What if we want to launch several applications?
   private static VisibilityGraphsTestVisualizer ui;
   private static SharedMemoryJavaFXMessager messager;
   private static AtomicBoolean uiHasBeenConstructed = new AtomicBoolean(false);

   public VisibilityGraphsTestVisualizerApplication()
   {
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      messager = new SharedMemoryJavaFXMessager(UIVisibilityGraphsTopics.API);
      messager.startMessager();

      ui = new VisibilityGraphsTestVisualizer(primaryStage, messager);
      ui.show();

      uiHasBeenConstructed.set(true);
   }

   @Override
   public void stop() throws Exception
   {
      ui.stop();
   }

   public VisibilityGraphsTestVisualizer getUI()
   {
      return ui;
   }

   public SharedMemoryJavaFXMessager getMessager()
   {
      return messager;
   }

   public void startMeUp()
   {
      Thread thread = new Thread(() -> launch());
      thread.start();

      LogTools.info("Waiting for UI to be constructed.");
      while (!uiHasBeenConstructed.get())
      {
         ThreadTools.sleep(100L);
      }

      LogTools.info("UI is constructed.");
   }
}
