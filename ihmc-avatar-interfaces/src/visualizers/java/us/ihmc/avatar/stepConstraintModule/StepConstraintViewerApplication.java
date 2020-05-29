package us.ihmc.avatar.stepConstraintModule;

import javafx.application.Application;
import javafx.stage.Stage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class StepConstraintViewerApplication extends Application
{
   private static StepConstraintCalculatorViewer ui;
   private static SharedMemoryJavaFXMessager messager;
   private static AtomicBoolean uiHasBeenConstructed = new AtomicBoolean(false);

   public StepConstraintViewerApplication()
   {
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      messager = new SharedMemoryJavaFXMessager(StepConstraintCalculatorViewerAPI.API);
      messager.startMessager();

      ui = new StepConstraintCalculatorViewer(primaryStage, messager);
      ui.show();

      uiHasBeenConstructed.set(true);
   }

   @Override
   public void stop() throws Exception
   {
      ui.stop();
   }

   public StepConstraintCalculatorViewer getUI()
   {
      return ui;
   }

   public SharedMemoryJavaFXMessager getMessager()
   {
      return messager;
   }

   public void startOnAThread()
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

   public static void main(String[] args)
   {
      StepConstraintViewerApplication application = new StepConstraintViewerApplication();
      application.startOnAThread();
      ThreadTools.sleepForever();
   }

   public void submitPlanarRegionsListToVisualizer(PlanarRegionsList planarRegionsList)
   {
      messager.submitMessage(StepConstraintCalculatorViewerAPI.PlanarRegionData, planarRegionsList);
   }

   public void submitStepConstraintRegionsToVisualizer(List<StepConstraintRegion> stepConstraintRegionList)
   {
      messager.submitMessage(StepConstraintCalculatorViewerAPI.StepConstraintRegionData, stepConstraintRegionList);
   }


}
