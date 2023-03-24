package us.ihmc.avatar.stepConstraintModule;

import java.util.HashMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import javafx.stage.Stage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.javafx.ApplicationNoModule;
import us.ihmc.log.LogTools;
import us.ihmc.messager.javafx.SharedMemoryJavaFXMessager;
import us.ihmc.robotics.RegionInWorldInterface;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DBasics;

public class StepConstraintViewerApplication extends ApplicationNoModule
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

   public void submitObstacleExtrusions(HashMap<RegionInWorldInterface<?>, List<ConcavePolygon2DBasics>> obstacleExtrusions)
   {
      messager.submitMessage(StepConstraintCalculatorViewerAPI.ObstacleExtrusionsData, obstacleExtrusions);
   }

   public void submitTooSmallRegions(List<PlanarRegion> tooSmallRegions)
   {
      messager.submitMessage(StepConstraintCalculatorViewerAPI.TooSmallRegionData, new PlanarRegionsList(tooSmallRegions));
   }

   public void submitTooSteepRegions(List<PlanarRegion> tooSteepRegions)
   {
      messager.submitMessage(StepConstraintCalculatorViewerAPI.TooSteepRegionData, new PlanarRegionsList(tooSteepRegions));
   }

   public void submitMaskedRegions(List<PlanarRegion> maskedRegions)
   {
      messager.submitMessage(StepConstraintCalculatorViewerAPI.MaskedRegionsData, new PlanarRegionsList(maskedRegions));
   }

   public void submitMaskedRegionsObstacleExtrusions(HashMap<RegionInWorldInterface<?>, List<ConcavePolygon2DBasics>> extrusions)
   {
      messager.submitMessage(StepConstraintCalculatorViewerAPI.MaskedRegionsObstacleExtrusionsData, extrusions);
   }
}
