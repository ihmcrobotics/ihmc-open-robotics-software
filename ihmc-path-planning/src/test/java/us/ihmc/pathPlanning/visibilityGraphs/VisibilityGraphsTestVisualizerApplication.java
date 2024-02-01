package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import javafx.stage.Stage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javafx.ApplicationNoModule;
import us.ihmc.log.LogTools;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.messager.javafx.SharedMemoryJavaFXMessager;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapSolution;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapWithNavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.pathPlanning.visibilityGraphs.visualizer.VisibilityGraphsTestVisualizer;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class VisibilityGraphsTestVisualizerApplication extends ApplicationNoModule
{
   // Default UI parameters which should be changeable on the fly
   private static final boolean showBodyPath = true;
   private static final boolean showClusterRawPoints = false;
   private static final boolean showClusterNavigableExtrusions = false;
   private static final boolean showClusterNonNavigableExtrusions = false;
   private static final boolean showRegionInnerConnections = false;
   private static final boolean showRegionInterConnections = false;

   //TODO: +++JerryPratt: Why or why do these have to be static? What if we want to launch several applications?
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

      setDefaultUISettings(messager);

      ui = new VisibilityGraphsTestVisualizer(primaryStage, messager);
      ui.show();

      uiHasBeenConstructed.set(true);
   }

   private void setDefaultUISettings(JavaFXMessager messager)
   {
      messager.submitMessage(UIVisibilityGraphsTopics.ShowBodyPath, showBodyPath);
      messager.submitMessage(UIVisibilityGraphsTopics.ShowClusterRawPoints, showClusterRawPoints);
      messager.submitMessage(UIVisibilityGraphsTopics.ShowClusterNavigableExtrusions, showClusterNavigableExtrusions);
      messager.submitMessage(UIVisibilityGraphsTopics.ShowClusterNonNavigableExtrusions, showClusterNonNavigableExtrusions);
      messager.submitMessage(UIVisibilityGraphsTopics.ShowInnerRegionVisibilityMapEdges, showRegionInnerConnections);
      messager.submitMessage(UIVisibilityGraphsTopics.ShowInterRegionVisibilityMap, showRegionInterConnections);
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
      VisibilityGraphsTestVisualizerApplication application = new VisibilityGraphsTestVisualizerApplication();
      application.startOnAThread();
      ThreadTools.sleepForever();
   }

   public void submitVisibilityGraphToVisualizer(VisibilityGraph visibilityGraph)
   {
      messager.submitMessage(UIVisibilityGraphsTopics.VisibilityGraph, visibilityGraph);
   }

   public void submitVisibilityGraphSolutionToVisualizer(VisibilityMapSolution visibilityMapSolution)
   {
      messager.submitMessage(UIVisibilityGraphsTopics.StartVisibilityMap, visibilityMapSolution.getStartMap());
      messager.submitMessage(UIVisibilityGraphsTopics.GoalVisibilityMap, visibilityMapSolution.getGoalMap());
      messager.submitMessage(UIVisibilityGraphsTopics.InterRegionVisibilityMap, visibilityMapSolution.getInterRegionVisibilityMap());

      submitNavigableRegionsToVisualizer(visibilityMapSolution.getVisibilityMapsWithNavigableRegions());
   }

   public void submitNavigableRegionsToVisualizer(List<VisibilityMapWithNavigableRegion> navigableRegions)
   {
      messager.submitMessage(UIVisibilityGraphsTopics.NavigableRegionData, navigableRegions);
      messager.submitMessage(UIVisibilityGraphsTopics.NavigableRegionVisibilityMap, navigableRegions);
   }

   public void submitStartToVisualizer(Point3D start)
   {
      messager.submitMessage(UIVisibilityGraphsTopics.StartPosition, start);
   }

   public void submitGoalToVisualizer(Point3D goal)
   {
      messager.submitMessage(UIVisibilityGraphsTopics.GoalPosition, goal);
   }

   public void submitPlanarRegionsListToVisualizer(PlanarRegionsList planarRegionsList)
   {
      messager.submitMessage(UIVisibilityGraphsTopics.PlanarRegionData, planarRegionsList);
   }

   public void submitShadowPlanarRegionsListToVisualizer(PlanarRegionsList planarRegionsList)
   {
      messager.submitMessage(UIVisibilityGraphsTopics.ShadowPlanarRegionData, planarRegionsList);
   }

}
