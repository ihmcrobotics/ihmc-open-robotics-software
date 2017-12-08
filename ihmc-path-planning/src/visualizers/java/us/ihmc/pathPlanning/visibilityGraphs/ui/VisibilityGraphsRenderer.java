package us.ihmc.pathPlanning.visibilityGraphs.ui;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import javafx.scene.Group;
import javafx.scene.Node;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pathPlanning.visibilityGraphs.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionsManager;
import us.ihmc.pathPlanning.visibilityGraphs.VisibilityGraphsParameters;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class VisibilityGraphsRenderer
{
   private static final boolean VERBOSE = true;

   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final Group root = new Group();

   private final AtomicReference<PlanarRegionsList> planarRegionsReference;
   private final AtomicReference<Point3D> startPositionReference;
   private final AtomicReference<Point3D> goalPositionReference;

   private final AtomicReference<VisibilityGraphsParameters> parameters;

   private final BodyPathMeshViewer bodyPathMeshViewer;
   private final NavigableRegionInnerVizMapMeshViewer navigableRegionInnerVizMapMeshViewer;
   private final NavigableRegionsInterConnectionViewer navigableRegionsInterConnectionViewer;
   private final ClusterMeshViewer clusterMeshViewer;

   private NavigableRegionsManager navigableRegionsManager;

   public VisibilityGraphsRenderer(REAMessager messager)
   {
      planarRegionsReference = messager.createInput(UIVisibilityGraphsTopics.PlanarRegionData);
      startPositionReference = messager.createInput(UIVisibilityGraphsTopics.StartPosition);
      goalPositionReference = messager.createInput(UIVisibilityGraphsTopics.GoalPosition);
      parameters = messager.createInput(UIVisibilityGraphsTopics.VisibilityGraphsParameters, new DefaultVisibilityGraphParameters());

      messager.registerTopicListener(UIVisibilityGraphsTopics.VisibilityGraphsComputePath, request -> computePathOnThread());

      bodyPathMeshViewer = new BodyPathMeshViewer(messager);
      root.getChildren().add(bodyPathMeshViewer.getRoot());
      navigableRegionInnerVizMapMeshViewer = new NavigableRegionInnerVizMapMeshViewer(messager);
      root.getChildren().add(navigableRegionInnerVizMapMeshViewer.getRoot());
      
      navigableRegionsManager = new NavigableRegionsManager(parameters.get());
      navigableRegionsInterConnectionViewer = new NavigableRegionsInterConnectionViewer(messager, navigableRegionsManager);
      root.getChildren().add(navigableRegionsInterConnectionViewer.getRoot());
      clusterMeshViewer = new ClusterMeshViewer(messager);
      root.getChildren().add(clusterMeshViewer.getRoot());
   }

   public void clear()
   {
      planarRegionsReference.set(null);
      startPositionReference.set(null);
      goalPositionReference.set(null);
   }

   public void start()
   {
      bodyPathMeshViewer.start();
      navigableRegionInnerVizMapMeshViewer.start();
      clusterMeshViewer.start();
      navigableRegionsInterConnectionViewer.start();
   }

   public void stop()
   {
      bodyPathMeshViewer.stop();
      navigableRegionInnerVizMapMeshViewer.start();
      clusterMeshViewer.stop();
      navigableRegionsInterConnectionViewer.stop();
      executorService.shutdownNow();
   }

   private void computePathOnThread()
   {
      executorService.submit(this::computePath);
   }

   private void computePath()
   {
      PlanarRegionsList planarRegionsList = planarRegionsReference.get();

      if (planarRegionsList == null)
         return;

      Point3D start = startPositionReference.get();

      if (start == null)
         return;

      Point3D goal = goalPositionReference.get();

      if (goal == null)
         return;

      if (VERBOSE)
         PrintTools.info(this, "Computing body path.");

      try
      {
         List<PlanarRegion> planarRegions = planarRegionsList.getPlanarRegionsAsList();
         
         planarRegions = planarRegions.stream().filter(region -> region.getConcaveHullSize() > 2).collect(Collectors.toList());

         navigableRegionsManager.setPlanarRegions(planarRegions);

         List<Point3D> bodyPath = navigableRegionsManager.calculateBodyPath(start, goal);
         bodyPathMeshViewer.processBodyPath(bodyPath);
         navigableRegionInnerVizMapMeshViewer.processNavigableRegions(navigableRegionsManager.getListOfLocalPlanners());

         clusterMeshViewer.processNavigableRegions(navigableRegionsManager.getListOfLocalPlanners());
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public Node getRoot()
   {
      return root;
   }
}
