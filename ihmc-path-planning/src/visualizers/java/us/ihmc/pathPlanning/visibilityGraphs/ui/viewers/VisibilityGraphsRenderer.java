package us.ihmc.pathPlanning.visibilityGraphs.ui.viewers;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

import javafx.scene.Group;
import javafx.scene.Node;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionsManager;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
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
   private final ClusterMeshViewer clusterMeshViewer;
   private final NavigableRegionInnerVizMapMeshViewer navigableRegionInnerVizMapMeshViewer;
   private final NavigableRegionsInterConnectionViewer navigableRegionsInterConnectionViewer;

   private final REAMessager messager;

   public VisibilityGraphsRenderer(REAMessager messager)
   {
      this.messager = messager;
      planarRegionsReference = messager.createInput(UIVisibilityGraphsTopics.PlanarRegionData);
      startPositionReference = messager.createInput(UIVisibilityGraphsTopics.StartPosition);
      goalPositionReference = messager.createInput(UIVisibilityGraphsTopics.GoalPosition);
      parameters = messager.createInput(UIVisibilityGraphsTopics.VisibilityGraphsParameters, new DefaultVisibilityGraphParameters());

      messager.registerTopicListener(UIVisibilityGraphsTopics.VisibilityGraphsComputePath, request -> computePathOnThread());

      bodyPathMeshViewer = new BodyPathMeshViewer(messager, executorService);
      clusterMeshViewer = new ClusterMeshViewer(messager, executorService);
      navigableRegionInnerVizMapMeshViewer = new NavigableRegionInnerVizMapMeshViewer(messager, executorService);
      navigableRegionsInterConnectionViewer = new NavigableRegionsInterConnectionViewer(messager, executorService);

      root.getChildren().addAll(bodyPathMeshViewer.getRoot(), navigableRegionInnerVizMapMeshViewer.getRoot(), navigableRegionsInterConnectionViewer.getRoot(), clusterMeshViewer.getRoot());
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
      clusterMeshViewer.start();
      navigableRegionInnerVizMapMeshViewer.start();
      navigableRegionsInterConnectionViewer.start();
   }

   public void stop()
   {
      bodyPathMeshViewer.stop();
      clusterMeshViewer.stop();
      navigableRegionInnerVizMapMeshViewer.start();
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

         NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters.get());
         navigableRegionsManager.setPlanarRegions(planarRegions);
         List<Point3DReadOnly> bodyPath = navigableRegionsManager.calculateBodyPath(start, goal);

         messager.submitMessage(UIVisibilityGraphsTopics.BodyPathData, bodyPath);
         messager.submitMessage(UIVisibilityGraphsTopics.NavigableRegionData, navigableRegionsManager.getListOfLocalPlanners());
         messager.submitMessage(UIVisibilityGraphsTopics.InterRegionConnectionData, navigableRegionsManager.getInterRegionConnections());
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
