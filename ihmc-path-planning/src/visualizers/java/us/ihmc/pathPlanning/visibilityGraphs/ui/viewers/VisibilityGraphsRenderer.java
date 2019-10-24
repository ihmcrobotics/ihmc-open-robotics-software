package us.ihmc.pathPlanning.visibilityGraphs.ui.viewers;

import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.BodyPathData;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.GlobalReset;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.GoalVisibilityMap;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.InterRegionVisibilityMap;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.NavigableRegionData;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.NavigableRegionVisibilityMap;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ShowClusterNavigableExtrusions;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ShowClusterNonNavigableExtrusions;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ShowClusterRawPoints;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ShowGoalVisibilityMap;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ShowInterRegionVisibilityMap;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ShowNavigableRegionVisibilityMaps;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ShowStartVisibilityMap;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.StartVisibilityMap;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionsManager;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
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

   private final AtomicReference<VisibilityGraphsParametersReadOnly> parameters;

   private final BodyPathMeshViewer bodyPathMeshViewer;
   private final ClusterMeshViewer clusterMeshViewer;
   private final VisibilityMapHolderViewer startMapViewer;
   private final VisibilityMapHolderViewer goalMapViewer;
   private final NavigableRegionViewer navigableRegionViewer;
   private final VisibilityMapHolderViewer interRegionConnectionsViewer;

   private final Messager messager;

   public VisibilityGraphsRenderer(Messager messager)
   {
      this.messager = messager;
      planarRegionsReference = messager.createInput(UIVisibilityGraphsTopics.PlanarRegionData);
      startPositionReference = messager.createInput(UIVisibilityGraphsTopics.StartPosition);
      goalPositionReference = messager.createInput(UIVisibilityGraphsTopics.GoalPosition);
      parameters = messager.createInput(UIVisibilityGraphsTopics.VisibilityGraphsParameters, new DefaultVisibilityGraphParameters());

      messager.registerTopicListener(UIVisibilityGraphsTopics.VisibilityGraphsComputePath, request -> computePathOnThread(false));
      messager.registerTopicListener(UIVisibilityGraphsTopics.VisibilityGraphsComputePathWithOcclusions, request -> computePathOnThread(true));

      bodyPathMeshViewer = new BodyPathMeshViewer(messager, executorService);

      clusterMeshViewer = new ClusterMeshViewer(messager, executorService);
      clusterMeshViewer.setTopics(GlobalReset, ShowClusterRawPoints, ShowClusterNavigableExtrusions, ShowClusterNonNavigableExtrusions, NavigableRegionData);

      startMapViewer = new VisibilityMapHolderViewer(messager, executorService);
      startMapViewer.setCustomColor(Color.YELLOW);
      startMapViewer.setTopics(ShowStartVisibilityMap, StartVisibilityMap);

      goalMapViewer = new VisibilityMapHolderViewer(messager, executorService);
      goalMapViewer.setCustomColor(Color.CORNFLOWERBLUE);
      goalMapViewer.setTopics(ShowGoalVisibilityMap, GoalVisibilityMap);

      navigableRegionViewer = new NavigableRegionViewer(messager, executorService);
      navigableRegionViewer.setTopics(GlobalReset, ShowNavigableRegionVisibilityMaps, NavigableRegionVisibilityMap);

      interRegionConnectionsViewer = new VisibilityMapHolderViewer(messager, executorService);
      interRegionConnectionsViewer.setCustomColor(Color.CRIMSON);
      interRegionConnectionsViewer.setTopics(ShowInterRegionVisibilityMap, InterRegionVisibilityMap);

      root.getChildren().addAll(bodyPathMeshViewer.getRoot(), clusterMeshViewer.getRoot(), startMapViewer.getRoot(), goalMapViewer.getRoot(),
                                navigableRegionViewer.getRoot(), interRegionConnectionsViewer.getRoot());
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
      startMapViewer.start();
      goalMapViewer.start();
      navigableRegionViewer.start();
      interRegionConnectionsViewer.start();
   }

   public void stop()
   {
      bodyPathMeshViewer.stop();
      clusterMeshViewer.stop();
      startMapViewer.stop();
      goalMapViewer.stop();
      navigableRegionViewer.start();
      interRegionConnectionsViewer.stop();
      executorService.shutdownNow();
   }

   private void computePathOnThread(boolean computePathWithOcclusions)
   {
      executorService.submit(() -> computePath(computePathWithOcclusions));
   }

   private void computePath(boolean computePathWithOcclusions)
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
         LogTools.info("Computing body path.", this);

      try
      {
         List<PlanarRegion> planarRegions = planarRegionsList.getPlanarRegionsAsList();

         NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters.get());
         navigableRegionsManager.setPlanarRegions(planarRegions);

         List<Point3DReadOnly> bodyPath;
         if (computePathWithOcclusions)
         {
            bodyPath = navigableRegionsManager.calculateBodyPath(start, goal);
         }
         else
         {
            bodyPath = navigableRegionsManager.calculateBodyPath(start, goal);
         }

         messager.submitMessage(BodyPathData, bodyPath);
         messager.submitMessage(StartVisibilityMap, navigableRegionsManager.getStartMap());
         messager.submitMessage(GoalVisibilityMap, navigableRegionsManager.getGoalMap());
         messager.submitMessage(NavigableRegionVisibilityMap, navigableRegionsManager.getNavigableRegionsList());
         messager.submitMessage(InterRegionVisibilityMap, navigableRegionsManager.getInterRegionConnections());
         messager.submitMessage(NavigableRegionData, navigableRegionsManager.getNavigableRegionsList());
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
