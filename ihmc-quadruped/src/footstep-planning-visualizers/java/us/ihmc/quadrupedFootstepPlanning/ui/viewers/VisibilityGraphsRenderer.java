package us.ihmc.quadrupedFootstepPlanning.ui.viewers;

import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.ClusterMeshViewer;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.NavigableRegionViewer;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.VisibilityMapHolderViewer;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawStepPlannerMessagerAPI;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;


public class VisibilityGraphsRenderer
{
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final Group root = new Group();

   private final AtomicReference<PlanarRegionsList> planarRegionsReference;
   private final AtomicReference<Point3D> startPositionReference;
   private final AtomicReference<Point3D> goalPositionReference;

   private final ClusterMeshViewer clusterMeshViewer;
   private final VisibilityMapHolderViewer startMapViewer;
   private final VisibilityMapHolderViewer goalMapViewer;
   private final NavigableRegionViewer navigableRegionViewer;
   private final VisibilityMapHolderViewer interRegionConnectionsViewer;

   public VisibilityGraphsRenderer(Messager messager)
   {
      planarRegionsReference = messager.createInput(PawStepPlannerMessagerAPI.PlanarRegionDataTopic);
      startPositionReference = messager.createInput(PawStepPlannerMessagerAPI.StartPositionTopic);
      goalPositionReference = messager.createInput(PawStepPlannerMessagerAPI.GoalPositionTopic);

      clusterMeshViewer = new ClusterMeshViewer(messager, executorService);
      clusterMeshViewer.setTopics(PawStepPlannerMessagerAPI.GlobalResetTopic, PawStepPlannerMessagerAPI.ShowClusterRawPoints,
                                  PawStepPlannerMessagerAPI.ShowClusterNavigableExtrusions, PawStepPlannerMessagerAPI.ShowClusterNonNavigableExtrusions,
                                  PawStepPlannerMessagerAPI.VisibilityMapWithNavigableRegionData);

      startMapViewer = new VisibilityMapHolderViewer(messager, executorService);
      startMapViewer.setCustomColor(Color.YELLOW);
      startMapViewer.setTopics(PawStepPlannerMessagerAPI.ShowStartVisibilityMap, PawStepPlannerMessagerAPI.StartVisibilityMap);

      goalMapViewer = new VisibilityMapHolderViewer(messager, executorService);
      goalMapViewer.setCustomColor(Color.CORNFLOWERBLUE);
      goalMapViewer.setTopics(PawStepPlannerMessagerAPI.ShowGoalVisibilityMap, PawStepPlannerMessagerAPI.GoalVisibilityMap);

      navigableRegionViewer = new NavigableRegionViewer(messager, executorService);
      navigableRegionViewer.setTopics(PawStepPlannerMessagerAPI.GlobalResetTopic, PawStepPlannerMessagerAPI.ShowNavigableRegionVisibilityMaps,
                                      PawStepPlannerMessagerAPI.VisibilityMapWithNavigableRegionData);

      interRegionConnectionsViewer = new VisibilityMapHolderViewer(messager, executorService);
      interRegionConnectionsViewer.setCustomColor(Color.CRIMSON);
      interRegionConnectionsViewer.setTopics(PawStepPlannerMessagerAPI.ShowInterRegionVisibilityMap, PawStepPlannerMessagerAPI.InterRegionVisibilityMap);

      root.getChildren().addAll(clusterMeshViewer.getRoot(), startMapViewer.getRoot(), goalMapViewer.getRoot(), navigableRegionViewer.getRoot(),
                                interRegionConnectionsViewer.getRoot());
   }

   public void clear()
   {
      planarRegionsReference.set(null);
      startPositionReference.set(null);
      goalPositionReference.set(null);
   }

   public void start()
   {
      clusterMeshViewer.start();
      startMapViewer.start();
      goalMapViewer.start();
      navigableRegionViewer.start();
      interRegionConnectionsViewer.start();
   }

   public void stop()
   {
      clusterMeshViewer.stop();
      startMapViewer.stop();
      goalMapViewer.stop();
      navigableRegionViewer.start();
      interRegionConnectionsViewer.stop();
      executorService.shutdownNow();
   }

   public Node getRoot()
   {
      return root;
   }
}
