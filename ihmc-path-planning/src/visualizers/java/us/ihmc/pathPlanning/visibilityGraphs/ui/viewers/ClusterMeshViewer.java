package us.ihmc.pathPlanning.visibilityGraphs.ui.viewers;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.ui.VisualizationParameters;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;

public class ClusterMeshViewer extends AnimationTimer
{
   private final boolean isExecutorServiceProvided;
   private final ExecutorService executorService;

   private final Group root = new Group();
   private final Group rawPointsGroup = new Group();
   private final Group navigableExtrusionsGroup = new Group();
   private final Group nonNavigableExtrusionsGroup = new Group();
   private final AtomicReference<Map<Integer, MeshView>> rawPointsToRenderReference = new AtomicReference<>(null);
   private final AtomicReference<Map<Integer, MeshView>> navigableExtrusionsToRenderReference = new AtomicReference<>(null);
   private final AtomicReference<Map<Integer, MeshView>> nonNavigableExtrusionsToRenderReference = new AtomicReference<>(null);

   private final AtomicReference<Boolean> resetRequested;
   private final AtomicReference<Boolean> showRawPoints;
   private final AtomicReference<Boolean> showNavigableExtrusions;
   private final AtomicReference<Boolean> showNonNavigableExtrusions;

   private final AtomicReference<List<NavigableRegion>> newRequestReference;

   public ClusterMeshViewer(REAMessager messager)
   {
      this(messager, null);
   }

   public ClusterMeshViewer(REAMessager messager, ExecutorService executorService)
   {
      isExecutorServiceProvided = executorService == null;

      if (isExecutorServiceProvided)
         this.executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
      else
         this.executorService = executorService;

      resetRequested = messager.createInput(UIVisibilityGraphsTopics.GlobalReset, false);
      showRawPoints = messager.createInput(UIVisibilityGraphsTopics.ShowClusterRawPoints, false);
      showNavigableExtrusions = messager.createInput(UIVisibilityGraphsTopics.ShowClusterNavigableExtrusions, false);
      showNonNavigableExtrusions = messager.createInput(UIVisibilityGraphsTopics.ShowClusterNonNavigableExtrusions, false);
      newRequestReference = messager.createInput(UIVisibilityGraphsTopics.NavigableRegionData, null);
      root.setMouseTransparent(true);
      root.getChildren().addAll(rawPointsGroup, navigableExtrusionsGroup, nonNavigableExtrusionsGroup);
   }

   @Override
   public void handle(long now)
   {
      if (resetRequested.getAndSet(false))
      {
         rawPointsGroup.getChildren().clear();
         rawPointsToRenderReference.getAndSet(null);
         navigableExtrusionsGroup.getChildren().clear();
         navigableExtrusionsToRenderReference.getAndSet(null);
         nonNavigableExtrusionsGroup.getChildren().clear();
         nonNavigableExtrusionsToRenderReference.getAndSet(null);
         return;
      }

      Map<Integer, MeshView> rawPointsToRender = rawPointsToRenderReference.get();

      if (rawPointsToRender != null)
      {
         rawPointsGroup.getChildren().clear();

         if (showRawPoints.get())
            rawPointsGroup.getChildren().addAll(rawPointsToRender.values());
      }

      Map<Integer, MeshView> navigableExtrusionsRender = navigableExtrusionsToRenderReference.get();

      if (navigableExtrusionsRender != null)
      {
         navigableExtrusionsGroup.getChildren().clear();

         if (showNavigableExtrusions.get())
            navigableExtrusionsGroup.getChildren().addAll(navigableExtrusionsRender.values());
      }

      Map<Integer, MeshView> nonNavigableExtrusionsRender = nonNavigableExtrusionsToRenderReference.get();

      if (nonNavigableExtrusionsRender != null)
      {
         nonNavigableExtrusionsGroup.getChildren().clear();

         if (showNonNavigableExtrusions.get())
            nonNavigableExtrusionsGroup.getChildren().addAll(nonNavigableExtrusionsRender.values());
      }

      if (showRawPoints.get() || showNavigableExtrusions.get() || showNonNavigableExtrusions.get())
      {
         List<NavigableRegion> newRequest = newRequestReference.getAndSet(null);

         if (newRequest != null)
            processNavigableRegionsOnThread(newRequest);
      }
   }

   private void processNavigableRegionsOnThread(List<NavigableRegion> navigableRegionLocalPlanners)
   {
      executorService.execute(() -> processNavigableRegions(navigableRegionLocalPlanners));
   }

   private void processNavigableRegions(List<NavigableRegion> navigableRegionLocalPlanners)
   {
      Map<Integer, JavaFXMeshBuilder> rawPointsMeshBuilders = new HashMap<>();
      Map<Integer, JavaFXMeshBuilder> navigableExtrusionsMeshBuilders = new HashMap<>();
      Map<Integer, JavaFXMeshBuilder> nonNavigableExtrusionsMeshBuilders = new HashMap<>();
      Map<Integer, Material> materials = new HashMap<>();

      for (NavigableRegion navigableRegionLocalPlanner : navigableRegionLocalPlanners)
      {
         int regionId = navigableRegionLocalPlanner.getMapId();
         JavaFXMeshBuilder rawPointsMeshBuilder = getOrCreate(rawPointsMeshBuilders, regionId);
         JavaFXMeshBuilder navigableExtrusionsMeshBuilder = getOrCreate(navigableExtrusionsMeshBuilders, regionId);
         JavaFXMeshBuilder nonNavigableExtrusionsMeshBuilder = getOrCreate(nonNavigableExtrusionsMeshBuilders, regionId);

         List<Cluster> clusters = navigableRegionLocalPlanner.getAllClusters();
         for (Cluster cluster : clusters)
         {
            for (Point3D rawPoint : cluster.getRawPointsInWorld3D())
               rawPointsMeshBuilder.addTetrahedron(VisualizationParameters.CLUSTER_RAWPOINT_SIZE, rawPoint);
            navigableExtrusionsMeshBuilder.addMultiLine(cluster.getNavigableExtrusionsInWorld3D(), VisualizationParameters.NAVIGABLECLUSTER_LINE_THICKNESS,
                                                        false);
            nonNavigableExtrusionsMeshBuilder.addMultiLine(cluster.getNonNavigableExtrusionsInWorld3D(),
                                                           VisualizationParameters.NON_NAVIGABLECLUSTER_LINE_THICKNESS, false);
         }

         materials.put(regionId, new PhongMaterial(getLineColor(regionId)));
      }

      HashMap<Integer, MeshView> rawPointsMapToRender = new HashMap<>();
      HashMap<Integer, MeshView> navigableExtrusionsMapToRender = new HashMap<>();
      HashMap<Integer, MeshView> nonNavigableExtrusionsMapToRender = new HashMap<>();

      for (Integer id : rawPointsMeshBuilders.keySet())
      {
         MeshView rawPointsMeshView = new MeshView(rawPointsMeshBuilders.get(id).generateMesh());
         rawPointsMeshView.setMaterial(materials.get(id));
         rawPointsMapToRender.put(id, rawPointsMeshView);

         MeshView navigableExtrusionsMeshView = new MeshView(navigableExtrusionsMeshBuilders.get(id).generateMesh());
         navigableExtrusionsMeshView.setMaterial(materials.get(id));
         navigableExtrusionsMapToRender.put(id, navigableExtrusionsMeshView);

         MeshView nonNavigableExtrusionsMeshView = new MeshView(nonNavigableExtrusionsMeshBuilders.get(id).generateMesh());
         nonNavigableExtrusionsMeshView.setMaterial(materials.get(id));
         nonNavigableExtrusionsMapToRender.put(id, nonNavigableExtrusionsMeshView);
      }

      rawPointsToRenderReference.set(rawPointsMapToRender);
      navigableExtrusionsToRenderReference.set(navigableExtrusionsMapToRender);
      nonNavigableExtrusionsToRenderReference.set(nonNavigableExtrusionsMapToRender);
   }

   private JavaFXMeshBuilder getOrCreate(Map<Integer, JavaFXMeshBuilder> meshBuilders, int regionId)
   {
      JavaFXMeshBuilder meshBuilder = meshBuilders.get(regionId);
      if (meshBuilder == null)
      {
         meshBuilder = new JavaFXMeshBuilder();
         meshBuilders.put(regionId, meshBuilder);
      }
      return meshBuilder;
   }

   private Color getLineColor(int regionId)
   {
      return PlanarRegionViewer.getRegionColor(regionId).darker();
   }

   @Override
   public void stop()
   {
      super.stop();
      if (!isExecutorServiceProvided)
         executorService.shutdownNow();
   }

   public Node getRoot()
   {
      return root;
   }
}
