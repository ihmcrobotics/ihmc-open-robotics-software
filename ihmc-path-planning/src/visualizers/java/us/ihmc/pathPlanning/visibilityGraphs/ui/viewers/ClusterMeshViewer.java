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
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.javaFXVisualizers.IdMappedColorFunction;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapWithNavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.ui.VisualizationParameters;

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

   private AtomicReference<Boolean> resetRequested;
   private AtomicReference<Boolean> showRawPoints;
   private AtomicReference<Boolean> showNavigableExtrusions;
   private AtomicReference<Boolean> showNonNavigableExtrusions;

   private AtomicReference<List<VisibilityMapWithNavigableRegion>> newRequestReference;

   private final Messager messager;

   public ClusterMeshViewer(Messager messager)
   {
      this(messager, null);
   }

   public ClusterMeshViewer(Messager messager, ExecutorService executorService)
   {
      this.messager = messager;

      isExecutorServiceProvided = executorService == null;

      if (isExecutorServiceProvided)
         this.executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
      else
         this.executorService = executorService;

      root.setMouseTransparent(true);
      root.getChildren().addAll(rawPointsGroup, navigableExtrusionsGroup, nonNavigableExtrusionsGroup);
   }

   public void setTopics(Topic<Boolean> resetRequestedTopic, Topic<Boolean> showClusterRawPointsTopic, Topic<Boolean> showClusterNavigableExtrusionsTopic,
                         Topic<Boolean> showClusterNonNavigableExtrusionsTopic, Topic<List<VisibilityMapWithNavigableRegion>> navigableRegionDataTopic)
   {
      resetRequested = messager.createInput(resetRequestedTopic, false);
      showRawPoints = messager.createInput(showClusterRawPointsTopic, false);
      showNavigableExtrusions = messager.createInput(showClusterNavigableExtrusionsTopic, false);
      showNonNavigableExtrusions = messager.createInput(showClusterNonNavigableExtrusionsTopic, false);
      newRequestReference = messager.createInput(navigableRegionDataTopic, null);
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
         List<VisibilityMapWithNavigableRegion> newRequest = newRequestReference.getAndSet(null);

         if (newRequest != null)
            processNavigableRegionsOnThread(newRequest);
      }
   }

   private void processNavigableRegionsOnThread(List<VisibilityMapWithNavigableRegion> navigableRegionLocalPlanners)
   {
      executorService.execute(() -> processNavigableRegions(navigableRegionLocalPlanners));
   }

   private void processNavigableRegions(List<VisibilityMapWithNavigableRegion> navigableRegionLocalPlanners)
   {
      Map<Integer, JavaFXMeshBuilder> rawPointsMeshBuilders = new HashMap<>();
      Map<Integer, JavaFXMeshBuilder> navigableExtrusionsMeshBuilders = new HashMap<>();
      Map<Integer, JavaFXMeshBuilder> nonNavigableExtrusionsMeshBuilders = new HashMap<>();
      Map<Integer, Material> navigableMaterials = new HashMap<>();
      Map<Integer, Material> nonNavigableMaterials = new HashMap<>();

      for (VisibilityMapWithNavigableRegion navigableRegionLocalPlanner : navigableRegionLocalPlanners)
      {
         int regionId = navigableRegionLocalPlanner.getMapId();
         JavaFXMeshBuilder rawPointsMeshBuilder = getOrCreate(rawPointsMeshBuilders, regionId);
         JavaFXMeshBuilder navigableExtrusionsMeshBuilder = getOrCreate(navigableExtrusionsMeshBuilders, regionId);
         JavaFXMeshBuilder nonNavigableExtrusionsMeshBuilder = getOrCreate(nonNavigableExtrusionsMeshBuilders, regionId);

         buildNavigableExtrusion(navigableExtrusionsMeshBuilder, navigableRegionLocalPlanner.getHomeRegionCluster());

         List<Cluster> allClusters = navigableRegionLocalPlanner.getAllClusters();
         for (Cluster cluster : allClusters)
         {
            buildRawClusterPoints(rawPointsMeshBuilder, cluster);
         }
         
         //TODO: +++JerryPratt: Show Nonnavigable regions for all clusters or just obstacle clusters?
         //TODO: +++JerryPratt: Or make a third boolean checkbox to show obstacle non-navigable vs. homeRegion non-navigable.
         List<Cluster> obstacleClusters = navigableRegionLocalPlanner.getObstacleClusters();
         for (Cluster cluster : obstacleClusters)
         {
            buildNonNavigableExtrusion(nonNavigableExtrusionsMeshBuilder, cluster);
         }

         navigableMaterials.put(regionId, new PhongMaterial(getNavigableLineColor(regionId)));
         nonNavigableMaterials.put(regionId, new PhongMaterial(getNonNavigableLineColor(regionId)));
      }

      HashMap<Integer, MeshView> rawPointsMapToRender = new HashMap<>();
      HashMap<Integer, MeshView> navigableExtrusionsMapToRender = new HashMap<>();
      HashMap<Integer, MeshView> nonNavigableExtrusionsMapToRender = new HashMap<>();

      for (Integer id : rawPointsMeshBuilders.keySet())
      {
         MeshView rawPointsMeshView = new MeshView(rawPointsMeshBuilders.get(id).generateMesh());
         rawPointsMeshView.setMaterial(nonNavigableMaterials.get(id));
         rawPointsMapToRender.put(id, rawPointsMeshView);

         MeshView navigableExtrusionsMeshView = new MeshView(navigableExtrusionsMeshBuilders.get(id).generateMesh());
         navigableExtrusionsMeshView.setMaterial(navigableMaterials.get(id));
         navigableExtrusionsMapToRender.put(id, navigableExtrusionsMeshView);

         MeshView nonNavigableExtrusionsMeshView = new MeshView(nonNavigableExtrusionsMeshBuilders.get(id).generateMesh());
         nonNavigableExtrusionsMeshView.setMaterial(nonNavigableMaterials.get(id));
         nonNavigableExtrusionsMapToRender.put(id, nonNavigableExtrusionsMeshView);
      }

      rawPointsToRenderReference.set(rawPointsMapToRender);
      navigableExtrusionsToRenderReference.set(navigableExtrusionsMapToRender);
      nonNavigableExtrusionsToRenderReference.set(nonNavigableExtrusionsMapToRender);
   }

   private void buildRawClusterPoints(JavaFXMeshBuilder rawPointsMeshBuilder, Cluster cluster)
   {
      for (Point3DReadOnly rawPoint : cluster.getRawPointsInWorld())
      {
         rawPointsMeshBuilder.addTetrahedron(VisualizationParameters.CLUSTER_RAWPOINT_SIZE, rawPoint);
      }
   }
 
   private void buildNavigableExtrusion(JavaFXMeshBuilder navigableExtrusionsMeshBuilder, Cluster cluster)
   {
      boolean close = cluster.isClosed();
      
      navigableExtrusionsMeshBuilder
            .addMultiLine(cluster.getNavigableExtrusionsInWorld(), VisualizationParameters.NAVIGABLECLUSTER_LINE_THICKNESS, close);
   }
   
   private void buildNonNavigableExtrusion(JavaFXMeshBuilder nonNavigableExtrusionsMeshBuilder, Cluster cluster)
   {
      boolean close = cluster.isClosed();

      nonNavigableExtrusionsMeshBuilder
            .addMultiLine(cluster.getNonNavigableExtrusionsInWorld(), VisualizationParameters.NON_NAVIGABLECLUSTER_LINE_THICKNESS, close);
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

   private Color getNonNavigableLineColor(int regionId)
   {
      return IdMappedColorFunction.INSTANCE.apply(regionId).darker();
   }

   private Color getNavigableLineColor(int regionId)
   {
      return IdMappedColorFunction.INSTANCE.apply(regionId).brighter();
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
