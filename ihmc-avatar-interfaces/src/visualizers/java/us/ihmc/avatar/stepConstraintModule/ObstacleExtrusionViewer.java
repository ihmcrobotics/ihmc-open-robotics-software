package us.ihmc.avatar.stepConstraintModule;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.javaFXVisualizers.IdMappedColorFunction;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapWithNavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.ui.VisualizationParameters;
import us.ihmc.robotics.RegionInWorldInterface;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DBasics;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DReadOnly;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

public class ObstacleExtrusionViewer extends AnimationTimer
{
   private final boolean isExecutorServiceProvided;
   private final ExecutorService executorService;

   private final Group root = new Group();
   private final Group rawPointsGroup = new Group();
   private final Group extrusionsGroup = new Group();
   private final AtomicReference<Map<Integer, MeshView>> rawPointsToRenderReference = new AtomicReference<>(null);
   private final AtomicReference<Map<Integer, MeshView>> extrusionsToRenderReference = new AtomicReference<>(null);

//   private AtomicReference<Boolean> resetRequested;
   private AtomicReference<Boolean> showRawPoints;
   private AtomicReference<Boolean> showExtrusions;

   private AtomicReference<HashMap<RegionInWorldInterface, List<ConcavePolygon2DBasics>>> newRequestReference;

   private final Messager messager;

   public ObstacleExtrusionViewer(Messager messager)
   {
      this(messager, null);
   }

   public ObstacleExtrusionViewer(Messager messager, ExecutorService executorService)
   {
      this.messager = messager;

      isExecutorServiceProvided = executorService == null;

      if (isExecutorServiceProvided)
         this.executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
      else
         this.executorService = executorService;

      root.setMouseTransparent(true);
      root.getChildren().addAll(rawPointsGroup, extrusionsGroup);
   }

   public void setTopics(Topic<Boolean> resetRequestedTopic, Topic<Boolean> showExtrusionsRawPointsTopic, Topic<Boolean> showObstacleExtrusionsTopic,
                         Topic<HashMap<RegionInWorldInterface, List<ConcavePolygon2DBasics>>> obstacleExtrusionsTopic)
   {
//      resetRequested = messager.createInput(resetRequestedTopic, false);
      showRawPoints = messager.createInput(showExtrusionsRawPointsTopic, false);
      showExtrusions = messager.createInput(showObstacleExtrusionsTopic, false);
      newRequestReference = messager.createInput(obstacleExtrusionsTopic, null);
   }

   @Override
   public void handle(long now)
   {
//      if (resetRequested.getAndSet(false))
//      {
//         rawPointsGroup.getChildren().clear();
//         rawPointsToRenderReference.getAndSet(null);
//         extrusionsGroup.getChildren().clear();
//         extrusionsToRenderReference.getAndSet(null);
//         return;
//      }

      Map<Integer, MeshView> rawPointsToRender = rawPointsToRenderReference.get();

      if (rawPointsToRender != null)
      {
         rawPointsGroup.getChildren().clear();

         if (showRawPoints.get())
            rawPointsGroup.getChildren().addAll(rawPointsToRender.values());
      }


      Map<Integer, MeshView> extrusionsRender = extrusionsToRenderReference.get();

      if (extrusionsRender != null)
      {
         extrusionsGroup.getChildren().clear();

         if (showExtrusions.get())
            extrusionsGroup.getChildren().addAll(extrusionsRender.values());
      }

      if (showRawPoints.get()  || showExtrusions.get())
      {
         HashMap<RegionInWorldInterface, List<ConcavePolygon2DBasics>> newRequest = newRequestReference.getAndSet(null);

         if (newRequest != null)
            processNavigableRegionsOnThread(newRequest);
      }
   }

   private void processNavigableRegionsOnThread(HashMap<RegionInWorldInterface, List<ConcavePolygon2DBasics>> obstacleExtrusionsMap)
   {
      executorService.execute(() -> processNavigableRegions(obstacleExtrusionsMap));
   }

   private void processNavigableRegions(HashMap<RegionInWorldInterface, List<ConcavePolygon2DBasics>> obstacleExtrusionsMap)
   {
      Map<Integer, JavaFXMeshBuilder> rawPointsMeshBuilders = new HashMap<>();
      Map<Integer, JavaFXMeshBuilder> extrusionMeshBuilders = new HashMap<>();
      Map<Integer, Material> navigableMaterials = new HashMap<>();

      for (RegionInWorldInterface constraintRegion : obstacleExtrusionsMap.keySet())
      {
         int regionId = constraintRegion.getRegionId();
         JavaFXMeshBuilder rawPointsMeshBuilder = getOrCreate(rawPointsMeshBuilders, regionId);
         JavaFXMeshBuilder extrusionMeshBuilder = getOrCreate(extrusionMeshBuilders, regionId);

         List<ConcavePolygon2DBasics> obstacleExtrusions = obstacleExtrusionsMap.get(constraintRegion);

         for (ConcavePolygon2DBasics obstacleExtrusion : obstacleExtrusions)
         {
            buildRawClusterPoints(rawPointsMeshBuilder, constraintRegion.getTransformToWorld(), obstacleExtrusion);
            buildExtrusion(extrusionMeshBuilder, constraintRegion.getTransformToWorld(), obstacleExtrusion);
         }
         navigableMaterials.put(regionId, new PhongMaterial(getLineColor(regionId)));
      }

      HashMap<Integer, MeshView> rawPointsMapToRender = new HashMap<>();
      HashMap<Integer, MeshView> extrusionsMapToRender = new HashMap<>();

      for (Integer id : rawPointsMeshBuilders.keySet())
      {
         MeshView rawPointsMeshView = new MeshView(rawPointsMeshBuilders.get(id).generateMesh());
         rawPointsMeshView.setMaterial(navigableMaterials.get(id));
         rawPointsMapToRender.put(id, rawPointsMeshView);

         MeshView extrusionsMeshView = new MeshView(extrusionMeshBuilders.get(id).generateMesh());
         extrusionsMeshView.setMaterial(navigableMaterials.get(id));
         extrusionsMapToRender.put(id, extrusionsMeshView);
      }

      rawPointsToRenderReference.set(rawPointsMapToRender);
      extrusionsToRenderReference.set(extrusionsMapToRender);
   }

   private void buildRawClusterPoints(JavaFXMeshBuilder rawPointsMeshBuilder, RigidBodyTransformReadOnly transformToWorld, ConcavePolygon2DReadOnly extrusion)
   {
      for (Point2DReadOnly rawPoint : extrusion.getVertexBufferView())
      {
         Point3D pointInWorld = new Point3D(rawPoint);
         transformToWorld.transform(pointInWorld);

         rawPointsMeshBuilder.addTetrahedron(VisualizationParameters.CLUSTER_RAWPOINT_SIZE, pointInWorld);
      }
   }

   private void buildExtrusion(JavaFXMeshBuilder extrusionMeshBuilder, RigidBodyTransformReadOnly transformToWorld, ConcavePolygon2DReadOnly extrusion)
   {
      List<Point3DReadOnly> pointsInWorld = new ArrayList<>();

      for (Point2DReadOnly rawPoint : extrusion.getVertexBufferView())
      {
         Point3D pointInWorld = new Point3D(rawPoint);
         transformToWorld.transform(pointInWorld);

         pointsInWorld.add(pointInWorld);
      }

      extrusionMeshBuilder
            .addMultiLine(pointsInWorld, VisualizationParameters.NAVIGABLECLUSTER_LINE_THICKNESS, true);
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
