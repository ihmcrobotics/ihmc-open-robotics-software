package us.ihmc.pathPlanning.visibilityGraphs.ui.viewers;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.application.Platform;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.pathPlanning.visibilityGraphs.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionsManager;
import us.ihmc.pathPlanning.visibilityGraphs.VisibilityMap;
import us.ihmc.pathPlanning.visibilityGraphs.ui.VisualizationParameters;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;

public class NavigableRegionInnerVizMapMeshViewer extends AnimationTimer
{
   private final Group root = new Group();
   private AtomicReference<Map<Integer, MeshView>> regionVisMapToRenderReference = new AtomicReference<>(null);
   private Map<Integer, MeshView> regionVisMapRendered;

   private final REAMessager messager;
   private final AtomicReference<Boolean> resetRequested;
   private final AtomicReference<Boolean> show;

   public NavigableRegionInnerVizMapMeshViewer(REAMessager messager)
   {
      this.messager = messager;
      resetRequested = messager.createInput(UIVisibilityGraphsTopics.GlobalReset, false);
      show = messager.createInput(UIVisibilityGraphsTopics.ShowLocalGraphs, false);
      messager.registerTopicListener(UIVisibilityGraphsTopics.ShowLocalGraphs, this::handleShowThreadSafe);
      root.setMouseTransparent(true);
   }

   private void handleShowThreadSafe(boolean show)
   {
      if (Platform.isFxApplicationThread())
         handleShow(show);
      else
         Platform.runLater(() -> handleShow(show));
   }

   private void handleShow(boolean show)
   {
      if (!show)
         root.getChildren().clear();
      else
         root.getChildren().addAll(regionVisMapRendered.values());
   }

   @Override
   public void handle(long now)
   {
      if (resetRequested.getAndSet(false))
      {
         root.getChildren().clear();
         regionVisMapToRenderReference.getAndSet(null);
         return;
      }

      Map<Integer, MeshView> regionVisMapToRender = regionVisMapToRenderReference.getAndSet(null);

      if (regionVisMapToRender != null)
      {
         regionVisMapRendered = regionVisMapToRender;

         if (show.get())
         {
            root.getChildren().clear();
            root.getChildren().addAll(regionVisMapToRender.values());
         }
      }
   }

   public void processNavigableRegions(List<NavigableRegion> navigableRegionLocalPlanners)
   {
      Map<Integer, JavaFXMeshBuilder> meshBuilders = new HashMap<>();
      Map<Integer, Material> materials = new HashMap<>();

      for (NavigableRegion navigableRegionLocalPlanner : navigableRegionLocalPlanners)
      {
         int regionId = navigableRegionLocalPlanner.getRegionId();
         JavaFXMeshBuilder meshBuilder = meshBuilders.get(regionId);
         if (meshBuilder == null)
         {
            meshBuilder = new JavaFXMeshBuilder();
            meshBuilders.put(regionId, meshBuilder);
         }

         RigidBodyTransform transformToWorld = navigableRegionLocalPlanner.getLocalReferenceFrame().getTransformToWorldFrame();
         VisibilityMap localVisibilityGraph = navigableRegionLocalPlanner.getLocalVisibilityGraph();

         for (Connection connection : localVisibilityGraph.getConnections())
         {
            Point3D edgeSource = toWorld(new Point2D(connection.getSourcePoint().getX(), connection.getSourcePoint().getY()), transformToWorld);
            Point3D edgeTarget = toWorld(new Point2D(connection.getTargetPoint().getX(), connection.getTargetPoint().getY()), transformToWorld);
            meshBuilder.addLine(edgeSource, edgeTarget, VisualizationParameters.VISBILITYMAP_LINE_THICKNESS);
         }

         materials.put(regionId, new PhongMaterial(getLineColor(regionId)));
      }

      HashMap<Integer, MeshView> regionVisMapToRender = new HashMap<>();

      for (Integer id : meshBuilders.keySet())
      {
         MeshView meshView = new MeshView(meshBuilders.get(id).generateMesh());
         meshView.setMaterial(materials.get(id));
         regionVisMapToRender.put(id, meshView);
      }

      regionVisMapToRenderReference.set(regionVisMapToRender);
   }

   public void processGlobalMap(NavigableRegionsManager navigableRegionsManager)
   {
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();

      for (Connection connection : navigableRegionsManager.getGlobalMapPoints())
      {
         meshBuilder.addLine(connection.getSourcePoint(), connection.getTargetPoint(), VisualizationParameters.VISBILITYMAP_LINE_THICKNESS);
      }
      
      HashMap<Integer, MeshView> regionVisMapToRender = new HashMap<>();
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(new PhongMaterial(getLineColor(50)));
      regionVisMapToRender.put(50, meshView);
      regionVisMapToRenderReference.set(regionVisMapToRender);

   }

   private Color getLineColor(int regionId)
   {
      return VizGraphsPlanarRegionViewer.getRegionColor(regionId).brighter();
   }

   private Point3D toWorld(Point2D localPoint, RigidBodyTransform transformToWorld)
   {
      Point3D worldPoint = new Point3D(localPoint);
      transformToWorld.transform(worldPoint);
      return worldPoint;
   }

   public Node getRoot()
   {
      return root;
   }
}
