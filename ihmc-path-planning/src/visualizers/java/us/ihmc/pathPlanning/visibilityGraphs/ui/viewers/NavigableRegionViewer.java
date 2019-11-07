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
import us.ihmc.pathPlanning.visibilityGraphs.VisibilityGraph;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.*;
import us.ihmc.pathPlanning.visibilityGraphs.ui.VisualizationParameters;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;

public class NavigableRegionViewer extends AnimationTimer
{
   private final boolean isExecutorServiceProvided;
   private final ExecutorService executorService;

   private final Group root = new Group();
   private AtomicReference<Map<Integer, MeshView>> regionVisMapToRenderReference = new AtomicReference<>(null);
   private AtomicReference<Boolean> resetRequested;
   private AtomicReference<Boolean> showInnerEdges;
   private AtomicReference<Boolean> showHomeRegionNodes;

   private AtomicReference<List<VisibilityMapWithNavigableRegion>> newRequestReference;
   private AtomicReference<VisibilityGraph> latestVisibilityGraphReference;

   private final Messager messager;
   private List<VisibilityMapWithNavigableRegion> lastestPulledSolution;
   private VisibilityGraph latestPulledFullGraphs;
   private boolean lastShowInnerEdges;
   private boolean lastShowHomeRegionNodes;

   public NavigableRegionViewer(Messager messager)
   {
      this(messager, null);
   }

   public NavigableRegionViewer(Messager messager, ExecutorService executorService)
   {
      this.messager = messager;

      isExecutorServiceProvided = executorService == null;

      if (isExecutorServiceProvided)
         this.executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
      else
         this.executorService = executorService;

      root.setMouseTransparent(true);
   }

   public void setTopics(Topic<Boolean> globalResetTopic, Topic<Boolean> showNavigableRegionVisibilityMapsTopic, Topic<List<VisibilityMapWithNavigableRegion>> navigableRegionVisibilityMapTopic)
   {
      resetRequested = messager.createInput(globalResetTopic, false);
      showInnerEdges = messager.createInput(showNavigableRegionVisibilityMapsTopic, false);
      showHomeRegionNodes = messager.createInput(UIVisibilityGraphsTopics.ShowInnerRegionVisibilityMapHomeNodes, false);
      newRequestReference = messager.createInput(navigableRegionVisibilityMapTopic, null);
      latestVisibilityGraphReference = messager.createInput(UIVisibilityGraphsTopics.VisibilityGraph, null);
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

      Map<Integer, MeshView> regionVisMapToRender = regionVisMapToRenderReference.get();

      if (regionVisMapToRender != null)
      {
         root.getChildren().clear();

         if (showInnerEdges.get() || showHomeRegionNodes.get())
            root.getChildren().addAll(regionVisMapToRender.values());
      }

      boolean tempShowInnerEdges = showInnerEdges.get();
      boolean tempShowHomeRegionNodes = showHomeRegionNodes.get();
      if (tempShowInnerEdges || tempShowHomeRegionNodes)
      {
         List<VisibilityMapWithNavigableRegion> latestSolutionTemp = newRequestReference.getAndSet(null);
         if (latestSolutionTemp != null)
         {
            lastestPulledSolution = latestSolutionTemp;
         }

         VisibilityGraph latestFullGraphsTemp = latestVisibilityGraphReference.getAndSet(null);
         if (latestSolutionTemp != null)
         {
            latestPulledFullGraphs = latestFullGraphsTemp;
         }

         boolean recompute = false;
         recompute |= tempShowInnerEdges != lastShowInnerEdges;
         recompute |= tempShowHomeRegionNodes != lastShowHomeRegionNodes;
         if (latestSolutionTemp != null || latestSolutionTemp != null || recompute)
         {
            processNavigableRegionsOnThread(lastestPulledSolution, latestPulledFullGraphs);
         }
      }
      lastShowInnerEdges = tempShowInnerEdges;
      lastShowHomeRegionNodes = tempShowHomeRegionNodes;
   }

   private void processNavigableRegionsOnThread(List<VisibilityMapWithNavigableRegion> newRequest, VisibilityGraph visibilityGraph)
   {
      executorService.execute(() -> processNavigableRegions(newRequest, visibilityGraph));
   }

   private void processNavigableRegions(List<VisibilityMapWithNavigableRegion> newRequest, VisibilityGraph visibilityGraph)
   {
      Map<Integer, JavaFXMeshBuilder> meshBuilders = new HashMap<>();
      Map<Integer, Material> materials = new HashMap<>();

      for (VisibilityMapWithNavigableRegion navigableRegionLocalPlanner : newRequest)
      {
         int regionId = navigableRegionLocalPlanner.getMapId();
         JavaFXMeshBuilder meshBuilder = meshBuilders.get(regionId);
         if (meshBuilder == null)
         {
            meshBuilder = new JavaFXMeshBuilder();
            meshBuilders.put(regionId, meshBuilder);
         }

         VisibilityMap visibilityGraphInWorld = navigableRegionLocalPlanner.getVisibilityMapInWorld();

         if (showHomeRegionNodes.get())
         {
            for (VisibilityGraphNavigableRegion visibilityGraphNavigableRegion : visibilityGraph.getVisibilityGraphNavigableRegions())
            {
               for (VisibilityGraphNode homeRegionNode : visibilityGraphNavigableRegion.getHomeRegionNodes())
               {
                  ConnectionPoint3D pointInWorld = homeRegionNode.getPointInWorld();
                  meshBuilder.addTetrahedron(VisualizationParameters.ESCAPE_EXTRUDEDPOINT_SIZE, pointInWorld);
               }
            }
         }

         if (showInnerEdges.get())
         {
            for (Connection connection : visibilityGraphInWorld.getConnections())
            {
               Point3DReadOnly edgeSource = connection.getSourcePoint();
               Point3DReadOnly edgeTarget = connection.getTargetPoint();
               meshBuilder.addLine(edgeSource, edgeTarget, VisualizationParameters.VISBILITYMAP_LINE_THICKNESS);
            }
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
