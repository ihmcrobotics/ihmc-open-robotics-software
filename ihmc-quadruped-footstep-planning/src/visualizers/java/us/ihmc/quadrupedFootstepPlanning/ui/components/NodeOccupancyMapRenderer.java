package us.ihmc.quadrupedFootstepPlanning.ui.components;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.ui.SimpleFootstepNode;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class NodeOccupancyMapRenderer extends AnimationTimer
{
   private final boolean isExecutorServiceProvided;
   private final ExecutorService executorService;

   private static final double cellWidth = 0.02;
   private static final double nodeOffsetZ = 0.05;

   private final Group root = new Group();

   private final AtomicReference<JavaFXMultiColorMeshBuilder> meshBuilderToRender = new AtomicReference<>(null);
   private final AtomicReference<PlanarRegionsList> planarRegionsList;
   private final AtomicBoolean show = new AtomicBoolean();
   private final AtomicBoolean reset = new AtomicBoolean(false);

   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);

   private final ConvexPolygon2D cellPolygon = new ConvexPolygon2D();

   public NodeOccupancyMapRenderer(Messager messager, ExecutorService executorService)
   {
      isExecutorServiceProvided = executorService == null;

      if (isExecutorServiceProvided)
         this.executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
      else
         this.executorService = executorService;

      messager.registerTopicListener(FootstepPlannerMessagerAPI.ComputePathTopic, data -> reset.set(true));
      planarRegionsList = messager.createInput(FootstepPlannerMessagerAPI.PlanarRegionDataTopic);

      cellPolygon.addVertex(cellWidth, 0.0);
      cellPolygon.addVertex(0.5 * cellWidth, 0.5 * Math.sqrt(3.0) * cellWidth);
      cellPolygon.addVertex(-0.5 * cellWidth, 0.5 * Math.sqrt(3.0) * cellWidth);
      cellPolygon.addVertex(-cellWidth, 0.0);
      cellPolygon.addVertex(-0.5 * cellWidth, -0.5 * Math.sqrt(3.0) * cellWidth);
      cellPolygon.addVertex(0.5 * cellWidth, -0.5 * Math.sqrt(3.0) * cellWidth);
      cellPolygon.update();
   }

   public void show(boolean show)
   {
      this.show.set(show);

      if (show)
         meshBuilderToRender.set(meshBuilder);
   }

   public void reset()
   {
      this.reset.set(true);
   }

   public void processNodesToRenderOnThread(Collection<SimpleFootstepNode> nodes, Color color)
   {
      executorService.execute(() -> processNodesToRender(nodes, color));
   }

   private void processNodesToRender(Collection<SimpleFootstepNode> nodes, Color color)
   {
      if (nodes == null || nodes.isEmpty())
         return;

      for (SimpleFootstepNode node : nodes)
      {
         double x = node.getXIndex() * FootstepNode.gridSizeXY;
         double y = node.getYIndex() * FootstepNode.gridSizeXY;
         double z = getHeightAtPoint(x, y) + nodeOffsetZ;
         RigidBodyTransform transform = new RigidBodyTransform();
         transform.setTranslation(x, y, z);

         meshBuilder.addPolygon(transform, cellPolygon, color);
      }

      meshBuilderToRender.set(meshBuilder);
   }

   private double getHeightAtPoint(double x, double y)
   {
      PlanarRegionsList planarRegionsList = this.planarRegionsList.get();
      if (planarRegionsList == null)
         return 0.0;
      Point3DReadOnly projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(new Point3D(x, y, 100.0), planarRegionsList);
      return projectedPoint == null ? 0.0 : projectedPoint.getZ();
   }

   @Override
   public void handle(long now)
   {
      if (show.get() && meshBuilderToRender.get() != null)
      {
         JavaFXMultiColorMeshBuilder meshBuilder = meshBuilderToRender.getAndSet(null);
         MeshView meshView = new MeshView();
         meshView.setMesh(meshBuilder.generateMesh());
         meshView.setMaterial(meshBuilder.generateMaterial());

         root.getChildren().clear();
         root.getChildren().add(meshView);
      }
      else if (!show.get() && !root.getChildren().isEmpty())
      {
         root.getChildren().clear();
      }

      if (reset.getAndSet(false))
      {
         meshBuilder.clear();
         meshBuilderToRender.set(null);
      }
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
