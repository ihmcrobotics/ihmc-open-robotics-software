package us.ihmc.quadrupedFootstepPlanning.ui.components;

import controller_msgs.msg.dds.FootstepPlannerCellMessage;
import controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage;
import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.util.Pair;
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
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.Collection;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class NodeOccupancyMapRenderer extends AnimationTimer
{
   private static final double cellWidth = 0.02;
   private static final double nodeOffsetZ = 0.05;

   private ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final Group root = new Group();
   private final AtomicReference<Pair<Mesh, Material>> nodeGraphToRender = new AtomicReference<>(null);
   private final AtomicReference<PlanarRegionsList> planarRegionsList = new AtomicReference<>(null);
   private final AtomicBoolean show = new AtomicBoolean();
   private final AtomicBoolean reset = new AtomicBoolean(false);

   private final MeshView nodeGraphMeshView = new MeshView();
   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final ConvexPolygon2D cellPolygon = new ConvexPolygon2D();

   public NodeOccupancyMapRenderer(Messager messager)
   {
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ComputePathTopic, data -> reset.set(true));

      cellPolygon.addVertex(cellWidth, 0.0);
      cellPolygon.addVertex(0.5 * cellWidth, 0.5 * Math.sqrt(3.0) * cellWidth);
      cellPolygon.addVertex(-0.5 * cellWidth, 0.5 * Math.sqrt(3.0) * cellWidth);
      cellPolygon.addVertex(-cellWidth, 0.0);
      cellPolygon.addVertex(-0.5 * cellWidth, -0.5 * Math.sqrt(3.0) * cellWidth);
      cellPolygon.addVertex(0.5 * cellWidth, -0.5 * Math.sqrt(3.0) * cellWidth);
      cellPolygon.update();

      root.getChildren().add(nodeGraphMeshView);
   }

   public void show(boolean show)
   {
      this.show.set(show);
   }

   public void reset()
   {
      this.reset.set(true);
   }

   public void processNodesToRender(Collection<SimpleFootstepNode> nodes, Color color)
   {
      palette.clearPalette();
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);

      for (SimpleFootstepNode node : nodes)
      {
         double x = node.getXIndex() * FootstepNode.gridSizeXY;
         double y = node.getYIndex() * FootstepNode.gridSizeXY;
         double z = getHeightAtPoint(x, y) + nodeOffsetZ;
         RigidBodyTransform transform = new RigidBodyTransform();
         transform.setTranslation(x, y, z);

         meshBuilder.addPolygon(transform, cellPolygon, color);
      }

      nodeGraphToRender.set(new Pair<>(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
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
      if (show.get() && root.getChildren().isEmpty())
         root.getChildren().add(nodeGraphMeshView);
      else if (!show.get() && !root.getChildren().isEmpty())
         root.getChildren().clear();

      if (reset.getAndSet(false))
      {
         nodeGraphToRender.set(null);
         nodeGraphMeshView.setMesh(null);
         nodeGraphMeshView.setMaterial(null);
         return;
      }

      Pair<Mesh, Material> newMesh = nodeGraphToRender.get();
      if (newMesh != null)
      {
         nodeGraphMeshView.setMesh(newMesh.getKey());
         nodeGraphMeshView.setMaterial(newMesh.getValue());
      }
   }

   @Override
   public void stop()
   {
      super.stop();
      executorService.shutdownNow();
   }

   public Node getRoot()
   {
      return root;
   }
}
