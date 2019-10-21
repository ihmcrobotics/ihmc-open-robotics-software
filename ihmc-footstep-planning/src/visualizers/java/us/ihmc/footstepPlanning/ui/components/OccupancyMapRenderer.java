package us.ihmc.footstepPlanning.ui.components;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

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
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class OccupancyMapRenderer extends AnimationTimer
{
   private static final double cellWidth = 0.02;
   private static final double nodeOffsetZ = 0.05;
   private static final double cellOpacity = 0.9;
   private static final Color validCellColor = Color.rgb(219, 124, 87, cellOpacity);
   private static final Color rejectedCellColor = Color.rgb(139, 0, 0, cellOpacity);

   private ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final Group root = new Group();
   private final AtomicReference<Pair<Mesh, Material>> footstepGraphToRender = new AtomicReference<>(null);
   private final AtomicReference<PlanarRegionsList> planarRegionsList = new AtomicReference<>(null);
   private final AtomicReference<Boolean> show;
   private final AtomicBoolean reset = new AtomicBoolean(false);

   private final MeshView footstepGraphMeshView = new MeshView();
   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final ConvexPolygon2D cellPolygon = new ConvexPolygon2D();

   public OccupancyMapRenderer(Messager messager)
   {
      messager
            .registerTopicListener(FootstepPlannerMessagerAPI.OccupancyMapTopic, message -> executorService.execute(() -> processOccupancyMapMessage(message)));
      messager.registerTopicListener(FootstepPlannerMessagerAPI.PlanarRegionDataTopic, planarRegionsList::set);
      this.show = messager.createInput(FootstepPlannerMessagerAPI.ShowOccupancyMap, true);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.PlanarRegionDataTopic, data -> reset.set(true));
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ComputePathTopic, data -> reset.set(true));

      cellPolygon.addVertex(cellWidth, 0.0);
      cellPolygon.addVertex(0.5 * cellWidth, 0.5 * Math.sqrt(3.0) * cellWidth);
      cellPolygon.addVertex(-0.5 * cellWidth, 0.5 * Math.sqrt(3.0) * cellWidth);
      cellPolygon.addVertex(-cellWidth, 0.0);
      cellPolygon.addVertex(-0.5 * cellWidth, -0.5 * Math.sqrt(3.0) * cellWidth);
      cellPolygon.addVertex(0.5 * cellWidth, -0.5 * Math.sqrt(3.0) * cellWidth);
      cellPolygon.update();

      root.getChildren().add(footstepGraphMeshView);
   }

   private void processOccupancyMapMessage(FootstepPlannerOccupancyMapMessage message)
   {
      palette.clearPalette();
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);

      Object<FootstepPlannerCellMessage> occupiedCells = message.getOccupiedCells();
      for (int i = 0; i < occupiedCells.size(); i++)
      {
         FootstepPlannerCellMessage cell = occupiedCells.get(i);
         double x = cell.getXIndex() * LatticeNode.gridSizeXY;
         double y = cell.getYIndex() * LatticeNode.gridSizeXY;
         double z = getHeightAtPoint(x, y) + nodeOffsetZ;
         RigidBodyTransform transform = new RigidBodyTransform();
         transform.setTranslation(x, y, z);

         if (cell.getNodeIsValid())
            meshBuilder.addPolygon(transform, cellPolygon, validCellColor);
         else
            meshBuilder.addPolygon(transform, cellPolygon, rejectedCellColor);
      }

      footstepGraphToRender.set(new Pair<>(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
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
         root.getChildren().add(footstepGraphMeshView);
      else if (!show.get() && !root.getChildren().isEmpty())
         root.getChildren().clear();

      if (reset.getAndSet(false))
      {
         footstepGraphToRender.set(null);
         footstepGraphMeshView.setMesh(null);
         footstepGraphMeshView.setMaterial(null);
         return;
      }

      Pair<Mesh, Material> newMesh = footstepGraphToRender.get();
      if (newMesh != null)
      {
         footstepGraphMeshView.setMesh(newMesh.getKey());
         footstepGraphMeshView.setMaterial(newMesh.getValue());
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
