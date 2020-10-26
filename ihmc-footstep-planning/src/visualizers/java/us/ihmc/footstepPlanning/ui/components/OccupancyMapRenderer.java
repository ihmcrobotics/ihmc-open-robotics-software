package us.ihmc.footstepPlanning.ui.components;

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
import us.ihmc.footstepPlanning.graphSearch.graph.LatticePoint;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlannerCell;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlannerOccupancyMap;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.Collection;
import java.util.HashSet;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class OccupancyMapRenderer extends AnimationTimer
{
   private static final double cellWidth = 0.02;
   private static final double nodeOffsetZ = 0.05;
   private static final double cellOpacity = 0.5;
   private static final Color occupancyCellColor = Color.rgb(219, 124, 87, cellOpacity);

   private ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final Group root = new Group();

   private final AtomicReference<Pair<Mesh, Material>> occupancyMapToRender = new AtomicReference<>(null);
   private final AtomicReference<PlanarRegionsList> planarRegionsList = new AtomicReference<>(null);
   private final AtomicReference<Boolean> show;
   private final AtomicBoolean reset = new AtomicBoolean(false);

   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);

   private final HashSet<PlannerCell> renderedCells = new HashSet<>();
   private final MeshView occupancyMapMeshView = new MeshView();
   private final ConvexPolygon2D cellPolygon = new ConvexPolygon2D();

   public OccupancyMapRenderer(Messager messager)
   {
      messager.registerTopicListener(FootstepPlannerMessagerAPI.OccupancyMap, occupancyMap -> executorService.execute(() -> processOccupancyMap(occupancyMap)));
      messager.registerTopicListener(FootstepPlannerMessagerAPI.PlanarRegionData, planarRegionsList::set);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.AssumeFlatGround, data -> reset.set(true));
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ComputePath, data -> reset.set(true));
      messager.registerTopicListener(FootstepPlannerMessagerAPI.GlobalReset, data -> reset.set(true));
      show = messager.createInput(FootstepPlannerMessagerAPI.ShowOccupancyMap, true);

      cellPolygon.addVertex(cellWidth, 0.0);
      cellPolygon.addVertex(0.5 * cellWidth, 0.5 * Math.sqrt(3.0) * cellWidth);
      cellPolygon.addVertex(-0.5 * cellWidth, 0.5 * Math.sqrt(3.0) * cellWidth);
      cellPolygon.addVertex(-cellWidth, 0.0);
      cellPolygon.addVertex(-0.5 * cellWidth, -0.5 * Math.sqrt(3.0) * cellWidth);
      cellPolygon.addVertex(0.5 * cellWidth, -0.5 * Math.sqrt(3.0) * cellWidth);
      cellPolygon.update();

      root.getChildren().add(occupancyMapMeshView);
   }

   private void processOccupancyMap(PlannerOccupancyMap occupancyMap)
   {
      palette.clearPalette();

      Collection<PlannerCell> occupiedCells = occupancyMap.getOccupiedCells();
      for (PlannerCell cell : occupiedCells)
      {
         if (renderedCells.contains(cell))
            continue;

         double x = cell.getXIndex() * LatticePoint.gridSizeXY;
         double y = cell.getYIndex() * LatticePoint.gridSizeXY;
         double z = getHeightAtPoint(x, y) + nodeOffsetZ;
         RigidBodyTransform transform = new RigidBodyTransform();
         transform.getTranslation().set(x, y, z);

         renderedCells.add(cell);
         meshBuilder.addPolygon(transform, cellPolygon, occupancyCellColor);
      }

      occupancyMapToRender.set(new Pair<>(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
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
         root.getChildren().add(occupancyMapMeshView);
      else if (!show.get() && !root.getChildren().isEmpty())
         root.getChildren().clear();

      if (reset.getAndSet(false))
      {
         meshBuilder.clear();
         occupancyMapToRender.set(null);
         occupancyMapMeshView.setMesh(null);
         occupancyMapMeshView.setMaterial(null);
         renderedCells.clear();
         return;
      }

      Pair<Mesh, Material> newOccupancyMapMesh = occupancyMapToRender.getAndSet(null);
      if (newOccupancyMapMesh != null)
      {
         occupancyMapMeshView.setMesh(newOccupancyMapMesh.getKey());
         occupancyMapMeshView.setMaterial(newOccupancyMapMesh.getValue());
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
