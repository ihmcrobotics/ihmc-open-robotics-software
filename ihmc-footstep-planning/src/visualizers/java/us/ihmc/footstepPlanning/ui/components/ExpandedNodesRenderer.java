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
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlannerCell;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlannerLatticeMap;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlannerOccupancyMap;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import java.util.ArrayList;
import java.util.Collection;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class ExpandedNodesRenderer extends AnimationTimer
{
   private static final double nodeOffsetZ = 0.05;
   private static final double cellOpacity = 0.9;
   private static final Color validFootColor = Color.rgb(219, 62, 87, cellOpacity);
   private static final Color rejectedCellColor = Color.rgb(139, 0, 0, cellOpacity);
   private static final Color leftFootColor = Color.rgb(0, 204, 0, cellOpacity);
   private static final Color rightFootColor = Color.rgb(255, 0, 0, cellOpacity);

   private ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final Group root = new Group();

   private final AtomicReference<Pair<Mesh, Material>> latticeMapToRender = new AtomicReference<>(null);
   private final AtomicReference<PlanarRegionsList> planarRegionsList = new AtomicReference<>(null);
   private final AtomicReference<Boolean> show;
   private final AtomicBoolean reset = new AtomicBoolean(false);

   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);

   private SideDependentList<ConvexPolygon2D> defaultContactPoints = PlannerTools.createDefaultFootPolygons();

   private final MeshView latticeMapMeshView = new MeshView();

   public ExpandedNodesRenderer(Messager messager)
   {
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ExpandedNodesMap, latticeMap -> executorService.submit(() -> processExpandedNodes(latticeMap)));
      messager.registerTopicListener(FootstepPlannerMessagerAPI.PlanarRegionData, planarRegionsList::set);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.AssumeFlatGround, data -> reset.set(true));
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ComputePath, data -> reset.set(true));
      show = messager.createInput(FootstepPlannerMessagerAPI.ShowExpandedNodes, true);

      root.getChildren().add(latticeMapMeshView);
   }

   private synchronized void processExpandedNodes(PlannerLatticeMap latticeMap)
   {
      palette.clearPalette();

      Collection<FootstepNode> latticeNodes = latticeMap.getLatticeNodes();
      for (FootstepNode node : latticeNodes)
      {
         double x = node.getX();
         double y = node.getY();
         double z = getHeightAtPoint(x, y) + nodeOffsetZ;
         RigidBodyTransform transform = new RigidBodyTransform();
         transform.setTranslation(x, y, z);
         transform.setRotationYaw(node.getYaw());

         Point2D[] vertices = new Point2D[defaultContactPoints.get(RobotSide.LEFT).getNumberOfVertices()];
         for (int j = 0; j < vertices.length; j++)
            vertices[j] = new Point2D(defaultContactPoints.get(RobotSide.LEFT).getVertex(j));

         meshBuilder.addMultiLine(transform, vertices, 0.01, validFootColor, true);
         meshBuilder.addPolygon(transform, defaultContactPoints.get(RobotSide.LEFT), validFootColor);
      }

      latticeMapToRender.set(new Pair<>(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
   }

   private double getHeightAtPoint(double x, double y)
   {
      PlanarRegionsList planarRegionsList = this.planarRegionsList.get();
      if (planarRegionsList == null)
         return 0.0;
      Point3DReadOnly projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(new Point3D(x, y, 100.0), planarRegionsList);
      return projectedPoint == null ? 0.0 : projectedPoint.getZ();
   }

   public void setDefaultContactPoints(RobotContactPointParameters<RobotSide> contactPointParameters)
   {
      defaultContactPoints = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
      {
         ArrayList<Point2D> footPoints = contactPointParameters.getFootContactPoints().get(side);
         ConvexPolygon2D scaledFoot = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPoints));
         defaultContactPoints.set(side, scaledFoot);
      }
   }

   @Override
   public void handle(long now)
   {
      if (show.get() && root.getChildren().isEmpty())
         root.getChildren().add(latticeMapMeshView);
      else if (!show.get() && !root.getChildren().isEmpty())
         root.getChildren().clear();

      if (reset.getAndSet(false))
      {
         meshBuilder.clear();
         latticeMapToRender.set(null);
         latticeMapMeshView.setMesh(null);
         latticeMapMeshView.setMaterial(null);
         return;
      }

      Pair<Mesh, Material> newLatticeMapMesh = latticeMapToRender.get();
      if (newLatticeMapMesh != null)
      {
         latticeMapMeshView.setMesh(newLatticeMapMesh.getKey());
         latticeMapMeshView.setMaterial(newLatticeMapMesh.getValue());
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
