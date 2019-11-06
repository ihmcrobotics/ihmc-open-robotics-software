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
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlannerLatticeMap;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlannerNodeData;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlannerNodeDataList;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import java.util.*;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class FullGraphRenderer extends AnimationTimer
{
   private static final double cellOpacity = 0.9;
   private static final Color validFootColor = Color.rgb(0, 255, 0, cellOpacity);
   private static final Color parentFootColor = Color.rgb(0, 0, 255, cellOpacity);
   private static final Color rejectedFootColor = Color.rgb(139, 0, 0, cellOpacity);

   private ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final Group root = new Group();

   private final AtomicReference<Pair<Mesh, Material>> latticeMapToRender = new AtomicReference<>(null);
   private final AtomicReference<Boolean> show;
   private final AtomicReference<Integer> stepToShow;
   private final AtomicReference<BipedalFootstepPlannerNodeRejectionReason> rejectionReasonToShow;
   private final AtomicBoolean reset = new AtomicBoolean(false);

   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);

   private final PlannerNodeDataList fullGraph = new PlannerNodeDataList();
   private final HashMap<LatticeNode, HashSet<PlannerNodeData>> childMap = new HashMap<>();
   private final ArrayList<LatticeNode> expandedNodes = new ArrayList<>();

   private SideDependentList<ConvexPolygon2D> defaultContactPoints = PlannerTools.createDefaultFootPolygons();

   private final MeshView latticeMapMeshView = new MeshView();

   public FullGraphRenderer(Messager messager)
   {
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ExpandedNodesMap, latticeMap -> executorService.execute(() -> processExpandedNodes(latticeMap)));
      messager.registerTopicListener(FootstepPlannerMessagerAPI.FootstepGraphPart, nodeDataList -> executorService.execute(() -> processFootstepGraph(nodeDataList)));
      messager.registerTopicListener(FootstepPlannerMessagerAPI.AssumeFlatGround, data -> reset.set(true));
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ComputePath, data -> reset.set(true));
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ExpansionStepToShow, data -> updateGraphic());
      messager.registerTopicListener(FootstepPlannerMessagerAPI.RejectionReasonToShow, data -> updateGraphic());

      show = messager.createInput(FootstepPlannerMessagerAPI.ShowFullGraph, true);
      stepToShow = messager.createInput(FootstepPlannerMessagerAPI.ExpansionStepToShow, -1);
      rejectionReasonToShow = messager.createInput(FootstepPlannerMessagerAPI.RejectionReasonToShow, null);

      root.getChildren().add(latticeMapMeshView);
   }

   private void processExpandedNodes(PlannerLatticeMap latticeMap)
   {
      expandedNodes.addAll(latticeMap.getLatticeNodes());
   }

   private void processFootstepGraph(PlannerNodeDataList nodeDataList)
   {
      fullGraph.getNodeData().addAll(nodeDataList.getNodeData());
      fullGraph.getNodeData().sort(Comparator.comparingInt(PlannerNodeData::getNodeId));

      for (PlannerNodeData nodeData : nodeDataList.getNodeData())
      {
         if (nodeData.getParentNodeId() == -1)
            continue;

         PlannerNodeData parentNode = fullGraph.getNodeData().get(nodeData.getParentNodeId());
         childMap.computeIfAbsent(parentNode.getLatticeNode(), data -> new HashSet<>()).add(nodeData);
      }
   }

   private void updateGraphic()
   {
      LatticeNode nodeToShow = expandedNodes.get(stepToShow.get());
      Collection<PlannerNodeData> childrenToShow = childMap.get(nodeToShow);
      PlannerNodeData parentToShow = fullGraph.getDataForNode(nodeToShow);

      palette.clearPalette();
      meshBuilder.clear();

      addFoot(parentToShow, parentFootColor);
      for (PlannerNodeData childToShow : childrenToShow)
      {
         if (childToShow.getRejectionReason() == null)
         {
            addFoot(childToShow, validFootColor);
         }
         else if (rejectionReasonToShow.get() == null)
         {
            addFoot(childToShow, rejectedFootColor);
         }
         else if (rejectionReasonToShow.get() == childToShow.getRejectionReason())
         {
            addFoot(childToShow, rejectedFootColor);
         }
      }

   }

   private void addFoot(PlannerNodeData nodeData, Color color)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      nodeData.getNodePose().get(transform);

      Point2D[] vertices = new Point2D[defaultContactPoints.get(RobotSide.LEFT).getNumberOfVertices()];
      for (int j = 0; j < vertices.length; j++)
         vertices[j] = new Point2D(defaultContactPoints.get(RobotSide.LEFT).getVertex(j));

      meshBuilder.addMultiLine(transform, vertices, 0.01, color, true);
      meshBuilder.addPolygon(transform, defaultContactPoints.get(RobotSide.LEFT), color);
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

         fullGraph.clear();
         childMap.clear();
         expandedNodes.clear();
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
