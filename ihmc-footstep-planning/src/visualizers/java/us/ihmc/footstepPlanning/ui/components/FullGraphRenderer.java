package us.ihmc.footstepPlanning.ui.components;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.util.Pair;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.*;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
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
   private static final double nodeOffsetZ = 0.03;
   private static final double cellOpacity = 0.9;
   private static final Color validFootColor = Color.rgb(34, 139, 34, cellOpacity);
   private static final Color parentFootColor = Color.rgb(0, 0, 255, cellOpacity);
   private static final Color rejectedFootColor = Color.rgb(139, 0, 0, cellOpacity);

   private ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final Group root = new Group();

   private final AtomicReference<Pair<Mesh, Material>> footstepGraphToRender = new AtomicReference<>(null);
   private final AtomicReference<Boolean> show;
   private final AtomicReference<Boolean> showRejectedNodes;
   private final AtomicReference<Double> fractionToShow;
   private final AtomicReference<RejectionReasonToVisualize> rejectionReasonToShow;
   private final AtomicBoolean reset = new AtomicBoolean(false);

   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);

   private final PlannerNodeDataList fullGraph = new PlannerNodeDataList();
   private final List<PlannerNodeData> allNodeData = new ArrayList<>();
   private final HashMap<FootstepNode, List<PlannerNodeData>> childMap = new HashMap<>();
   private final ArrayList<FootstepNode> expandedNodes = new ArrayList<>();

   private SideDependentList<ConvexPolygon2D> defaultContactPoints = PlannerTools.createDefaultFootPolygons();

   private final MeshView footstepGraphMeshView = new MeshView();

   public FullGraphRenderer(Messager messager)
   {
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ExpandedNodesMap, latticeMap -> executorService.submit(() -> processExpandedNodes(latticeMap)));
      messager.registerTopicListener(FootstepPlannerMessagerAPI.FootstepGraphPart, nodeDataList -> executorService.submit(() -> processFootstepGraph(nodeDataList)));
      messager.registerTopicListener(FootstepPlannerMessagerAPI.AssumeFlatGround, data -> reset.set(true));
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ComputePath, data -> reset.set(true));
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ExpansionFractionToShow, data -> updateGraphicOnThread());
      messager.registerTopicListener(FootstepPlannerMessagerAPI.RejectionReasonToShow, data -> updateGraphicOnThread());
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ShowRejectedNodes, data -> updateGraphicOnThread());

      show = messager.createInput(FootstepPlannerMessagerAPI.ShowFullGraph, true);
      showRejectedNodes = messager.createInput(FootstepPlannerMessagerAPI.ShowRejectedNodes, true);
      fractionToShow = messager.createInput(FootstepPlannerMessagerAPI.ExpansionFractionToShow, 0.0);
      rejectionReasonToShow = messager.createInput(FootstepPlannerMessagerAPI.RejectionReasonToShow, RejectionReasonToVisualize.ALL);

      root.getChildren().add(footstepGraphMeshView);
   }

   private synchronized void processExpandedNodes(PlannerLatticeMap latticeMap)
   {
      expandedNodes.clear();
      expandedNodes.addAll(latticeMap.getLatticeNodes());
   }

   private synchronized void processFootstepGraph(PlannerNodeDataList nodeDataList)
   {
      fullGraph.clear();
      allNodeData.clear();
      childMap.clear();

      fullGraph.getNodeData().addAll(nodeDataList.getNodeData());
      fullGraph.getNodeData().sort(Comparator.comparingInt(PlannerNodeData::getNodeId));
      allNodeData.addAll(nodeDataList.getNodeData());

      for (PlannerNodeData nodeData : nodeDataList.getNodeData())
      {
         if (nodeData.getParentNodeId() == -1)
            continue;

         FootstepNode parentNode = null;
         for (FootstepNode otherNode : expandedNodes)
         {
            if (nodeData.getParentNodeId() == otherNode.getNodeIndex())
            {
               parentNode = otherNode;
               break;
            }
         }

         childMap.computeIfAbsent(parentNode, node -> new ArrayList<>()).add(nodeData);
      }

      expandedNodes.sort(expansionOrderComparator);
   }

   private void updateGraphicOnThread()
   {
      executorService.submit(this::updateGraphic);
   }

   private synchronized void updateGraphic()
   {
      double alpha = MathTools.clamp(fractionToShow.get(), 0.0, 1.0);

      int frameIndex = (int) (alpha * (expandedNodes.size() - 1));
      FootstepNode nodeToShow = expandedNodes.get(frameIndex);
      Collection<PlannerNodeData> childrenToShow = childMap.get(nodeToShow);
      PlannerNodeData parentToShow = null;
      for (PlannerNodeData nodeData : allNodeData)
      {
         if (nodeData.getFootstepNode().equals(nodeToShow))
            parentToShow = nodeData;
      }

      palette.clearPalette();
      meshBuilder.clear();

      addFoot(parentToShow, parentFootColor);
      for (PlannerNodeData childToShow : childrenToShow)
      {
         if (childToShow.getRejectionReason() == null)
         {
            addFoot(childToShow, validFootColor);
         }
         else if (showRejectedNodes.get())
         {
            if (rejectionReasonToShow.get() != null && rejectionReasonToShow.get().equals(childToShow.getRejectionReason()))
            {
               addFoot(childToShow, rejectedFootColor);
            }
         }
      }

      footstepGraphToRender.set(new Pair<>(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
   }

   private final Comparator<FootstepNode> expansionOrderComparator = (node1, node2) ->
   {
      int earliestIndex1 = getEarliestChildNodeIndex(node1);
      int earliestIndex2 = getEarliestChildNodeIndex(node2);
      return Integer.compare(earliestIndex1, earliestIndex2);
   };

   private int getEarliestChildNodeIndex(FootstepNode footstepNode)
   {
      int smallestIndex = Integer.MAX_VALUE;
      List<PlannerNodeData> childData = childMap.get(footstepNode);
      for (PlannerNodeData child : childData)
         smallestIndex = Math.min(smallestIndex, child.getNodeId());

      return smallestIndex;
   }

   private void addFoot(PlannerNodeData nodeData, Color color)
   {
      RigidBodyTransform transform = new RigidBodyTransform(nodeData.getNodePose());
      transform.getTranslation().addZ(nodeOffsetZ);

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
         root.getChildren().add(footstepGraphMeshView);
      else if (!show.get() && !root.getChildren().isEmpty())
         root.getChildren().clear();

      if (reset.getAndSet(false))
      {
         meshBuilder.clear();
         footstepGraphToRender.set(null);
         footstepGraphMeshView.setMesh(null);
         footstepGraphMeshView.setMaterial(null);

         fullGraph.clear();
         allNodeData.clear();
         childMap.clear();
         expandedNodes.clear();
         return;
      }

      Pair<Mesh, Material> newLatticeMapMesh = footstepGraphToRender.get();
      if (newLatticeMapMesh != null)
      {
         footstepGraphMeshView.setMesh(newLatticeMapMesh.getKey());
         footstepGraphMeshView.setMaterial(newLatticeMapMesh.getValue());
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
