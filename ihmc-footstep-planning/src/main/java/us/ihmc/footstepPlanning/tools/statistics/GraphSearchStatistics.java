package us.ihmc.footstepPlanning.tools.statistics;

import org.jgrapht.Graph;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlannerLatticeMap;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlannerNodeData;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlannerNodeDataList;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;
import us.ihmc.pathPlanning.graph.structure.DirectedGraph;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.pathPlanning.statistics.PlannerStatistics;
import us.ihmc.pathPlanning.statistics.StatisticsType;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;

public class GraphSearchStatistics implements PlannerStatistics<GraphSearchStatistics>
{
   private PlannerLatticeMap expandedNodes;
   private PlannerNodeDataList fullGraph;

   public GraphSearchStatistics()
   {
   }

   public GraphSearchStatistics(GraphSearchStatistics other)
   {
      this();
      set(other);
   }

   public StatisticsType getStatisticsType()
   {
      return StatisticsType.GRAPH_SEARCH;
   }

   public void set(GraphSearchStatistics other)
   {
      setExpandedNodes(other.expandedNodes);
      setFullGraph(other.fullGraph);
   }

   public void setExpandedNodes(PlannerLatticeMap expandedNodes)
   {
      this.expandedNodes = expandedNodes;
   }

   public void setFullGraph(PlannerNodeDataList fullGraph)
   {
      this.fullGraph = fullGraph;
   }

   public PlannerLatticeMap getExpandedNodes()
   {
      return expandedNodes;
   }

   public PlannerNodeDataList getFullGraph()
   {
      return fullGraph;
   }

   public void set(FootstepPlanningModule planningModule)
   {
      PlannerNodeDataList fullGraphList = new PlannerNodeDataList();
      fullGraphList.setIsFootstepGraph(true);

      PlannerLatticeMap expandedNodeMap = new PlannerLatticeMap();

      FootstepPlannerRequest request = planningModule.getRequest();
      Pose3D stanceFootPose = request.getStanceFootPose();
      FootstepNode startNode = new FootstepNode(stanceFootPose.getX(), stanceFootPose.getY(), stanceFootPose.getYaw(), request.getInitialStanceSide());
      SimplePlanarRegionFootstepNodeSnapper snapper = planningModule.getSnapper();
      DirectedGraph<FootstepNode> graph = planningModule.getFootstepPlanner().getGraph();

      RigidBodyTransform startNodePose = snapper.snapFootstepNode(startNode).getOrComputeSnappedNodeTransform(startNode);
      fullGraphList.addNode(-1, startNode, startNodePose, null);
      expandedNodeMap.addFootstepNode(startNode);

      HashMap<FootstepNode, HashSet<GraphEdge<FootstepNode>>> outgoingEdges = graph.getOutgoingEdges();
      for (FootstepNode footstepNode : outgoingEdges.keySet())
      {
         expandedNodeMap.addFootstepNode(footstepNode);
         for (GraphEdge<FootstepNode> outgoingEdge : outgoingEdges.get(footstepNode))
         {
            FootstepNode childNode = outgoingEdge.getEndNode();
            RigidBodyTransform nodePose = snapper.snapFootstepNode(childNode).getOrComputeSnappedNodeTransform(childNode);
            fullGraphList.addNode(footstepNode.getNodeIndex(), childNode, nodePose, null);
         }
      }

      ArrayList<BipedalFootstepPlannerListener> listeners = planningModule.getChecker().getListeners();
      if (!listeners.isEmpty())
      {
         BipedalFootstepPlannerListener listener = listeners.get(0);
         for (FootstepNode footstepNode : listener.getRejectedNodeData().keySet())
         {
            expandedNodeMap.addFootstepNode(footstepNode);
            List<PlannerNodeData> rejectedDataList = listener.getRejectedNodeData().get(footstepNode);
            if (rejectedDataList != null)
            {
               for (PlannerNodeData rejectedData : rejectedDataList)
                  fullGraphList.addNode(rejectedData);
            }
         }
      }

      setExpandedNodes(expandedNodeMap);
      setFullGraph(fullGraphList);
   }
}
