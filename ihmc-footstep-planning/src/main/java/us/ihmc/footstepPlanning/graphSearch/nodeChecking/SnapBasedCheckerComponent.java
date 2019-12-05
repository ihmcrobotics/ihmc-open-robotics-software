package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.pathPlanning.graph.structure.DirectedGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public interface SnapBasedCheckerComponent
{
   void setFootstepGraph(DirectedGraph<FootstepNode> graph);

   void setPlanarRegions(PlanarRegionsList planarRegions);

   boolean isNodeValid(FootstepNode nodeToCheck, FootstepNode previousNode);

   BipedalFootstepPlannerNodeRejectionReason getRejectionReason();
}
