package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.pathPlanning.graph.structure.DirectedGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.function.UnaryOperator;

public interface SnapBasedCheckerComponent
{
   void setParentNodeSupplier(UnaryOperator<FootstepNode> parentNodeSupplier);

   void setPlanarRegions(PlanarRegionsList planarRegions);

   boolean isNodeValid(FootstepNode nodeToCheck, FootstepNode previousNode);

   BipedalFootstepPlannerNodeRejectionReason getRejectionReason();
}
