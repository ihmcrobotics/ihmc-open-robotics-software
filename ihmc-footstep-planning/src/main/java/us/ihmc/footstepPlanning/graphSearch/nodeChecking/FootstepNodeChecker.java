package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public interface FootstepNodeChecker
{
   public void setPlanarRegions(PlanarRegionsList planarRegions);

   public boolean isNodeValid(FootstepNode node, FootstepNode previosNode);
}
