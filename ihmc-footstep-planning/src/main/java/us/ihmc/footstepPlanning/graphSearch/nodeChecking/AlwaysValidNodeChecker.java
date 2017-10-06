package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class AlwaysValidNodeChecker implements FootstepNodeChecker
{
   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
   }

   @Override
   public boolean isNodeValid(FootstepNode node, FootstepNode previosNode)
   {
      return true;
   }
}
