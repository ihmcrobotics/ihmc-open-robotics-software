package us.ihmc.footstepPlanning.aStar.implementations;

import us.ihmc.footstepPlanning.aStar.FootstepNode;
import us.ihmc.footstepPlanning.aStar.FootstepNodeChecker;
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
