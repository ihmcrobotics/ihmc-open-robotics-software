package us.ihmc.footstepPlanning.aStar;

import us.ihmc.robotics.geometry.PlanarRegionsList;

public interface FootstepNodeChecker
{
   public void setPlanarRegions(PlanarRegionsList planarRegions);

   public boolean isNodeValid(FootstepNode node);
}
