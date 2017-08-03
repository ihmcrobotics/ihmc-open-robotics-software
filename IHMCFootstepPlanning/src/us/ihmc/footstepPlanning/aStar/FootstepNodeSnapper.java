package us.ihmc.footstepPlanning.aStar;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public interface FootstepNodeSnapper
{
   void setPlanarRegions(PlanarRegionsList planarRegionsList);

   boolean snapFootstepNode(FootstepNode footstepNode);
}
