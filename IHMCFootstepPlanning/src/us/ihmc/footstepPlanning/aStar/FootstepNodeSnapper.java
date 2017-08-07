package us.ihmc.footstepPlanning.aStar;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public interface FootstepNodeSnapper
{
   void setPlanarRegions(PlanarRegionsList planarRegionsList);

   RigidBodyTransform snapFootstepNode(FootstepNode footstepNode, ConvexPolygon2D footholdIntersectionToPack);
}
