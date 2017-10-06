package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public interface FootstepNodeSnapper
{
   void setPlanarRegions(PlanarRegionsList planarRegionsList);

   RigidBodyTransform snapFootstepNode(FootstepNode footstepNode, ConvexPolygon2D footholdIntersectionToPack);
}
