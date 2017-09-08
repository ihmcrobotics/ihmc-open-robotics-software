package us.ihmc.footstepPlanning.aStar.implementations;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.aStar.FootstepNode;
import us.ihmc.footstepPlanning.aStar.FootstepNodeSnapper;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FlatGroundFootstepNodeSnapper implements FootstepNodeSnapper
{
   private final SideDependentList<ConvexPolygon2D> footPolygons;

   public FlatGroundFootstepNodeSnapper(SideDependentList<ConvexPolygon2D> footPolygons)
   {
      this.footPolygons = footPolygons;
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
   }

   @Override
   public RigidBodyTransform snapFootstepNode(FootstepNode footstepNode, ConvexPolygon2D footholdIntersectionToPack)
   {
      if(footholdIntersectionToPack != null)
         footholdIntersectionToPack.set(footPolygons.get(footstepNode.getRobotSide()));
      return new RigidBodyTransform();
   }
}
