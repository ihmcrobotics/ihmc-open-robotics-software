package us.ihmc.footstepPlanning.aStar.implementations;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.aStar.FootstepNode;
import us.ihmc.footstepPlanning.aStar.FootstepNodeChecker;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.SideDependentList;

public class SnapBasedNodeChecker implements FootstepNodeChecker
{
   private PlanarRegionsList planarRegions;
   private final ConvexPolygon2d footPolygon = new ConvexPolygon2d();
   private final SideDependentList<ConvexPolygon2d> footPolygons;

   public SnapBasedNodeChecker(SideDependentList<ConvexPolygon2d> footPolygons)
   {
      this.footPolygons = footPolygons;
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      this.planarRegions = planarRegions;
   }

   @Override
   public boolean isNodeValid(FootstepNode node)
   {
      if (planarRegions == null)
         return true;

      footPolygon.setAndUpdate(footPolygons.get(node.getRobotSide()));
      RigidBodyTransform worldToSole = new RigidBodyTransform();
      worldToSole.setRotationYawAndZeroTranslation(node.getYaw());
      worldToSole.setTranslation(node.getX(), node.getY(), 0.0);
      footPolygon.applyTransformAndProjectToXYPlane(worldToSole);

      RigidBodyTransform snapTransform = PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(footPolygon, planarRegions);
      if (snapTransform == null)
         return false;

      return true;
   }

}
