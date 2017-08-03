package us.ihmc.footstepPlanning.aStar.implementations;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.aStar.FootstepNode;
import us.ihmc.footstepPlanning.aStar.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.SideDependentList;

public class SimplePlanarRegionFootstepNodeSnapper implements FootstepNodeSnapper
{
   private PlanarRegionsList planarRegionsList = null;

   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private final RigidBodyTransform footstepPose = new RigidBodyTransform();

   public SimplePlanarRegionFootstepNodeSnapper(SideDependentList<ConvexPolygon2D> footPolygons)
   {
      this.footPolygons = footPolygons;
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   @Override
   public boolean snapFootstepNode(FootstepNode footstepNode)
   {
      if (planarRegionsList == null)
         return false;

      footstepPose.setRotationYawAndZeroTranslation(footstepNode.getYaw());
      footstepPose.setTranslationX(footstepNode.getX());
      footstepPose.setTranslationY(footstepNode.getY());

      ConvexPolygon2D footPolygon = footPolygons.get(footstepNode.getRobotSide());
      RigidBodyTransform worldToSole = new RigidBodyTransform();
      worldToSole.setRotationYawAndZeroTranslation(footstepNode.getYaw());
      worldToSole.setTranslation(footstepNode.getX(), footstepNode.getY(), 0.0);
      footPolygon.applyTransformAndProjectToXYPlane(worldToSole);
      RigidBodyTransform snapTransform = PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(footPolygon, planarRegionsList);

      if(snapTransform == null)
         return false;

      snapTransform.transform(footstepPose);
      footstepNode.setSoleTransform(footstepPose);
      return true;
   }
}
