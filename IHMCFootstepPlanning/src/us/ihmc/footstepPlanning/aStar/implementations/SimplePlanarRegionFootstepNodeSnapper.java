package us.ihmc.footstepPlanning.aStar.implementations;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.aStar.FootstepNode;
import us.ihmc.footstepPlanning.aStar.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;

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
   public RigidBodyTransform snapFootstepNode(FootstepNode footstepNode, ConvexPolygon2D footholdIntersectionToPack)
   {
      if (planarRegionsList == null)
         return null;

      PlanarRegion planarRegionToPack = new PlanarRegion();
      ConvexPolygon2D footPolygon = new ConvexPolygon2D(footPolygons.get(footstepNode.getRobotSide()));
      footstepPose.setRotationYawAndZeroTranslation(footstepNode.getYaw());
      footstepPose.setTranslation(footstepNode.getX(), footstepNode.getY(), 0.0);
      footPolygon.applyTransformAndProjectToXYPlane(footstepPose);
      RigidBodyTransform snapTransform = PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(footPolygon, planarRegionsList, planarRegionToPack);

      if (snapTransform == null)
         return null;

      ArrayList<ConvexPolygon2D> intersections = new ArrayList<>();
      planarRegionToPack.getPolygonIntersectionsWhenProjectedVertically(footPolygon, intersections);
      if (intersections.size() != 1)
         return null;

      if(footholdIntersectionToPack != null)
         footholdIntersectionToPack.set(intersections.get(0));

      return snapTransform;
   }
}