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
   private static final double minPercentageOfFoothold = 0.95;

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

      PlanarRegion region = new PlanarRegion();

      ConvexPolygon2D footPolygon = new ConvexPolygon2D(footPolygons.get(footstepNode.getRobotSide()));
      footstepPose.setRotationYawAndZeroTranslation(footstepNode.getYaw());
      footstepPose.setTranslation(footstepNode.getX(), footstepNode.getY(), 0.0);
      footPolygon.applyTransformAndProjectToXYPlane(footstepPose);
      RigidBodyTransform snapTransform = PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(footPolygon, planarRegionsList, region);

      if (snapTransform == null)
         return false;

      ArrayList<ConvexPolygon2D> intersections = new ArrayList<>();
      region.getPolygonIntersectionsWhenProjectedVertically(footPolygon, intersections);
      if (intersections.size() != 1)
         return false;

      double area = intersections.get(0).getArea();
      double footArea = footPolygon.getArea();
      if (area < minPercentageOfFoothold * footArea)
         return false;

      snapTransform.transform(footstepPose);
      footstepNode.setSoleTransform(footstepPose);
      return true;
   }
}