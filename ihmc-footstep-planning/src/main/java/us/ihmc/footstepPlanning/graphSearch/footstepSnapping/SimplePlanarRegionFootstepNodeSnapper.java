package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import java.util.ArrayList;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.robotSide.SideDependentList;

public class SimplePlanarRegionFootstepNodeSnapper extends FootstepNodeSnapper
{
   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private final RigidBodyTransform footstepPose = new RigidBodyTransform();

   public SimplePlanarRegionFootstepNodeSnapper(SideDependentList<ConvexPolygon2D> footPolygons)
   {
      this.footPolygons = footPolygons;
   }

   @Override
   public FootstepNodeSnapData snapInternal(FootstepNode footstepNode)
   {
      if (planarRegionsList == null)
         return FootstepNodeSnapData.emptyData();

      PlanarRegion planarRegionToPack = new PlanarRegion();
      ConvexPolygon2D footPolygon = new ConvexPolygon2D(footPolygons.get(footstepNode.getRobotSide()));
      footstepPose.setRotationYawAndZeroTranslation(footstepNode.getYaw());
      footstepPose.setTranslation(footstepNode.getX(), footstepNode.getY(), 0.0);
      footPolygon.applyTransformAndProjectToXYPlane(footstepPose);
      RigidBodyTransform snapTransform = PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(footPolygon, planarRegionsList, planarRegionToPack);

      if (snapTransform == null)
         return FootstepNodeSnapData.emptyData();

      ArrayList<ConvexPolygon2D> intersections = new ArrayList<>();
      planarRegionToPack.getPolygonIntersectionsWhenProjectedVertically(footPolygon, intersections);
      if (intersections.size() == 0)
         return FootstepNodeSnapData.emptyData();
      else
      {
         return new FootstepNodeSnapData(snapTransform, combineIntersectionPolygons(intersections));
      }
   }

   private static ConvexPolygon2D combineIntersectionPolygons(ArrayList<ConvexPolygon2D> intersections)
   {
      ConvexPolygon2D combinedFootholdIntersection = new ConvexPolygon2D();
      for (int i = 0; i < intersections.size(); i++)
      {
         combinedFootholdIntersection.addVertices(intersections.get(i));
      }
      
      combinedFootholdIntersection.update();
      return combinedFootholdIntersection;
   }
}