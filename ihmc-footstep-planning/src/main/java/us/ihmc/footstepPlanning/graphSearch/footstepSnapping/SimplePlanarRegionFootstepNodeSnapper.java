package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import java.util.ArrayList;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerNodeUtils;
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
      footPolygon.applyTransformAndProjectToXYPlane(snapTransform);
      planarRegionToPack.getPolygonIntersectionsWhenProjectedVertically(footPolygon, intersections);
      if (intersections.size() == 0)
         return FootstepNodeSnapData.emptyData();
      else
      {
         ConvexPolygon2D allIntersections = combineIntersectionPolygons(intersections);
         RigidBodyTransform regionToWorld = new RigidBodyTransform();
         planarRegionToPack.getTransformToWorld(regionToWorld);

         RigidBodyTransform soleTransform = BipedalFootstepPlannerNodeUtils.getSnappedSoleTransform(footstepNode, snapTransform);

         RigidBodyTransform regionToSole = new RigidBodyTransform();
         regionToSole.setAndInvert(soleTransform);
         regionToSole.multiply(regionToWorld);

         allIntersections.applyTransformAndProjectToXYPlane(regionToSole);
         return new FootstepNodeSnapData(snapTransform, allIntersections);
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