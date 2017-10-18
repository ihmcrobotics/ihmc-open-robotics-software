package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import java.util.ArrayList;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
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
      PlanarRegion planarRegionToPack = new PlanarRegion();
      ConvexPolygon2D footPolygon = new ConvexPolygon2D();
      FootstepNodeTools.getFootPolygon(footstepNode, footPolygons.get(footstepNode.getRobotSide()), footPolygon);

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

         RigidBodyTransform soleTransform = new RigidBodyTransform();
         FootstepNodeTools.getSnappedNodeTransform(footstepNode, snapTransform, soleTransform);

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