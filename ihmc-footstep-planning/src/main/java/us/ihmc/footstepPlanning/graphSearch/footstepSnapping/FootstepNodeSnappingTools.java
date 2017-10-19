package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.ArrayList;

public class FootstepNodeSnappingTools
{
   /**
    * Computes the convex hull of the intersections of footPolygonInWorldFrame when snapped to planarRegion using
    * snapTransform
    *
    * @param planarRegion
    * @param footPolygon foot polygon in world frame
    * @param snapTransform
    * @return intersection polygon in region frame
    */
   public static ConvexPolygon2D getConvexHullOfPolygonIntersections(PlanarRegion planarRegion, ConvexPolygon2D footPolygon,
                                                                     RigidBodyTransform snapTransform)
   {
      ArrayList<ConvexPolygon2D> intersections = new ArrayList<>();
      footPolygon.applyTransformAndProjectToXYPlane(snapTransform);
      planarRegion.getPolygonIntersectionsWhenProjectedVertically(footPolygon, intersections);
      return getConvexHull(intersections);
   }

   /**
    * Computes the convex hull of the given list of intersections. Returns an empty ConvexPolygon2D
    * if the list is empty
    *
    * @param intersections
    * @return
    */
   private static ConvexPolygon2D getConvexHull(ArrayList<ConvexPolygon2D> intersections)
   {
      ConvexPolygon2D combinedFootholdIntersection = new ConvexPolygon2D();
      for (int i = 0; i < intersections.size(); i++)
      {
         combinedFootholdIntersection.addVertices(intersections.get(i));
      }

      combinedFootholdIntersection.update();
      return combinedFootholdIntersection;
   }

   /**
    * Transforms footPolygonInRegionFrame from planarRegion frame to the snapped footstep node frame
    *
    * @param planarRegion
    * @param footstepNode
    * @param snapTransform
    * @param footPolygonInRegionFrame
    */
   public static void changeFromPlanarRegionToSoleFrame(PlanarRegion planarRegion, FootstepNode footstepNode, RigidBodyTransform snapTransform,
                                                        ConvexPolygon2D footPolygonInRegionFrame)
   {
      RigidBodyTransform regionToWorld = new RigidBodyTransform();
      planarRegion.getTransformToWorld(regionToWorld);

      RigidBodyTransform soleTransform = new RigidBodyTransform();
      FootstepNodeTools.getSnappedNodeTransform(footstepNode, snapTransform, soleTransform);

      RigidBodyTransform regionToSole = new RigidBodyTransform();
      regionToSole.setAndInvert(soleTransform);
      regionToSole.multiply(regionToWorld);

      footPolygonInRegionFrame.applyTransformAndProjectToXYPlane(regionToSole);
   }
}
