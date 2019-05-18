package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.List;

public class CliffDetectionTools
{
   public static boolean isNearCliff(RobotQuadrant robotQuadrant, PlanarRegionsList planarRegionsList, Point3DReadOnly footInWorld, double footYaw,
                                      FootstepPlannerParameters parameters)
   {
      double cliffHeightToAvoid = parameters.getCliffHeightToAvoid();
      if (!Double.isFinite(cliffHeightToAvoid))
         return true;

      Point3D highestNearbyPoint = new Point3D();
      double maximumCliffZInSoleFrame = findHighestNearbyPoint2(robotQuadrant, planarRegionsList, footInWorld, footYaw, highestNearbyPoint, parameters);

      return maximumCliffZInSoleFrame > cliffHeightToAvoid;
   }

   private static double findHighestNearbyPoint(PlanarRegionsList planarRegionsList, Point3DReadOnly footInWorld, Point3DBasics highestNearbyPointToPack,
                                                double minimumDistanceFromCliffBottoms)
   {
      double maxZInSoleFrame = Double.NEGATIVE_INFINITY;

      List<PlanarRegion> intersectingRegionsToPack = PlanarRegionTools
            .filterPlanarRegionsWithBoundingCircle(new Point2D(footInWorld), minimumDistanceFromCliffBottoms, planarRegionsList.getPlanarRegionsAsList());

      for (PlanarRegion intersectingRegion : intersectingRegionsToPack)
      {
         Point3DReadOnly closestPointInWorld = PlanarRegionTools.closestPointOnPlane(footInWorld, intersectingRegion);

         double heightOfPointFromFoot = closestPointInWorld.getZ() - footInWorld.getZ();
         double distanceToPoint = footInWorld.distanceXY(closestPointInWorld);

         if (distanceToPoint < minimumDistanceFromCliffBottoms && heightOfPointFromFoot > maxZInSoleFrame)
         {
            maxZInSoleFrame = heightOfPointFromFoot;
            highestNearbyPointToPack.set(closestPointInWorld);
         }
      }

      return maxZInSoleFrame;
   }

   public static double findHighestNearbyPoint2(RobotQuadrant robotQuadrant, PlanarRegionsList planarRegionsList, Point3DReadOnly footInWorld, double footYaw,
                                                 Point3DBasics highestNearbyPointToPack, FootstepPlannerParameters parameters)
   {
      double maxZInSoleFrame = Double.NEGATIVE_INFINITY;

      RigidBodyTransform transformToRegion = new RigidBodyTransform();
      transformToRegion.setRotationYaw(footYaw);

      double forward = robotQuadrant.isQuadrantInFront() ?
            parameters.getMinimumFrontEndForwardDistanceFromCliffBottoms() :
            parameters.getMinimumHindEndForwardDistanceFromCliffBottoms();
      double backward = robotQuadrant.isQuadrantInFront() ?
            parameters.getMinimumFrontEndBackwardDistanceFromCliffBottoms() :
            parameters.getMinimumHindEndBackwardDistanceFromCliffBottoms();
      double lateral = parameters.getMinimumLateralDistanceFromCliffBottoms();

      ConvexPolygon2D tempPolygon = new ConvexPolygon2D();
      tempPolygon.addVertex(forward, lateral);
      tempPolygon.addVertex(forward, -lateral);
      tempPolygon.addVertex(-backward, lateral);
      tempPolygon.addVertex(-backward, -lateral);
      tempPolygon.update();
      tempPolygon.applyTransform(transformToRegion);
      tempPolygon.translate(footInWorld.getX(), footInWorld.getY());

      List<PlanarRegion> intersectingRegions = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(tempPolygon, planarRegionsList.getPlanarRegionsAsList());

      for (PlanarRegion intersectingRegion : intersectingRegions)
      {
         Point3DReadOnly closestPointInWorld = PlanarRegionTools.closestPointOnPlane(footInWorld, intersectingRegion);

         double heightOfPointFromFoot = closestPointInWorld.getZ() - footInWorld.getZ();

         if (tempPolygon.isPointInside(closestPointInWorld.getX(), closestPointInWorld.getY()))
         {
            maxZInSoleFrame = heightOfPointFromFoot;
            highestNearbyPointToPack.set(closestPointInWorld);
         }
      }

      return maxZInSoleFrame;
   }
}
