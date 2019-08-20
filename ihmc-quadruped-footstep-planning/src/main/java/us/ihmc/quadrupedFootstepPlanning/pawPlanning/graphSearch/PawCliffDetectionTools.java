package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;

public class PawCliffDetectionTools
{
   public static boolean isNearCliff(PlanarRegionsList planarRegionsList, Point3DReadOnly pawInWorld, double pawYaw, PawPlannerParameters parameters,
                                     double forward, double backward, double left, double right)
   {
      return isNearCliff(planarRegionsList, pawInWorld, pawYaw, parameters.getCliffHeightToAvoid(), forward, backward, left, right);
   }

   public static boolean isNearCliff(PlanarRegionsList planarRegionsList, Point3DReadOnly pawInWorld, double pawYaw, double cliffHeightToAvoid,
                                     double forward, double backward, double left, double right)
   {
      if (!Double.isFinite(cliffHeightToAvoid))
         return true;

      Point3D highestNearbyPoint = new Point3D();
      double maximumCliffZInSoleFrame = findHighestNearbyPoint2(planarRegionsList, pawInWorld, pawYaw, highestNearbyPoint, forward, backward, left, right);

      return maximumCliffZInSoleFrame > cliffHeightToAvoid;
   }

   private static double findHighestNearbyPoint(PlanarRegionsList planarRegionsList, Point3DReadOnly pawInWorld, Point3DBasics highestNearbyPointToPack,
                                                double minimumDistanceFromCliffBottoms)
   {
      double maxZInSoleFrame = Double.NEGATIVE_INFINITY;

      List<PlanarRegion> intersectingRegionsToPack = PlanarRegionTools
            .filterPlanarRegionsWithBoundingCircle(new Point2D(pawInWorld), minimumDistanceFromCliffBottoms, planarRegionsList.getPlanarRegionsAsList());

      for (PlanarRegion intersectingRegion : intersectingRegionsToPack)
      {
         Point3DReadOnly closestPointInWorld = PlanarRegionTools.closestPointOnPlane(pawInWorld, intersectingRegion);

         double heightOfPointFromPaw = closestPointInWorld.getZ() - pawInWorld.getZ();
         double distanceToPoint = pawInWorld.distanceXY(closestPointInWorld);

         if (distanceToPoint < minimumDistanceFromCliffBottoms && heightOfPointFromPaw > maxZInSoleFrame)
         {
            maxZInSoleFrame = heightOfPointFromPaw;
            highestNearbyPointToPack.set(closestPointInWorld);
         }
      }

      return maxZInSoleFrame;
   }

   public static double findHighestNearbyPoint2(PlanarRegionsList planarRegionsList, Point3DReadOnly pawInWorld, double pawYaw,
                                                Point3DBasics highestNearbyPointToPack, double forward, double backward, double left, double right)
   {
      double maxZInSoleFrame = Double.NEGATIVE_INFINITY;

      RigidBodyTransform transformToRegion = new RigidBodyTransform();
      transformToRegion.setRotationYaw(pawYaw);


      ConvexPolygon2D tempPolygon = new ConvexPolygon2D();
      tempPolygon.addVertex(forward, left);
      tempPolygon.addVertex(forward, right);
      tempPolygon.addVertex(backward, left);
      tempPolygon.addVertex(backward, right);
      tempPolygon.update();
      tempPolygon.applyTransform(transformToRegion);
      tempPolygon.translate(pawInWorld.getX(), pawInWorld.getY());

      List<PlanarRegion> intersectingRegions = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(tempPolygon, planarRegionsList.getPlanarRegionsAsList());

      for (PlanarRegion intersectingRegion : intersectingRegions)
      {
         Point3DReadOnly closestPointInWorld = PlanarRegionTools.closestPointOnPlane(pawInWorld, intersectingRegion);

         double heightOfPointFromPaw = closestPointInWorld.getZ() - pawInWorld.getZ();

         if (tempPolygon.isPointInside(closestPointInWorld.getX(), closestPointInWorld.getY()) && heightOfPointFromPaw > maxZInSoleFrame)
         {
            maxZInSoleFrame = heightOfPointFromPaw;
            highestNearbyPointToPack.set(closestPointInWorld);
         }
      }

      return maxZInSoleFrame;
   }
}
