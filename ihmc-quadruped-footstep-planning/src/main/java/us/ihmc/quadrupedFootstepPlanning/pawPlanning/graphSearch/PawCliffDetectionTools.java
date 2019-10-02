package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionTools;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;

public class PawCliffDetectionTools
{
   public static boolean isNearCliff(PlanarRegionsList planarRegionsList, Point3DReadOnly pawInWorld, double pawYaw, PawStepPlannerParametersReadOnly parameters,
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
      double maximumCliffZInSoleFrame = findHighestNearbyPoint(planarRegionsList, pawInWorld, pawYaw, highestNearbyPoint, forward, backward, left, right);

      return maximumCliffZInSoleFrame > cliffHeightToAvoid;
   }

   public static PlanarRegion findHighestNearbyRegion(PlanarRegionsList planarRegionsList, Point3DReadOnly pawInWorld, Point3DBasics highestNearbyPointToPack,
                                                      double minimumDistanceFromCliffBottoms)
   {
      PlanarRegion highestRegion = null;
      highestNearbyPointToPack.setZ( Double.NEGATIVE_INFINITY);

      List<PlanarRegion> intersectingRegionsToPack = PlanarRegionTools
            .filterPlanarRegionsWithBoundingCircle(new Point2D(pawInWorld), minimumDistanceFromCliffBottoms, planarRegionsList.getPlanarRegionsAsList());

      for (PlanarRegion intersectingRegion : intersectingRegionsToPack)
      {
         Point3DReadOnly closestPointInWorld = PlanarRegionTools.closestPointOnPlane(pawInWorld, intersectingRegion);

         double heightOfPointFromPaw = closestPointInWorld.getZ() - pawInWorld.getZ();
         double distanceToPoint = pawInWorld.distanceXY(closestPointInWorld);

         if (distanceToPoint < minimumDistanceFromCliffBottoms && heightOfPointFromPaw > highestNearbyPointToPack.getZ())
         {
            highestRegion = intersectingRegion;
            highestNearbyPointToPack.set(closestPointInWorld);
         }
      }

      return highestRegion;
   }

   public static double findHighestNearbyPoint(PlanarRegionsList planarRegionsList, Point3DReadOnly pawInWorld, double pawYaw,
                                               Point3DBasics highestNearbyPointToPack, double forward, double backward, double left, double right)
   {
      RigidBodyTransform transformToRegion = new RigidBodyTransform();
      transformToRegion.setRotationYaw(pawYaw);

      ConvexPolygon2D avoidanceRegion = new ConvexPolygon2D();
      avoidanceRegion.addVertex(forward, left);
      avoidanceRegion.addVertex(forward, right);
      avoidanceRegion.addVertex(backward, left);
      avoidanceRegion.addVertex(backward, right);
      avoidanceRegion.update();
      avoidanceRegion.applyTransform(transformToRegion);
      avoidanceRegion.translate(pawInWorld.getX(), pawInWorld.getY());

      return findHighestNearbyPoint(planarRegionsList, pawInWorld, highestNearbyPointToPack, avoidanceRegion);

   }

   public static double findHighestNearbyPoint(PlanarRegionsList planarRegionsList, Point3DReadOnly pawInWorld, Point3DBasics highestNearbyPointToPack,
                                               ConvexPolygon2D avoidanceRegion)
   {
      double maxZInSoleFrame = Double.NEGATIVE_INFINITY;


      List<PlanarRegion> intersectingRegions = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(avoidanceRegion, planarRegionsList.getPlanarRegionsAsList());

      for (PlanarRegion intersectingRegion : intersectingRegions)
      {
         Point3DReadOnly closestPointInWorld = PlanarRegionTools.closestPointOnPlane(pawInWorld, intersectingRegion);

         double heightOfPointFromPaw = closestPointInWorld.getZ() - pawInWorld.getZ();

         if (avoidanceRegion.isPointInside(closestPointInWorld.getX(), closestPointInWorld.getY()) && heightOfPointFromPaw > maxZInSoleFrame)
         {
            maxZInSoleFrame = heightOfPointFromPaw;
            highestNearbyPointToPack.set(closestPointInWorld);
         }
      }

      return maxZInSoleFrame;
   }
}
