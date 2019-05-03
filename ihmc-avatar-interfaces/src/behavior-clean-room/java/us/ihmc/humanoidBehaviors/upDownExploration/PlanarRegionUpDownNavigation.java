package us.ihmc.humanoidBehaviors.upDownExploration;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.*;

public class PlanarRegionUpDownNavigation
{
   private static final double LEVEL_EPSILON = 1e-3;
   private static final double HIGH_LOW_MINIMUM = 0.10;
   private static final double MINIMUM_AREA = 0.5 * 0.5; // square half meter
   public static final double REQUIRED_FLAT_AREA_RADIUS = 0.35;
   public static final double MAX_NAVIGATION_DISTANCE = 10.0;
   public static final double CHECK_STEP_SIZE = REQUIRED_FLAT_AREA_RADIUS;

   public enum NavigationResult
   {
      WAYPOINT_FOUND,
      NO_LEVEL_REGIONS,
      NO_HIGHER_REGIONS,
      NO_LARGE_REGIONS,
      NO_QUALIFIED_REGIONS,
      TOO_MANY_QUALIFIED_REGIONS
   }

   public static Pair<NavigationResult, FramePose3D> up(ReferenceFrame midFeetZUpFrame, PlanarRegionsList planarRegionsList)
   {
      FramePose3D midFeetZUpPose = new FramePose3D();
      midFeetZUpPose.setFromReferenceFrame(midFeetZUpFrame);

      // TODO extract these filters into a method
      TreeSet<PlanarRegion> levelRegions = new TreeSet<>(Comparator.comparingInt(PlanarRegion::getRegionId));
      for (PlanarRegion planarRegion : planarRegionsList.getPlanarRegionsAsList())
      {
         if (isLevel(planarRegion, LEVEL_EPSILON))
         {
            levelRegions.add(planarRegion);
         }
      }
      LogTools.debug("Number of level regions: {}", levelRegions.size());
      if (levelRegions.isEmpty())
      {
         return Pair.of(NavigationResult.NO_LEVEL_REGIONS, null);
      }

      TreeSet<PlanarRegion> higherRegions = new TreeSet<>(Comparator.comparingInt(PlanarRegion::getRegionId));
      for (PlanarRegion planarRegion : planarRegionsList.getPlanarRegionsAsList())
      {
         if (centroid(planarRegion).getZ() > midFeetZUpPose.getZ() + HIGH_LOW_MINIMUM)
         {
            higherRegions.add(planarRegion);
         }
      }
      LogTools.debug("Number of higher regions: {}", higherRegions.size());
      if (higherRegions.isEmpty())
      {
         return Pair.of(NavigationResult.NO_HIGHER_REGIONS, null);
      }

      TreeSet<PlanarRegion> largeRegions = new TreeSet<>(Comparator.comparingInt(PlanarRegion::getRegionId));
      for (PlanarRegion planarRegion : planarRegionsList.getPlanarRegionsAsList())
      {
         if (area(planarRegion) > MINIMUM_AREA)
         {
            largeRegions.add(planarRegion);
         }
      }
      LogTools.debug("Number of large regions: {}", higherRegions.size());
      if (largeRegions.isEmpty())
      {
         return Pair.of(NavigationResult.NO_LARGE_REGIONS, null);
      }

      TreeSet<PlanarRegion> candidateRegions = new TreeSet<>(Comparator.comparingInt(PlanarRegion::getRegionId));
      for (PlanarRegion planarRegion : planarRegionsList.getPlanarRegionsAsList())
      {
         if (levelRegions.contains(planarRegion) && higherRegions.contains(planarRegion) && largeRegions.contains(planarRegion))
         {
            candidateRegions.add(planarRegion);
         }
      }
      LogTools.debug("Number of candidateRegions regions: {}", candidateRegions.size());
      if (candidateRegions.isEmpty())
      {
         return Pair.of(NavigationResult.NO_QUALIFIED_REGIONS, null);
      }
      if (candidateRegions.size() > 1)
      {
         return Pair.of(NavigationResult.TOO_MANY_QUALIFIED_REGIONS, null);
      }

      FramePose3D waypointPose = new FramePose3D();
      waypointPose.setFromReferenceFrame(midFeetZUpFrame);
      waypointPose.setPosition(centroid(candidateRegions.first()));

      return Pair.of(NavigationResult.WAYPOINT_FOUND, waypointPose);
   }

   public static Pair<NavigationResult, FramePose3D> upOrDown(ReferenceFrame midFeetZUpFrame, PlanarRegionsList planarRegionsList)
   {
      FramePose3D midFeetZUpPose = new FramePose3D();
      midFeetZUpPose.setFromReferenceFrame(midFeetZUpFrame);

      // move straight out from mid feet z up until polygon points share their highest collisions with the same region
      PolygonPoints2D polygonPoints = new PolygonPoints2D(5, REQUIRED_FLAT_AREA_RADIUS, midFeetZUpFrame);

      polygonPoints.add(2 * CHECK_STEP_SIZE, 0.0); // initial check step a bit out
      LogTools.info("Polygon center point {}", polygonPoints.getCenterPoint());

      FramePoint3D centerPoint;
      while ((centerPoint = centerPointOfPolygonWhenPointsShareHighestCollisionsWithSameRegion(polygonPoints, planarRegionsList, midFeetZUpPose.getZ())) == null
            && polygonPoints.getCenterPoint().getX() < MAX_NAVIGATION_DISTANCE)
      {
         polygonPoints.add(CHECK_STEP_SIZE, 0.0);
         LogTools.info("Stepping virtual polygon points forward by {}", CHECK_STEP_SIZE);
         LogTools.info("Polygon center point {}", polygonPoints.getCenterPoint());
      }

      if (centerPoint != null)
      {
         FramePose3D waypointPose = new FramePose3D();
         waypointPose.setFromReferenceFrame(midFeetZUpFrame);
         waypointPose.setPosition(centerPoint);

         LogTools.debug("Qualifying pose found at height {}: {}", centerPoint.getZ() - midFeetZUpPose.getZ(), waypointPose);

         return Pair.of(NavigationResult.WAYPOINT_FOUND, waypointPose);
      }

      return Pair.of(NavigationResult.NO_QUALIFIED_REGIONS, null);
   }

   public static FramePoint3D centerPointOfPolygonWhenPointsShareHighestCollisionsWithSameRegion(PolygonPoints2D polygonPoints,
                                                                                                 PlanarRegionsList planarRegionsList,
                                                                                                 double startingZ)
   {
      // map points to their collisions
      HashMap<FramePoint2D, TreeSet<Pair<PlanarRegion, Double>>> collisions = new HashMap<>();
      for (FramePoint2D point : polygonPoints.getPoints())
      {
         ReferenceFrame previousFrame = point.getReferenceFrame();
         point.changeFrame(ReferenceFrame.getWorldFrame());
         { // point frame changed block
            TreeSet<Pair<PlanarRegion, Double>> singlePointCollisions = new TreeSet<>(Comparator.comparingDouble(o -> o.getRight()));
            List<PlanarRegion> collidedRegions = planarRegionsList.findPlanarRegionsContainingPointByVerticalLineIntersection(point);

            if (collidedRegions == null || collidedRegions.isEmpty())
            {
               return null; // not every point collides once
            }

            for (PlanarRegion collidedRegion : collidedRegions)
            {
               singlePointCollisions.add(Pair.of(collidedRegion, collidedRegion.getPlaneZGivenXY(point.getX(), point.getY())));
            }

            collisions.put(point, singlePointCollisions);
         } // point frame changed block
         point.changeFrame(previousFrame);
      }

      // find highest collision
      Pair<PlanarRegion, Double> highestCollision = Pair.of(null, Double.NEGATIVE_INFINITY);
      for (TreeSet<Pair<PlanarRegion, Double>> value : collisions.values())
      {
         Pair<PlanarRegion, Double> collisionOccurrence = value.last(); // TODO might be first
         if (collisionOccurrence.getRight() >= highestCollision.getRight())
         {
            highestCollision = collisionOccurrence;
         }
      }

      for (TreeSet<Pair<PlanarRegion, Double>> value : collisions.values())
      {
         if (highestCollision.getLeft() != value.last().getLeft())  // TODO might be first
         {
            return null; // point has its highest collision on a region other than the region of the highest collision of all points
         }
      }

      // make sure to go up or down
      if (Math.abs(highestCollision.getRight() - startingZ) <  HIGH_LOW_MINIMUM)
      {
         return null;
      }

      FramePoint3D centerPoint3D = new FramePoint3D(polygonPoints.getCenterPoint());
      centerPoint3D.changeFrame(ReferenceFrame.getWorldFrame());
      centerPoint3D.setZ(highestCollision.getRight());

      return centerPoint3D; // all points have a collision and all of their highest collisions are with the same region
   }

   public static ArrayList<PlanarRegion> findHighestRegions(PlanarRegionsList regionsList, int numberToGet)
   {
      TreeSet<PlanarRegion> regionsSortedByHeight = new TreeSet<>(Comparator.comparingDouble(region -> centroid(region).getZ()));

      regionsSortedByHeight.addAll(regionsList.getPlanarRegionsAsList()); // sort regions into tree set w/ comparator above

      ArrayList<PlanarRegion> highestRegions = new ArrayList<>();
      for (int i = 0; i < numberToGet; i++)
      {
         PlanarRegion highRegion = regionsSortedByHeight.pollFirst();
         highestRegions.add(highRegion); // TODO might be poll last
         LogTools.debug("Found high region with id {} and centroid {}", highRegion.getRegionId(), centroid(highRegion));
      }
      return highestRegions;
   }

   /** Is normal Z up */
   public static boolean isLevel(PlanarRegion region, double epsilon)
   {
      return region.getNormal().epsilonEquals(new Vector3D(0.0, 0.0, 1.0), epsilon);
   }

   public static double area(PlanarRegion region)
   {
      double totalArea = 0;
      for (ConvexPolygon2D convexPolygon : region.getConvexPolygons())
      {
         totalArea += convexPolygon.getArea();
      }
      return totalArea;
   }

   public static Point3D centroid(PlanarRegion region)
   {
      region.update();
      Point2DReadOnly centroid2DLocal = region.getConvexHull().getCentroid();
      Point3D centroid3D = new Point3D(centroid2DLocal.getX(), centroid2DLocal.getY(), 0.0);
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      region.getTransformToWorld(transformToWorld);  // TODO this might be wrong
      transformToWorld.transform(centroid3D);
      return centroid3D;
   }
}
