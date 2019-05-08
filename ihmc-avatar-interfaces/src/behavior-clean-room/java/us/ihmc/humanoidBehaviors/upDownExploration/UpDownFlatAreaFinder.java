package us.ihmc.humanoidBehaviors.upDownExploration;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehaviorAPI;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.*;

import static us.ihmc.humanoidBehaviors.upDownExploration.UpDownFlatAreaFinder.UpDownResultType.*;

public class UpDownFlatAreaFinder
{
   private static final double LEVEL_EPSILON = 1e-3;
   private static final double HIGH_LOW_MINIMUM = 0.10;
   private static final double MINIMUM_AREA = 0.5 * 0.5; // square half meter
   public static final double REQUIRED_FLAT_AREA_RADIUS = 0.35;
   public static final double MAX_NAVIGATION_DISTANCE = 4.0;
   public static final double CHECK_STEP_SIZE = REQUIRED_FLAT_AREA_RADIUS;
   public static final int NUMBER_OF_VERTICES = 5;

   public enum UpDownResultType
   {
      STILL_SEARCHING,
      WAYPOINT_FOUND,
      HIT_MAX_SEARCH_DISTANCE,
   }

   public static Pair<UpDownResultType, FramePose3D> upOrDown(ReferenceFrame midFeetZUpFrame, PlanarRegionsList planarRegionsList, Messager messager)
   {
      FramePose3D midFeetZUpPose = new FramePose3D();
      midFeetZUpPose.setFromReferenceFrame(midFeetZUpFrame);

      // move straight out from mid feet z up until polygon points share their highest collisions with the same region
      PolygonPoints2D polygonPoints = new PolygonPoints2D(NUMBER_OF_VERTICES, REQUIRED_FLAT_AREA_RADIUS, midFeetZUpFrame);

      UpDownResult upDownResult = new UpDownResult(NUMBER_OF_VERTICES);

      polygonPoints.add(CHECK_STEP_SIZE, 0.0); // initial check step a bit out

      FramePoint3D centerPoint3D;
      UpDownResultType resultType = STILL_SEARCHING;
      do
      {
         polygonPoints.add(CHECK_STEP_SIZE, 0.0);
         LogTools.info("Stepping virtual polygon points forward by {}", CHECK_STEP_SIZE);

         centerPoint3D = centerPointOfPolygonWhenPointsShareHighestCollisionsWithSameRegion(polygonPoints,
                                                                                            planarRegionsList,
                                                                                            midFeetZUpPose.getZ(),
                                                                                            upDownResult);
         LogTools.info("Polygon center point {}", polygonPoints.getCenterPoint());
         if (centerPoint3D != null)
         {
            resultType = WAYPOINT_FOUND;
         }
         else if (polygonPoints.getCenterPoint().getX() >= MAX_NAVIGATION_DISTANCE)
         {
            resultType = HIT_MAX_SEARCH_DISTANCE;
         }

         messager.submitMessage(PatrolBehaviorAPI.UpDownGoalPoses, upDownResult);
         ThreadTools.sleepSeconds(0.25);
      }
      while (resultType == STILL_SEARCHING);

      if (resultType == WAYPOINT_FOUND)
      {
         FramePose3D waypointPose = new FramePose3D();
         waypointPose.setFromReferenceFrame(midFeetZUpFrame);
         waypointPose.setPosition(centerPoint3D);

         LogTools.debug("Qualifying pose found at height {}: {}", centerPoint3D.getZ() - midFeetZUpPose.getZ(), waypointPose);

         return Pair.of(resultType, waypointPose);
      }

      return Pair.of(resultType, null);
   }

   public static FramePoint3D centerPointOfPolygonWhenPointsShareHighestCollisionsWithSameRegion(PolygonPoints2D polygonPoints,
                                                                                                 PlanarRegionsList planarRegionsList,
                                                                                                 double startingZ,
                                                                                                 UpDownResult upDownResult)
   {
      upDownResult.setValid(false);

      // map points to their collisions
      HashMap<FramePoint2D, TreeSet<Pair<PlanarRegion, Double>>> collisions = new HashMap<>();
      List<FramePoint2D> points = polygonPoints.getPoints();
      for (int i = 0; i < points.size(); i++)
      {
         FramePoint2D point = points.get(i);
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

            // pack 3D point for visualization
            upDownResult.getPoints().get(i).setX(point.getX());
            upDownResult.getPoints().get(i).setY(point.getY());
            upDownResult.getPoints().get(i).setZ(singlePointCollisions.last().getRight());
         } // point frame changed block
         point.changeFrame(previousFrame);
      }

      // find highest collision
      Pair<PlanarRegion, Double> highestCollision = Pair.of(null, Double.NEGATIVE_INFINITY);
      for (TreeSet<Pair<PlanarRegion, Double>> value : collisions.values())
      {
         Pair<PlanarRegion, Double> collisionOccurrence = value.last(); // last is highest
         if (collisionOccurrence.getRight() >= highestCollision.getRight())
         {
            highestCollision = collisionOccurrence;
         }
      }

      if (Math.abs(highestCollision.getRight() - startingZ) <  HIGH_LOW_MINIMUM) // make sure to go up or down
      {
         return null;
      }

      for (TreeSet<Pair<PlanarRegion, Double>> value : collisions.values()) // make sure all points' highest collisions are on same region
      {
         if (highestCollision.getLeft() != value.last().getLeft())  // last is highest
         {
            return null; // point has its highest collision on a region other than the region of the highest collision of all points
         }
      }

      FramePoint3D centerPoint3D = new FramePoint3D(polygonPoints.getCenterPoint());
      centerPoint3D.changeFrame(ReferenceFrame.getWorldFrame());
      centerPoint3D.setZ(highestCollision.getRight());

      LogTools.debug("Returning valid center point! {}", centerPoint3D);
      upDownResult.setValid(true);

      return centerPoint3D; // all points have a collision and all of their highest collisions are with the same region
   }

   public static ArrayList<PlanarRegion> findHighestRegions(PlanarRegionsList regionsList, int numberToGet)
   {
      TreeSet<PlanarRegion> regionsSortedByHeight = new TreeSet<>(Comparator.comparingDouble(region -> centroid(region).getZ()));

      regionsSortedByHeight.addAll(regionsList.getPlanarRegionsAsList()); // sort regions into tree set w/ comparator above

      ArrayList<PlanarRegion> highestRegions = new ArrayList<>();
      for (int i = 0; i < numberToGet; i++)
      {
         PlanarRegion highRegion = regionsSortedByHeight.pollFirst(); // TODO might be poll last
         highestRegions.add(highRegion);
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
