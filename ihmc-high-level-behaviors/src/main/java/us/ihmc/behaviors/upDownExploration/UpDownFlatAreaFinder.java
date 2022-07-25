package us.ihmc.behaviors.upDownExploration;

import static us.ihmc.behaviors.upDownExploration.UpDownFlatAreaFinder.MultiAngleSearch.ANGLE_SUCCESS;
import static us.ihmc.behaviors.upDownExploration.UpDownFlatAreaFinder.MultiAngleSearch.HIT_MAX_ANGLE_TO_SEARCH;
import static us.ihmc.behaviors.upDownExploration.UpDownFlatAreaFinder.MultiAngleSearch.STILL_ANGLING;
import static us.ihmc.behaviors.upDownExploration.UpDownFlatAreaFinder.SingleAngleSearch.HIT_MAX_SEARCH_DISTANCE;
import static us.ihmc.behaviors.upDownExploration.UpDownFlatAreaFinder.SingleAngleSearch.STILL_SEARCHING;
import static us.ihmc.behaviors.upDownExploration.UpDownFlatAreaFinder.SingleAngleSearch.WAYPOINT_FOUND;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.TreeSet;

import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.behaviors.patrol.PatrolBehaviorAPI;
import us.ihmc.behaviors.upDownExploration.UpDownSequence.UpDown;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.commons.thread.ExceptionHandlingThreadScheduler;
import us.ihmc.commons.thread.TypedNotification;

public class UpDownFlatAreaFinder
{
   private static final double LEVEL_EPSILON = 1e-3;
   private static final double HIGH_LOW_MINIMUM = 0.10;
   private static final double MINIMUM_AREA = 0.5 * 0.5; // square half meter
   public static final double REQUIRED_FLAT_AREA_RADIUS = 0.30;
   public static final double MAX_NAVIGATION_DISTANCE = 4.0;
   public static final double CHECK_STEP_SIZE = REQUIRED_FLAT_AREA_RADIUS;
   public static final double ANGLE_STEP_SIZE = 0.05;
   public static final double MAX_ANGLE_TO_SEARCH = 0.6;
   public static final int NUMBER_OF_VERTICES = 6;
   public static final double MINIMUM_NO_HEIGHT_CHANGE_DISTANCE = 0.5;

   private final Messager messager;
   private final ExceptionHandlingThreadScheduler scheduler = new ExceptionHandlingThreadScheduler(getClass().getSimpleName());
   private final FramePose3D midFeetZUpPose = new FramePose3D();
   private final UpDownResult upDownResultForVisualization = new UpDownResult(NUMBER_OF_VERTICES);

   private volatile boolean abort = false;
   private volatile UpDown lastPlanUpDown = UpDown.DOWN;

   private ReferenceFrame midFeetZUpFrame;
   private PlanarRegionsList planarRegionsList;
   private double initialHeight;
   private FramePoint3D centerPoint3D;

   public enum SingleAngleSearch
   {
      STILL_SEARCHING,
      WAYPOINT_FOUND,
      HIT_MAX_SEARCH_DISTANCE,
      ABORTED
   }

   public enum MultiAngleSearch
   {
      STILL_ANGLING,
      ANGLE_SUCCESS,
      HIT_MAX_ANGLE_TO_SEARCH,
      ABORTED
   }

   public UpDownFlatAreaFinder(Messager messager)
   {
      this.messager = messager;
   }

   public TypedNotification<Optional<FramePose3D>> upOrDownOnAThread(ReferenceFrame midFeetZUpFrame,
                                                                     PlanarRegionsList planarRegionsList,
                                                                     boolean requireHeightChange)
   {
      TypedNotification<Optional<FramePose3D>> typedNotification = new TypedNotification<>();
      scheduler.scheduleOnce(() -> typedNotification.set(upOrDown(midFeetZUpFrame, planarRegionsList, requireHeightChange)));
      return typedNotification;
   }

   public void abort()
   {
      abort = true;
   }

   public Optional<FramePose3D> upOrDown(ReferenceFrame midFeetZUpFrame,
                                         PlanarRegionsList planarRegionsList,
                                         boolean requireHeightChange)
   {
      abort = false;
      this.midFeetZUpFrame = midFeetZUpFrame;
      this.planarRegionsList = planarRegionsList;

      midFeetZUpPose.setFromReferenceFrame(midFeetZUpFrame);

      // move straight out from mid feet z up until polygon points share their highest collisions with the same region
      upDownResultForVisualization.reset();
      initialHeight = midFeetZUpPose.getZ();

      double searchAngle = 0.0;
      MultiAngleSearch resultType = STILL_ANGLING;
      do
      {
         if (searchAngle(searchAngle, requireHeightChange) == WAYPOINT_FOUND)
         {
            resultType = ANGLE_SUCCESS;
         }
         else if (searchAngle > 0.0 && searchAngle(-searchAngle, requireHeightChange) == WAYPOINT_FOUND)
         {
            resultType = ANGLE_SUCCESS;
         }
         else if (searchAngle >= MAX_ANGLE_TO_SEARCH)
         {
            resultType = HIT_MAX_ANGLE_TO_SEARCH;
         }
         else if (abort)
         {
            resultType = MultiAngleSearch.ABORTED;
         }

         searchAngle += ANGLE_STEP_SIZE;
      }
      while (resultType == STILL_ANGLING);

      if (resultType == ANGLE_SUCCESS)
      {
         FramePose3D waypointPose = new FramePose3D();
         waypointPose.setFromReferenceFrame(midFeetZUpFrame);
         waypointPose.getPosition().set(centerPoint3D);

         if (waypointPose.getZ() > midFeetZUpPose.getZ())
         {
            lastPlanUpDown = UpDown.UP;
         }
         else
         {
            lastPlanUpDown = UpDown.DOWN;
         }

         LogTools.debug("Qualifying pose found at height {}: {}", centerPoint3D.getZ() - initialHeight, waypointPose);

         return Optional.of(waypointPose);
      }

      return Optional.empty();
   }

   private SingleAngleSearch searchAngle(double angle, boolean requireHeightChange)
   {
      UpDownPolyonCheckPoints2D polygonPoints = new UpDownPolyonCheckPoints2D(NUMBER_OF_VERTICES, REQUIRED_FLAT_AREA_RADIUS);
      polygonPoints.reset(midFeetZUpFrame);
      if (requireHeightChange)
      {
         polygonPoints.add(CHECK_STEP_SIZE * Math.cos(angle), CHECK_STEP_SIZE * Math.sin(angle)); // initial check step a bit out
      }
      else
      {
         polygonPoints.add(MINIMUM_NO_HEIGHT_CHANGE_DISTANCE * Math.cos(angle), MINIMUM_NO_HEIGHT_CHANGE_DISTANCE * Math.sin(angle)); // initial check step a bit out
      }
      SingleAngleSearch resultType = STILL_SEARCHING;
      do
      {
         polygonPoints.add(CHECK_STEP_SIZE * Math.cos(angle), CHECK_STEP_SIZE * Math.sin(angle));
         LogTools.trace("Stepping virtual polygon points forward by {}", CHECK_STEP_SIZE);

         if (centerPointOfPolygonWhenPointsShareHighestCollisionsWithSameRegion(polygonPoints, requireHeightChange))
         {
            resultType = WAYPOINT_FOUND;
         }
         else if (polygonPoints.getCenterPoint().getX() >= MAX_NAVIGATION_DISTANCE)
         {
            resultType = HIT_MAX_SEARCH_DISTANCE;
         }
         else if (abort)
         {
            resultType = SingleAngleSearch.ABORTED;
         }

         LogTools.trace("Polygon center point {}", polygonPoints.getCenterPoint());

         messager.submitMessage(PatrolBehaviorAPI.UpDownGoalPoses, upDownResultForVisualization);
         ThreadTools.sleepSeconds(0.02);

      }
      while (resultType == STILL_SEARCHING);

      return resultType;
   }

   private boolean centerPointOfPolygonWhenPointsShareHighestCollisionsWithSameRegion(UpDownPolyonCheckPoints2D polygonPoints,
                                                                                      boolean requireHeightChange)
   {
      boolean allPointsCollideAtLeastOnce = true;

      upDownResultForVisualization.setValid(false);

      // map points to their collisions
      HashMap<FramePoint2DReadOnly, TreeSet<Pair<PlanarRegion, Double>>> collisions = new HashMap<>();
      List<FramePoint2D> points = polygonPoints.getPoints();
      for (int i = 0; i < points.size(); i++)
      {
         FramePoint2D point = new FramePoint2D(points.get(i));
         point.changeFrame(ReferenceFrame.getWorldFrame());

         TreeSet<Pair<PlanarRegion, Double>> singlePointCollisions = new TreeSet<>(Comparator.comparingDouble(o -> o.getRight()));
         List<PlanarRegion> collidedRegions = planarRegionsList.findPlanarRegionsContainingPointByVerticalLineIntersection(point);

         upDownResultForVisualization.getPoints().get(i).setX(point.getX());
         upDownResultForVisualization.getPoints().get(i).setY(point.getY());

         if (collidedRegions == null || collidedRegions.isEmpty())
         {
            LogTools.trace("not every point collides once");
            allPointsCollideAtLeastOnce = false; // not every point collides once
            upDownResultForVisualization.getPoints().get(i).setZ(0.0);
         }
         else
         {
            for (PlanarRegion collidedRegion : collidedRegions)
            {
               singlePointCollisions.add(Pair.of(collidedRegion, collidedRegion.getPlaneZGivenXY(point.getX(), point.getY())));
            }

            collisions.put(point, singlePointCollisions);

            upDownResultForVisualization.getPoints().get(i).setZ(singlePointCollisions.last().getRight());
         }
      }

      if (allPointsCollideAtLeastOnce == false)
      {
         return false;
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

      if (requireHeightChange && Math.abs(highestCollision.getRight() - initialHeight) <  HIGH_LOW_MINIMUM) // make sure to go up or down
      {
         LogTools.trace("doesn't go up or down");
         return false;
      }

      for (TreeSet<Pair<PlanarRegion, Double>> value : collisions.values()) // make sure all points' highest collisions are on same region
      {
         if (highestCollision.getLeft() != value.last().getLeft())  // last is highest
         {
            LogTools.trace("point has its highest collision on a region other than the region of the highest collision of all points");
            return false; // point has its highest collision on a region other than the region of the highest collision of all points
         }
      }

      centerPoint3D = new FramePoint3D(polygonPoints.getCenterPoint());
      centerPoint3D.changeFrame(ReferenceFrame.getWorldFrame());
      centerPoint3D.setZ(highestCollision.getRight());

      LogTools.trace("Returning valid center point! {}", centerPoint3D);
      upDownResultForVisualization.setValid(true);

      return true;
   }

   public UpDown getLastPlanUpDown()
   {
      return lastPlanUpDown;
   }

   private ArrayList<PlanarRegion> findHighestRegions(PlanarRegionsList regionsList, int numberToGet)
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
   private boolean isLevel(PlanarRegion region, double epsilon)
   {
      return region.getNormal().epsilonEquals(new Vector3D(0.0, 0.0, 1.0), epsilon);
   }

   private double area(PlanarRegion region)
   {
      double totalArea = 0;
      for (ConvexPolygon2D convexPolygon : region.getConvexPolygons())
      {
         totalArea += convexPolygon.getArea();
      }
      return totalArea;
   }

   private Point3D centroid(PlanarRegion region)
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
