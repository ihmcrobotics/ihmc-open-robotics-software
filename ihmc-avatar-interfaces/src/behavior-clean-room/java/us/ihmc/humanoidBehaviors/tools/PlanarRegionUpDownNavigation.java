package us.ihmc.humanoidBehaviors.tools;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.TreeSet;

public class PlanarRegionUpDownNavigation
{
   private static final double LEVEL_EPSILON = 1e-3;
   private static final double HIGH_LOW_MINIMUM = 0.10;
   private static final double MINIMUM_AREA = 0.5 * 0.5; // square half meter

   public enum NavigationResult
   {
      WAYPOINT_FOUND,
      NO_LEVEL_REGIONS,
      NO_HIGHER_REGIONS,
      NO_LARGE_REGIONS,
      NO_QUALIFIED_REGIONS,
      TOO_MANY_QUALIFIED_REGIONS
   }

   public static Pair<NavigationResult, FramePose3D> up(FramePose3D midFeetZUp, PlanarRegionsList planarRegionsList)
   {
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
         if (centroid(planarRegion).getZ() > midFeetZUp.getZ() + HIGH_LOW_MINIMUM)
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
      waypointPose.setPosition(centroid(candidateRegions.first()));
      waypointPose.setOrientation(midFeetZUp.getOrientation());

      return Pair.of(NavigationResult.WAYPOINT_FOUND, waypointPose);
   }

   public static Pair<NavigationResult, FramePose3D> down(FramePose3D midFeetZUp, PlanarRegionsList planarRegionsList)
   {
      return Pair.of(NavigationResult.NO_QUALIFIED_REGIONS, null);
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
