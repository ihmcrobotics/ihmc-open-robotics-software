package us.ihmc.pathPlanning.visibilityGraphs.tools;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegions;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools.isPointInsideConvexPolygon2D;

public class NavigableRegionTools
{
   public static NavigableRegion getNavigableRegionContainingThisPoint(Point3DReadOnly point, NavigableRegions navigableRegions)
   {
      return getNavigableRegionContainingThisPoint(point, navigableRegions, 0.0);
   }

   public static NavigableRegion getNavigableRegionContainingThisPoint(Point3DReadOnly point, NavigableRegions navigableRegions, double epsilon)
   {
      List<NavigableRegion> containers = new ArrayList<>();

      List<NavigableRegion> navigableRegionsList = navigableRegions.getNaviableRegionsList();
      if (navigableRegionsList == null)
         return null;

      for (NavigableRegion navigableRegion : navigableRegionsList)
      {
         if (isPointInWorldInsideNavigableRegion(navigableRegion, point, epsilon))
         {
            containers.add(navigableRegion);
         }
      }

      if (containers.isEmpty())
         return null;
      if (containers.size() == 1)
         return containers.get(0);

      Point3D pointOnRegion = new Point3D();
      Vector3D regionNormal = new Vector3D();

      NavigableRegion closestContainer = containers.get(0);
      closestContainer.getHomePlanarRegion().getNormal(regionNormal);
      closestContainer.getHomePlanarRegion().getPointInRegion(pointOnRegion);
      double minDistance = EuclidGeometryTools.distanceFromPoint3DToPlane3D(point, pointOnRegion, regionNormal);

      for (int i = 1; i < containers.size(); i++)
      {
         NavigableRegion candidate = containers.get(i);
         candidate.getHomePlanarRegion().getNormal(regionNormal);
         candidate.getHomePlanarRegion().getPointInRegion(pointOnRegion);
         double distance = EuclidGeometryTools.distanceFromPoint3DToPlane3D(point, pointOnRegion, regionNormal);
         if (distance < minDistance)
         {
            closestContainer = candidate;
            minDistance = distance;
         }
      }

      return closestContainer;
   }

   public static boolean isPointInWorldInsideNavigableRegion(NavigableRegion navigableRegion, Point3DReadOnly pointInWorldToCheck, double epsilon)
   {
      RigidBodyTransform transformToWorld = navigableRegion.getTransformToWorld();
      Point2D pointInLocalToCheck = new Point2D(pointInWorldToCheck);
      pointInLocalToCheck.applyInverseTransform(transformToWorld, false);
      return isPointInLocalInsideNavigableRegion(navigableRegion, pointInLocalToCheck, epsilon);
   }

   public static boolean isPointInLocalInsideNavigableRegion(NavigableRegion navigableRegion, Point2DReadOnly pointInLocalToCheck, double epsilon)
   {
      PlanarRegion planarRegion = navigableRegion.getHomePlanarRegion();
      ConvexPolygon2D convexHull = planarRegion.getConvexHull();
      BoundingBox2D boundingBox = convexHull.getBoundingBox();

      if (!boundingBox.isInsideEpsilon(pointInLocalToCheck, epsilon))
         return false;
      if (!convexHull.isPointInside(pointInLocalToCheck, epsilon))
         return false;

      // TODO: RobertGriffin: This is not 100% correct. This checks that the point is inside the home region cluster of the navigable region. There are
      // no guarantees that this cluster is convex;
      if (!isPointInsideNavigableRegion(navigableRegion, pointInLocalToCheck))
         return false;

      for (Cluster obstacleCluster : navigableRegion.getObstacleClusters())
      {
         if (isPointInsideConvexHullOfCluster(obstacleCluster, pointInLocalToCheck))
            return false;
      }

      List<ConvexPolygon2D> convexPolygons = planarRegion.getConvexPolygons();
      // If inside the convex hull at this point, then if there is only one polygon, you are also inside that too...
      //TODO: Unit tests for all of this.
      if (convexPolygons.size() == 1)
      {
         return true;
      }

      if (MathTools.epsilonEquals(0.0, epsilon, 1.0e-10))
      {
         //TODO: +++JerryPratt: Discuss this one with Sylvain. Do we want to check inside the concave hull, or check each planar region individually?

         for (ConvexPolygon2D convexPolygon : convexPolygons)
         {
            //+++JerryPratt: Not sure if this one is faster or not. Discuss with Sylvain best way to do point inside convex polygon check.
            // Seems like you should be able to do a binary search on the distance to vertices, since it should be monotonic, right?
            //            boolean isInsidePolygon = convexPolygon.isPointInside(pointInLocalToCheck);
            boolean isInsidePolygon = isPointInsideConvexPolygon2D(convexPolygon, pointInLocalToCheck);

            if (isInsidePolygon)
               return true;
         }
         return false;

         //         return isPointInsidePolygon(planarRegion.getConcaveHull(), pointInLocalToCheck);
      }
      else
      {
         //TODO: +++JerryPratt: Discuss this one with Sylvain. Do we want to check inside the concave hull, or check each planar region individually?

         for (ConvexPolygon2D convexPolygon : convexPolygons)
         {
            //+++JerryPratt: Not sure if this one is faster or not. Discuss with Sylvain best way to do point inside convex polygon check.
            // Seems like you should be able to do a binary search on the distance to vertices, since it should be monotonic, right?
            //            boolean isInsidePolygon = convexPolygon.isPointInside(pointInLocalToCheck);
            boolean isInsidePolygon = convexPolygon.isPointInside(pointInLocalToCheck, epsilon);

            if (isInsidePolygon)
               return true;

            //TODO: +++JerryPratt: Discuss using the concaveHull or not. It seems buggy when points cross over the other side..
            // When ClusterTools.extrudePolygon() is buggy...
            //
            //            if (planarRegion.getConcaveHullSize() < convexHull.getNumberOfVertices())
            //               throw new IllegalArgumentException("The concave hull of this polygon is not valid.");
            //
            //         double[] epsilons = new double[planarRegion.getConcaveHullSize()];
            //         Arrays.fill(epsilons, epsilon);
            //         List<Point2D> concaveHull = ClusterTools.extrudePolygon(true, Arrays.asList(planarRegion.getConcaveHull()), epsilons);
            //
            //         return isPointInsidePolygon(concaveHull, pointInLocalToCheck);
         }
      }
      return false;
   }

   /**
    * Return true if the given point is contained inside the boundary.
    * https://stackoverflow.com/questions/8721406/how-to-determine-if-a-point-is-inside-a-2d-convex-polygon
    *
    * Also check https://en.wikipedia.org/wiki/Point_in_polygon.
    *
    * @param test The point to check
    * @return true if the point is inside the boundary, false otherwise
    *
    */
   public static boolean isPointInsideNavigableRegion(NavigableRegion navigableRegion, Point2DReadOnly test)
   {
      return isPointInsideConvexHullOfCluster(navigableRegion.getHomeRegionCluster(), test);
   }

   public static boolean isPointInsideConvexHullOfCluster(Cluster cluster, Point2DReadOnly test)
   {
      List<Point2DReadOnly> vertices = cluster.getNonNavigableExtrusionsInLocal();
      int numberOfVertices = vertices.size();

      int i;
      int j;
      boolean result = false;

      for (i = 0, j = numberOfVertices - 1; i < numberOfVertices; j = i++)
      {
         Point2DReadOnly iVertex = vertices.get(i);
         Point2DReadOnly jVertex = vertices.get(j);

         if ((iVertex.getY() > test.getY()) != (jVertex.getY() > test.getY())
               && (test.getX() < (jVertex.getX() - iVertex.getX()) * (test.getY() - iVertex.getY()) / (jVertex.getY() - iVertex.getY()) + iVertex.getX()))
         {
            result = !result;
         }
      }
      return result;
   }
}
