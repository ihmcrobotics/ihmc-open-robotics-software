package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.Arrays;
import java.util.List;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ExtrusionSide;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.ExtrusionHull;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;

import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.*;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.distanceSquaredFromPoint2DToLineSegment2D;
import static us.ihmc.euclid.tools.EuclidCoreTools.norm;
import static us.ihmc.euclid.tools.EuclidCoreTools.normSquared;

public class VisibilityTools
{
   public static boolean isPointVisible(Point2DReadOnly observer, Point2DReadOnly targetPoint, ExtrusionHull points, boolean closed)
   {
      return isPointVisible(observer, targetPoint, points.getPoints(), closed);
   }

   public static boolean isPointVisible(Point2DReadOnly observer, Point2DReadOnly targetPoint, List<Point2DReadOnly> listOfPointsInCluster, boolean closed)
   {
      int size = listOfPointsInCluster.size();
      int endIndex = size - 1;
      if (closed)
         endIndex++;

      for (int i = 0; i < endIndex; i++)
      {
         Point2DReadOnly first = listOfPointsInCluster.get(i);

         int nextIndex = i + 1;
         if (nextIndex == size)
            nextIndex = 0;

         Point2DReadOnly second = listOfPointsInCluster.get(nextIndex);

         if (EuclidGeometryTools.doLineSegment2DsIntersect(first, second, observer, targetPoint))
         {
            return false;
         }
      }
      return true;
   }

   public static boolean isPointVisibleInclusive(Point2DReadOnly observer, Point2DReadOnly targetPoint, List<Point2DReadOnly> listOfPointsInCluster,
                                                 boolean closed)
   {
      int size = listOfPointsInCluster.size();
      int endIndex = size - 1;
      if (closed)
         endIndex++;

      for (int i = 0; i < endIndex; i++)
      {
         Point2DReadOnly first = listOfPointsInCluster.get(i);

         int nextIndex = i + 1;
         if (nextIndex == size)
            nextIndex = 0;

         Point2DReadOnly second = listOfPointsInCluster.get(nextIndex);

         // this will return true if they share a point, which isn't always a good thing
         boolean sharesAPoint = first.epsilonEquals(targetPoint, 1.0e-10);
         sharesAPoint |= first.epsilonEquals(observer, 1.0e-10);
         sharesAPoint |= second.epsilonEquals(targetPoint, 1.0e-10);
         sharesAPoint |= second.epsilonEquals(observer, 1.0e-10);

         if (!sharesAPoint && EuclidGeometryTools.doLineSegment2DsIntersect(first, second, observer, targetPoint))
         {
            return false;
         }
      }
      return true;
   }

   public static double distanceToCluster(Point2DReadOnly firstPointOfLine, Point2DReadOnly secondPointOfLine, ExtrusionHull extrusion,
                                          Point2DBasics closestPointOnLineToPack, Point2DBasics closestPointOnClusterToPack,
                                          Vector2DBasics normalToClusterToPack, boolean closed)
   {
      return distanceToCluster(firstPointOfLine, secondPointOfLine, extrusion.getPoints(), closestPointOnLineToPack, closestPointOnClusterToPack,
                               normalToClusterToPack, closed);
   }

   public static double distanceToCluster(Point2DReadOnly firstPointOfLine, Point2DReadOnly secondPointOfLine, List<Point2DReadOnly> listOfPointsInCluster,
                                          Point2DBasics closestPointOnLineToPack, Point2DBasics closestPointOnClusterToPack,
                                          Vector2DBasics normalToClusterToPack, boolean closed)
   {
      int numberOfVertices = listOfPointsInCluster.size();

      if (numberOfVertices == 0)
      {
         if (closestPointOnLineToPack != null)
            closestPointOnLineToPack.setToNaN();
         if (closestPointOnClusterToPack != null)
            closestPointOnClusterToPack.setToNaN();
         if (normalToClusterToPack != null)
            normalToClusterToPack.setToNaN();
         return Double.NaN;
      }

      if (numberOfVertices == 1)
      {
         Point2DReadOnly clusterPoint = listOfPointsInCluster.get(0);
         orthogonalProjectionOnLineSegment2D(clusterPoint, firstPointOfLine, secondPointOfLine, closestPointOnLineToPack);
         if (normalToClusterToPack != null)
            normalToClusterToPack.setToZero();
         if (closestPointOnClusterToPack != null)
            closestPointOnClusterToPack.set(clusterPoint);
         return clusterPoint.distance(closestPointOnLineToPack);
      }

      if (numberOfVertices == 2)
      {
         if (normalToClusterToPack != null)
         {
            normalToClusterToPack.sub(listOfPointsInCluster.get(1), listOfPointsInCluster.get(0));
            EuclidGeometryTools.perpendicularVector2D(normalToClusterToPack, normalToClusterToPack);
            normalToClusterToPack.normalize();
         }

         return VisGraphGeometryTools
               .closestPoint2DsBetweenTwoLineSegment2Ds(firstPointOfLine, secondPointOfLine, listOfPointsInCluster.get(0), listOfPointsInCluster.get(1),
                                                        closestPointOnLineToPack, closestPointOnClusterToPack);
      }

      boolean pointIsVisible = isPointVisible(firstPointOfLine, secondPointOfLine, listOfPointsInCluster, closed);

      double minDistance = Double.POSITIVE_INFINITY;

      Point2DBasics closestPointOnLine = new Point2D();
      Point2DBasics closestPointOnCluster = new Point2D();

      for (int index = 0; index < numberOfVertices; index++)
      {
         Point2DReadOnly edgeStart = listOfPointsInCluster.get(index);
         Point2DReadOnly edgeEnd = listOfPointsInCluster.get(EuclidGeometryPolygonTools.next(index, numberOfVertices));

         double distance = VisGraphGeometryTools
               .closestPoint2DsBetweenTwoLineSegment2Ds(firstPointOfLine, secondPointOfLine, edgeStart, edgeEnd, closestPointOnLine,
                                                        closestPointOnCluster);
         if (distance < minDistance)
         {
            minDistance = distance;
            if (closestPointOnLineToPack != null)
               closestPointOnLineToPack.set(closestPointOnLine);
            if (closestPointOnClusterToPack != null)
               closestPointOnClusterToPack.set(closestPointOnCluster);

            if (normalToClusterToPack != null)
            {
               normalToClusterToPack.sub(edgeEnd, edgeStart);
               EuclidGeometryTools.perpendicularVector2D(normalToClusterToPack, normalToClusterToPack);
               normalToClusterToPack.normalize();
            }
         }
      }

      if (!pointIsVisible)
         minDistance = -minDistance;
      return minDistance;
   }

   public static double distanceToCluster(Point2DReadOnly point, ExtrusionHull extrusion, Point2DBasics closestPointInCluster, Vector2DBasics normalToCluster)
   {
      return distanceToCluster(point, extrusion.getPoints(), closestPointInCluster, normalToCluster);
   }

   public static double distanceToCluster(Point2DReadOnly point, List<Point2DReadOnly> listOfPointsInCluster, Point2DBasics closestPointInClusterToPack,
                                          Vector2DBasics normalToClusterToPack)
   {
      int numberOfVertices = listOfPointsInCluster.size();

      if (numberOfVertices == 0)
      {
         closestPointInClusterToPack.setToNaN();
         if (normalToClusterToPack != null)
            normalToClusterToPack.setToNaN();
         return Double.NaN;
      }

      if (numberOfVertices == 1)
      {
         closestPointInClusterToPack.set(listOfPointsInCluster.get(0));
         if (normalToClusterToPack != null)
            normalToClusterToPack.setToZero();
         return distanceBetweenPoint2Ds(point.getX(), point.getY(), listOfPointsInCluster.get(0));
      }

      if (numberOfVertices == 2)
      {
         orthogonalProjectionOnLineSegment2D(point, listOfPointsInCluster.get(0), listOfPointsInCluster.get(1), closestPointInClusterToPack);

         if (normalToClusterToPack != null)
         {
            normalToClusterToPack.sub(listOfPointsInCluster.get(1), listOfPointsInCluster.get(0));
            EuclidGeometryTools.perpendicularVector2D(normalToClusterToPack, normalToClusterToPack);
            normalToClusterToPack.normalize();
         }

         return point.distance(closestPointInClusterToPack);
      }

      Point2DBasics tempPoint = new Point2D();
      boolean isQueryOutsidePolygon = false;
      double minDistance = Double.POSITIVE_INFINITY;

      for (int index = 0; index < numberOfVertices; index++)
      {
         Point2DReadOnly edgeStart = listOfPointsInCluster.get(index);
         Point2DReadOnly edgeEnd = listOfPointsInCluster.get(EuclidGeometryPolygonTools.next(index, numberOfVertices));

         isQueryOutsidePolygon |= isPoint2DOnSideOfLine2D(point.getX(), point.getY(), edgeStart, edgeEnd, true);

         orthogonalProjectionOnLineSegment2D(point, edgeStart, edgeEnd, tempPoint);
         double distance = point.distance(tempPoint);

         if (distance < minDistance)
         {
            minDistance = distance;
            closestPointInClusterToPack.set(tempPoint);

            if (normalToClusterToPack != null)
            {
               normalToClusterToPack.sub(edgeEnd, edgeStart);
               EuclidGeometryTools.perpendicularVector2D(normalToClusterToPack, normalToClusterToPack);
               normalToClusterToPack.normalize();
            }
         }
      }

      if (!isQueryOutsidePolygon)
         minDistance = -minDistance;
      return minDistance;
   }

   public static boolean checkIfPointIsInRegionAndOutsidePreferredNonNavigableZone(Point2DReadOnly query, PlanarRegion homeRegion, List<Cluster> allClusters)
   {
      // Check that the point is actually navigable
      boolean isNavigable = PlanarRegionTools.isPointInLocalInsidePlanarRegion(homeRegion, query);

      if (isNavigable)
      {
         isNavigable = allClusters.stream().noneMatch(cluster -> cluster.isInsidePreferredNonNavigableZone(query));
      }

      return isNavigable;
   }

   public static boolean checkIfPointIsInRegionAndOutsideNonNavigableZone(Point2DReadOnly query, PlanarRegion homeRegion, List<Cluster> allClusters)
   {
      // Check that the point is actually navigable
      boolean isNavigable = PlanarRegionTools.isPointInLocalInsidePlanarRegion(homeRegion, query);

      if (isNavigable)
      {
         isNavigable = allClusters.stream().noneMatch(cluster -> cluster.isInsideNonNavigableZone(query));
      }

      return isNavigable;
   }

   //TODO: Rename.
   public static boolean isPointVisibleForStaticMaps(List<Cluster> clusters, Point2DReadOnly observer, Point2DReadOnly targetPoint)
   {
      return isPointVisibleForStaticMaps(clusters, observer, targetPoint, false);
   }

   public static boolean isPointVisibleForStaticMaps(List<Cluster> clusters, Point2DReadOnly observer, Point2DReadOnly targetPoint,
                                                     boolean checkPreferredExtrusions)
   {
      for (Cluster cluster : clusters)
      {
         boolean closed = cluster.isClosed();

         List<ExtrusionHull> preferredNonNavigableExtrusions = cluster.getPreferredNonNavigableExtrusionsInLocal();

         boolean isAnOuterExtrusion = cluster.getExtrusionSide() == ExtrusionSide.OUTSIDE;
         if (isAnOuterExtrusion)
         {
            BoundingBox2DReadOnly outerMostBoundingBoxToCheck = checkPreferredExtrusions ?
                  cluster.getPreferredNonNavigableExtrusionsBoundingBox() : cluster.getNonNavigableExtrusionsBoundingBox();

            // If either the target or observer or both are in the bounding box, we have to check the interior bounding box.
            // If both are outside the bounding box, then we can check if the line segment does not intersect.
            // If that is the case, then the point is visible and we can check the next one.
            if (!outerMostBoundingBoxToCheck.isInsideInclusive(observer) && !outerMostBoundingBoxToCheck.isInsideInclusive(targetPoint))
            {
               if (!outerMostBoundingBoxToCheck.doesIntersectWithLineSegment2D(observer, targetPoint))
               {
                  continue;
               }
            }

            /*
            ConvexPolygon2DReadOnly convexHullToCheck = checkPreferredExtrusions ? cluster.getPreferredNonNavigableExtrusionsConvexHull() :
                  cluster.getNonNavigableExtrusionsConvexHull();

            if (checkPreferredExtrusions && !convexHullToCheck.isPointInside(observer) && !convexHullToCheck.isPointInside(targetPoint))
            {
               if (VisibilityTools.isPointVisible(observer, targetPoint, convexHullToCheck.getPolygonVerticesView(), true))
               {
                  continue;
               }
            }

             */
         }

         if (!VisibilityTools.isPointVisible(observer, targetPoint, cluster.getNonNavigableExtrusionsInLocal(), closed))
            return false;

         // this is more expensive, as you potentially have to check multiple regions.
         if (checkPreferredExtrusions)
         {
            boolean startsInPreferredRegion = preferredNonNavigableExtrusions.stream().anyMatch(
                  extrusion -> EuclidGeometryPolygonTools.isPoint2DInsideConvexPolygon2D(observer, extrusion.getPoints(), extrusion.size(), true, 0.0));
            boolean isNotVisible = preferredNonNavigableExtrusions.stream().anyMatch(
                  extrusion -> !VisibilityTools.isPointVisible(observer, targetPoint, extrusion, closed));

            // If we start in a preferred region, that means you're already in a preferred extrusion. If it's an outer extrusion, we want to get out of the
            // non-preferred area, so we shouldn't check for visibility. If it's an inner extrusion, we want to cross into the non-preferred area.
            if (isAnOuterExtrusion ^ startsInPreferredRegion && isNotVisible)
               return false;
         }
      }

      return true;
   }
}
