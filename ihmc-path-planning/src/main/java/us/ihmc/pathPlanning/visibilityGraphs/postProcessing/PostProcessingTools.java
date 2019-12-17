package us.ihmc.pathPlanning.visibilityGraphs.postProcessing;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.ExtrusionHull;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityGraphNode;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityTools;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.*;

public class PostProcessingTools
{
   private static final double samePointEpsilon = 0.01;

   static void removeDuplicated3DPointsFromList(List<? extends Point3DReadOnly> listOfPoints, double samePointEpsilon)
   {
      int pointIndex = 0;
      while (pointIndex < listOfPoints.size() - 1)
      {
         int otherPointIndex = pointIndex + 1;
         while (otherPointIndex < listOfPoints.size())
         {
            if (listOfPoints.get(pointIndex).distance(listOfPoints.get(otherPointIndex)) < samePointEpsilon)
               listOfPoints.remove(otherPointIndex);
            else
               otherPointIndex++;
         }
         pointIndex++;
      }
   }

   static void removeDuplicated2DPointsFromList(List<? extends Point2DReadOnly> listOfPoints, double samePointEpsilon)
   {
      int pointIndex = 0;
      while (pointIndex < listOfPoints.size() - 1)
      {
         int otherPointIndex = pointIndex + 1;
         while (otherPointIndex < listOfPoints.size())
         {
            if (listOfPoints.get(pointIndex).distance(listOfPoints.get(otherPointIndex)) < samePointEpsilon)
               listOfPoints.remove(otherPointIndex);
            else
               otherPointIndex++;
         }
         pointIndex++;
      }
   }

   static void removeDuplicateStartOrEndPointsFromList(List<? extends Point3DReadOnly> listOfPoints, Point3DReadOnly startPoint, Point3DReadOnly endPoint,
                                                       double samePointEpsilon)
   {
      int pointIndex = 0;
      while (pointIndex < listOfPoints.size())
      {
         Point3DReadOnly pointToCheck = listOfPoints.get(pointIndex);
         if (pointToCheck.distance(startPoint) < samePointEpsilon || pointToCheck.distance(endPoint) < samePointEpsilon)
            listOfPoints.remove(pointIndex);
         else
            pointIndex++;
      }
   }

   static double findHeightOfPoint(Point2DReadOnly pointInWorld, Collection<NavigableRegion> navigableRegions)
   {
      return findHeightOfPoint(pointInWorld.getX(), pointInWorld.getY(), navigableRegions);
   }

   private static double findHeightOfPoint(double pointX, double pointY, Collection<NavigableRegion> navigableRegions)
   {
      double maxHeight = Double.NEGATIVE_INFINITY;
      for (NavigableRegion navigableRegion : navigableRegions)
      {
         PlanarRegion planarRegion = navigableRegion.getHomePlanarRegion();
         if (planarRegion.isPointInWorld2DInside(new Point3D(pointX, pointY, 0.0)))
         {
            double height = planarRegion.getPlaneZGivenXY(pointX, pointY);
            if (height > maxHeight)
               maxHeight = height;
         }
      }

      if (!Double.isFinite(maxHeight))
      {
         // we're not on region, so let's average the two regions
         double height = 0.0;
         for (NavigableRegion navigableRegion : navigableRegions)
         {
            height += navigableRegion.getPlaneZGivenXY(pointX, pointY);
         }
         height /= navigableRegions.size();
         maxHeight = height;
      }

      return maxHeight;
   }

   static List<Point2DReadOnly> getPointsAlongEdgeOfClusterClosestToPoint(Point2DReadOnly pointToCheck, Cluster cluster)
   {
      return getPointsAlongEdgeOfClusterClosestToPoint(pointToCheck, cluster.getRawPointsInWorld2D());
   }

   static List<Point2DReadOnly> getPointsAlongEdgeOfClusterClosestToPoint(Point2DReadOnly pointToCheck, List<Point2DReadOnly> points)
   {
      List<LineSegment2DReadOnly> segments = getClusterEdges(points);

      List<Point2DReadOnly> closestPointsToReturn = new ArrayList<>();
      for (LineSegment2DReadOnly lineSegment : segments)
      {
         Point2D closestPoint = new Point2D();
         lineSegment.orthogonalProjection(pointToCheck, closestPoint);

         boolean isProjectionVisible = true;
         for (LineSegment2DReadOnly otherSegment : segments)
         {
            if (otherSegment == lineSegment)
               continue;

            if (null != EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(pointToCheck, closestPoint, otherSegment.getFirstEndpoint(),
                                                                                 otherSegment.getSecondEndpoint()))
            {
               isProjectionVisible = false;
               break;
            }
         }

         if (isProjectionVisible)
            closestPointsToReturn.add(closestPoint);
      }

      return closestPointsToReturn;
   }

   static List<LineSegment2DReadOnly> getClusterEdges(List<Point2DReadOnly> points)
   {
      List<LineSegment2DReadOnly> edges = new ArrayList<>();
      for (int i = 0; i < points.size(); i++)
      {
         int previousIndex = i - 1;
         if (previousIndex < 0)
            previousIndex = points.size() - 1;

         edges.add(new LineSegment2D(points.get(previousIndex), points.get(i)));
      }

      return edges;
   }

   static List<Point2DReadOnly> getClosestPointOnEachCluster(Point2DReadOnly pointInWorld, List<Cluster> clusters)
   {
      List<Point2DReadOnly> closestClusterPoints = new ArrayList<>();

      for (Cluster cluster : clusters)
      {
         ExtrusionHull clusterPolygon = cluster.getNonNavigableExtrusionsInWorld2D();

         Point2D closestPointInCluster = new Point2D();
         VisibilityTools.distanceToCluster(pointInWorld, clusterPolygon, closestPointInCluster, null);
         closestClusterPoints.add(closestPointInCluster);
      }

      removeDuplicated2DPointsFromList(closestClusterPoints, samePointEpsilon);

      return closestClusterPoints;
   }

   static void collapseNearbyPoints(List<Point2D> points, double waypointResolution)
   {
      int intermediateWaypointIndex = 0;
      while (intermediateWaypointIndex < points.size() - 1)
      {
         Point2D thisWaypoint = points.get(intermediateWaypointIndex);
         Point2D nextWaypoint = points.get(intermediateWaypointIndex + 1);
         if (thisWaypoint.distance(nextWaypoint) < waypointResolution)
         { // collapse with the next one
            thisWaypoint.interpolate(thisWaypoint, nextWaypoint, 0.5);
            points.remove(intermediateWaypointIndex + 1);
         }
         else
         {
            intermediateWaypointIndex++;
         }
      }
   }

   static List<Point3D> computeIntermediateWaypointsToAddToAvoidObstacles(Point2DReadOnly originPointInWorld, Point2DReadOnly nextPointInWorld,
                                                                          VisibilityGraphNode connectionStartNode, VisibilityGraphNode connectionEndNode,
                                                                          double desiredDistanceFromObstacleCluster, double waypointResolution)
   {
      Set<NavigableRegion> navigableRegionsToSearch = new HashSet<>();
      NavigableRegion startRegion = connectionStartNode.getVisibilityGraphNavigableRegion().getNavigableRegion();
      NavigableRegion endRegion = connectionEndNode.getVisibilityGraphNavigableRegion().getNavigableRegion();
      navigableRegionsToSearch.add(startRegion);
      navigableRegionsToSearch.add(endRegion);

      return computeIntermediateWaypointsToAddToAvoidObstacles(originPointInWorld, nextPointInWorld, navigableRegionsToSearch,
                                                               desiredDistanceFromObstacleCluster, waypointResolution);
   }

   static List<Point3D> computeIntermediateWaypointsToAddToAvoidObstacles(Point2DReadOnly originPointInWorld2D, Point2DReadOnly nextPointInWorld2D,
                                                                          Collection<NavigableRegion> navigableRegionsToSearch,
                                                                          double desiredDistanceFromObstacleCluster, double waypointResolution)
   {
      List<Point3D> intermediateWaypoints3DToAdd = new ArrayList<>();

      if (originPointInWorld2D.distance(nextPointInWorld2D) < 2.0 * waypointResolution)
         return intermediateWaypoints3DToAdd;

      List<Point2D> intermediateWaypointsToAdd = new ArrayList<>();

      for (NavigableRegion navigableRegion : navigableRegionsToSearch)
      {
         for (Cluster cluster : navigableRegion.getObstacleClusters())
         {
            ExtrusionHull clusterPolygon = cluster.getNonNavigableExtrusionsInWorld2D();
            boolean isClosed = cluster.isClosed();

            Point2D closestPointInCluster = new Point2D();
            Point2D closestPointOnConnection = new Point2D();

            double connectionDistanceToObstacle = VisibilityTools
                  .distanceToCluster(originPointInWorld2D, nextPointInWorld2D, clusterPolygon, closestPointOnConnection, closestPointInCluster, null, isClosed);

            // only add the point if it's close to the obstacle cluster, and don't add if it's already been added.
            if (connectionDistanceToObstacle < desiredDistanceFromObstacleCluster && !intermediateWaypointsToAdd.contains(closestPointOnConnection))
               intermediateWaypointsToAdd.add(closestPointOnConnection);
         }
      }

      // sort the points by their percentage along the line segment to make sure they get added in order.
      WaypointComparator comparator = new WaypointComparator();
      comparator.setStartPoint(originPointInWorld2D);
      comparator.setEndPoint(nextPointInWorld2D);
      intermediateWaypointsToAdd.sort(comparator);

      // collapse intermediate waypoints
      collapseNearbyPoints(intermediateWaypointsToAdd, waypointResolution);

      for (Point2D intermediateWaypoint : intermediateWaypointsToAdd)
      {
         double heightOfPoint = findHeightOfPoint(intermediateWaypoint, navigableRegionsToSearch);
         Point3D pointIn3D = new Point3D(intermediateWaypoint.getX(), intermediateWaypoint.getY(), heightOfPoint);
         intermediateWaypoints3DToAdd.add(pointIn3D);
      }

      return intermediateWaypoints3DToAdd;
   }
}
