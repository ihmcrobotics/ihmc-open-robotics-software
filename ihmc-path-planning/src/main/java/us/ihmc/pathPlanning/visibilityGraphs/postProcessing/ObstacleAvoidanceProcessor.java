package us.ihmc.pathPlanning.visibilityGraphs.postProcessing;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegions;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityGraphNode;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapSolution;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityTools;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;

public class ObstacleAvoidanceProcessor
{
   private static final double minDistanceToMove = 0.01;
   private static final double cliffHeightToAvoid = 0.10;
   private static final double samePointEpsilon = 0.01;

   private final double desiredDistanceFromObstacleCluster;
   private final IntermediateComparator comparator = new IntermediateComparator();

   public ObstacleAvoidanceProcessor(VisibilityGraphsParameters parameters)
   {
      desiredDistanceFromObstacleCluster = parameters.getPreferredObstacleExtrusionDistance() - parameters.getObstacleExtrusionDistance();
   }

   public List<Point3DReadOnly> computePathFromNodes(List<VisibilityGraphNode> nodePath, VisibilityMapSolution visibilityMapSolution)
   {
      List<Point3D> newPath = nodePath.parallelStream().map(node -> new Point3D(node.getPointInWorld())).collect(Collectors.toList());

      int pathNodeIndex = 0;
      int waypointIndex = 0;
      // don't do the goal node
      while (pathNodeIndex < nodePath.size() - 1)
      {
         int nextPathNodeIndex = pathNodeIndex + 1;
         int nextWaypointIndex = waypointIndex + 1;

         Point3D startPointInWorld = newPath.get(waypointIndex);
         Point3D endPointInWorld = newPath.get(nextWaypointIndex);

         VisibilityGraphNode startVisGraphNode = nodePath.get(pathNodeIndex);
         VisibilityGraphNode endVisGraphNode = nodePath.get(nextPathNodeIndex);

         boolean isGoalNode = pathNodeIndex > nodePath.size() - 2;

         if (!isGoalNode)
         {
            adjustGoalNodePositionToAvoidObstacles(endPointInWorld, endVisGraphNode, visibilityMapSolution.getNavigableRegions());
            // run it again, in case moving it moved it to be within the range of another one.
            adjustGoalNodePositionToAvoidObstacles(endPointInWorld, endVisGraphNode, visibilityMapSolution.getNavigableRegions());
         }

         List<Point3D> intermediateWaypointsToAdd = computeIntermediateWaypointsToAddToAvoidObstacles(new Point2D(startPointInWorld),
                                                                                                      new Point2D(endPointInWorld), startVisGraphNode,
                                                                                                      endVisGraphNode);

         // shift all the points around
         for (Point3D intermediateWaypointToAdd : intermediateWaypointsToAdd)
         {
            adjustGoalNodePositionToAvoidObstacles(intermediateWaypointToAdd, endVisGraphNode, visibilityMapSolution.getNavigableRegions());
            // run it again, in case moving it moved it to be within the range of another one.
            adjustGoalNodePositionToAvoidObstacles(intermediateWaypointToAdd, endVisGraphNode, visibilityMapSolution.getNavigableRegions());
         }

         // prune duplicated points
         removeDuplicated3DPointsFromList(intermediateWaypointsToAdd, samePointEpsilon);

         // add the new points to the path
         for (Point3D intermediateWaypointToAdd : intermediateWaypointsToAdd)
         {
            if (intermediateWaypointToAdd.distance(startPointInWorld) > samePointEpsilon
                  && intermediateWaypointToAdd.distance(endPointInWorld) > samePointEpsilon)
            {
               waypointIndex++;
               newPath.add(waypointIndex, intermediateWaypointToAdd);
            }
         }

         waypointIndex++;
         pathNodeIndex++;
      }

      return newPath.parallelStream().map(Point3D::new).collect(Collectors.toList());
   }

   private void adjustGoalNodePositionToAvoidObstacles(Point3D nodeLocationToPack, VisibilityGraphNode node, NavigableRegions navigableRegions)
   {
      Point2D nextPointInWorld2D = new Point2D(nodeLocationToPack);

      NavigableRegion navigableRegion = node.getVisibilityGraphNavigableRegion().getNavigableRegion();

      Vector2DReadOnly nodeShiftToAvoidObstacles = getDirectionAndDistanceToShiftNodeToAvoidObstacles(nextPointInWorld2D, navigableRegion);

      if (nodeShiftToAvoidObstacles.length() > minDistanceToMove)
      {
         nextPointInWorld2D.add(nodeShiftToAvoidObstacles);

         Vector2DReadOnly extraShiftToAvoidCliffs = getDirectionAndDistanceToShiftToAvoidCliffs(nextPointInWorld2D, node, navigableRegions, 0.1);

         if (extraShiftToAvoidCliffs.length() > minDistanceToMove)
            nextPointInWorld2D.add(extraShiftToAvoidCliffs);

         double newHeight = navigableRegion.getPlaneZGivenXY(nextPointInWorld2D.getX(), nextPointInWorld2D.getY());
         nodeLocationToPack.set(nextPointInWorld2D, newHeight);
      }
   }

   private List<Point3D> computeIntermediateWaypointsToAddToAvoidObstacles(Point2DReadOnly originPointInWorld2D, Point2DReadOnly nextPointInWorld2D,
                                                                           VisibilityGraphNode connectionStartNode, VisibilityGraphNode connectionEndNode)
   {
      List<NavigableRegion> navigableRegionsToSearch = new ArrayList<>();
      NavigableRegion startRegion = connectionStartNode.getVisibilityGraphNavigableRegion().getNavigableRegion();
      NavigableRegion endRegion = connectionEndNode.getVisibilityGraphNavigableRegion().getNavigableRegion();
      navigableRegionsToSearch.add(startRegion);
      if (!startRegion.equals(endRegion))
         navigableRegionsToSearch.add(endRegion);

      return computeIntermediateWaypointsToAddToAvoidObstacles(originPointInWorld2D, nextPointInWorld2D, navigableRegionsToSearch);
   }

   private List<Point3D> computeIntermediateWaypointsToAddToAvoidObstacles(Point2DReadOnly originPointInWorld2D, Point2DReadOnly nextPointInWorld2D,
                                                                           List<NavigableRegion> navigableRegionsToSearch)
   {
      List<Point2D> intermediateWaypointsToAdd = new ArrayList<>();
      HashMap<Point2D, List<Point2D>> nearbyObstacleCollisions = new HashMap<>();

      for (NavigableRegion navigableRegion : navigableRegionsToSearch)
      {
         for (Cluster cluster : navigableRegion.getObstacleClusters())
         {
            List<Point2DReadOnly> clusterPolygon = cluster.getNonNavigableExtrusionsInWorld2D();
            boolean isClosed = cluster.isClosed();

            Point2D closestPointInCluster = new Point2D();
            Point2D closestPointOnConnection = new Point2D();

            Vector2D clusterNormal = new Vector2D();

            double connectionDistanceToObstacle = VisibilityTools
                  .distanceToCluster(originPointInWorld2D, nextPointInWorld2D, clusterPolygon, closestPointOnConnection, closestPointInCluster, clusterNormal,
                                     isClosed);
            if (connectionDistanceToObstacle < desiredDistanceFromObstacleCluster)
            {
               if (!intermediateWaypointsToAdd.contains(closestPointOnConnection))
               {
                  intermediateWaypointsToAdd.add(closestPointOnConnection);
                  nearbyObstacleCollisions.put(closestPointOnConnection, new ArrayList<>());
               }
               nearbyObstacleCollisions.get(closestPointOnConnection).add(closestPointInCluster);
            }
         }
      }

      comparator.setStartPoint(originPointInWorld2D);
      comparator.setEndPoint(nextPointInWorld2D);
      intermediateWaypointsToAdd.sort(comparator);

      // collapse intermediate waypoints
      int intermediateWaypointIndex = 0;
      while (intermediateWaypointIndex < intermediateWaypointsToAdd.size() - 1)
      {
         Point2D thisWaypoint = intermediateWaypointsToAdd.get(intermediateWaypointIndex);
         Point2D nextWaypoint = intermediateWaypointsToAdd.get(intermediateWaypointIndex + 1);
         if (thisWaypoint.distance(nextWaypoint) < 0.05)
         { // collapse with the next one
            thisWaypoint.interpolate(thisWaypoint, nextWaypoint, 0.5);
            nearbyObstacleCollisions.get(thisWaypoint).addAll(nearbyObstacleCollisions.get(nextWaypoint));
            nearbyObstacleCollisions.remove(nextWaypoint);
         }
         else
         {
            intermediateWaypointIndex++;
         }
      }

      // collapse nearby collisions
      for (Point2D intermediateWaypoint : intermediateWaypointsToAdd)
      {
         removeDuplicated2DPointsFromList(nearbyObstacleCollisions.get(intermediateWaypoint), samePointEpsilon);
      }

      List<Point3D> intermediateWaypoints3DToAdd = new ArrayList<>();

      // compute 2D waypoints with offsets
      for (Point2D intermediateWaypoint : intermediateWaypointsToAdd)
      {
         List<Point2D> nearbyCollisions = nearbyObstacleCollisions.get(intermediateWaypoint);
         Vector2D waypointTotalOffset = new Vector2D();
         for (Point2D nearbyCollision : nearbyCollisions)
         {
            Vector2D nodeOffset = new Vector2D();
            nodeOffset.sub(intermediateWaypoint, nearbyCollision);

            double distanceFromCluster = nodeOffset.length();
            double distanceToMove = desiredDistanceFromObstacleCluster - distanceFromCluster;
            nodeOffset.scale(distanceToMove / distanceFromCluster);

            waypointTotalOffset.add(nodeOffset);
         }
         waypointTotalOffset.scale(1.0 / nearbyCollisions.size());

         intermediateWaypoint.add(waypointTotalOffset);
      }

      for (Point2D intermediateWaypoint : intermediateWaypointsToAdd)
      {
         double maxHeight = Double.NEGATIVE_INFINITY;
         for (NavigableRegion navigableRegion : navigableRegionsToSearch)
         {
            double height = navigableRegion.getPlaneZGivenXY(intermediateWaypoint.getX(), intermediateWaypoint.getY());
            if (height > maxHeight)
               maxHeight = height;
         }

         intermediateWaypoints3DToAdd.add(new Point3D(intermediateWaypoint.getX(), intermediateWaypoint.getY(), maxHeight));
      }

      return intermediateWaypoints3DToAdd;
   }

   private Vector2DReadOnly getDirectionAndDistanceToShiftNodeToAvoidObstacles(Point2DReadOnly pointInWorldToCheck, NavigableRegion navigableRegion)
   {
      Vector2D nodeShift = new Vector2D();
      int numberOfShifts = 0;

      for (Cluster cluster : navigableRegion.getObstacleClusters())
      {
         List<Point2DReadOnly> clusterPolygon = cluster.getNonNavigableExtrusionsInWorld2D();

         Point2D closestPointInCluster = new Point2D();
         double distanceToCluster = VisibilityTools.distanceToCluster(pointInWorldToCheck, clusterPolygon, closestPointInCluster, null);
         if (distanceToCluster < 1.1 * desiredDistanceFromObstacleCluster)
         {
            double distanceToMove = desiredDistanceFromObstacleCluster - distanceToCluster;
            Vector2D nodeOffset = new Vector2D();
            nodeOffset.sub(pointInWorldToCheck, closestPointInCluster);
            nodeOffset.normalize();
            nodeOffset.scale(distanceToMove);

            nodeShift.add(nodeOffset);

            numberOfShifts += 1;
         }
      }

      nodeShift.scale(1.0 / numberOfShifts);

      return nodeShift;
   }

   private static Vector2DReadOnly getDirectionAndDistanceToShiftToAvoidCliffs(Point2DReadOnly pointToCheck, VisibilityGraphNode originalNode,
                                                                               NavigableRegions navigableRegions, double distanceInsideHomeRegionToAvoid)
   {
      double originalHeight = originalNode.getPointInWorld().getZ();
      double adjustedHeight = getMaxHeightOfPointInWorld(pointToCheck, navigableRegions);

      Vector2D nodeShiftToAvoidCliff = new Vector2D();

      if (Math.abs(adjustedHeight - originalHeight) > cliffHeightToAvoid)
      {
         Cluster cluster = originalNode.getVisibilityGraphNavigableRegion().getNavigableRegion().getHomeRegionCluster();
         List<Point2DReadOnly> clusterPolygon = cluster.getNonNavigableExtrusionsInWorld2D();

         Point2D closestPointInCluster = new Point2D();
         double distanceToCluster = VisibilityTools.distanceToCluster(pointToCheck, clusterPolygon, closestPointInCluster, null);

         double distanceToMove = distanceToCluster + distanceInsideHomeRegionToAvoid;
         nodeShiftToAvoidCliff.sub(closestPointInCluster, pointToCheck);
         nodeShiftToAvoidCliff.normalize();
         nodeShiftToAvoidCliff.scale(distanceToMove);
      }

      return nodeShiftToAvoidCliff;
   }

   private static double getMaxHeightOfPointInWorld(Point2DReadOnly pointInWorldToCheck, NavigableRegions navigableRegions)
   {
      double maxHeight = Double.NEGATIVE_INFINITY;
      for (NavigableRegion navigableRegion : navigableRegions.getNaviableRegionsList())
      {
         PlanarRegion planarRegion = navigableRegion.getHomePlanarRegion();

         RigidBodyTransform transformToWorld = new RigidBodyTransform();
         planarRegion.getTransformToWorld(transformToWorld);
         Point2D pointInLocalToCheck = new Point2D(pointInWorldToCheck);
         pointInLocalToCheck.applyInverseTransform(transformToWorld, false);
         double height = planarRegion.getPlaneZGivenXY(pointInWorldToCheck.getX(), pointInWorldToCheck.getY());
         if (PlanarRegionTools.isPointInLocalInsidePlanarRegion(navigableRegion.getHomePlanarRegion(), pointInLocalToCheck) && height > maxHeight)
         {
            maxHeight = height;
         }
      }

      return maxHeight;
   }

   private static void removeDuplicated2DPointsFromList(List<? extends Point2DReadOnly> listOfPoints, double samePointEpsilon)
   {
      int pointIndex = 0;
      while (pointIndex < listOfPoints.size() - 1)
      {
         if (listOfPoints.get(pointIndex).distance(listOfPoints.get(pointIndex + 1)) < samePointEpsilon)
            listOfPoints.remove(pointIndex + 1);
         else
            pointIndex++;
      }
   }

   private static void removeDuplicated3DPointsFromList(List<? extends Point3DReadOnly> listOfPoints, double samePointEpsilon)
   {
      int pointIndex = 0;
      while (pointIndex < listOfPoints.size() - 1)
      {
         if (listOfPoints.get(pointIndex).distance(listOfPoints.get(pointIndex + 1)) < samePointEpsilon)
            listOfPoints.remove(pointIndex + 1);
         else
            pointIndex++;
      }
   }

   private class IntermediateComparator implements Comparator<Point2DReadOnly>
   {
      private final Point2D startPoint = new Point2D();
      private final Point2D endPoint = new Point2D();

      public void setStartPoint(Point2DReadOnly startPoint)
      {
         this.startPoint.set(startPoint);
      }

      public void setEndPoint(Point2DReadOnly endPoint)
      {
         this.endPoint.set(endPoint);
      }

      @Override
      public int compare(Point2DReadOnly pointA, Point2DReadOnly pointB)
      {
         double distanceA = EuclidGeometryTools.percentageAlongLineSegment2D(pointA, startPoint, endPoint);
         double distanceB = EuclidGeometryTools.percentageAlongLineSegment2D(pointB, startPoint, endPoint);
         return Double.compare(distanceA, distanceB);
      }
   }
}
