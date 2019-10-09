package us.ihmc.pathPlanning.visibilityGraphs.postProcessing;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegions;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityGraphNode;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapSolution;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;

public class ObstacleAndCliffAvoidanceProcessor
{
   private static final double minDistanceToMove = 0.01;
   private static final double cliffHeightToAvoid = 0.10;
   private static final double samePointEpsilon = 0.01;

   private double desiredDistanceFromObstacleCluster;
   private double minimumDistanceFromObstacleCluster;
   private double desiredDistanceFromCliff;
   private double minimumDistanceFromCliff; // FIXME this is currently unused
   private double maxInterRegionConnectionLength;
   private final double waypointResolution;
   private final IntermediateComparator comparator = new IntermediateComparator();

   private final List<ObstacleAndCliffAvoidanceInfo> pointInfos = new ArrayList<>();

   private final VisibilityGraphsParametersReadOnly parameters;

   public ObstacleAndCliffAvoidanceProcessor(VisibilityGraphsParametersReadOnly parameters)
   {
      this.parameters = parameters;
      desiredDistanceFromObstacleCluster = parameters.getPreferredObstacleExtrusionDistance() - parameters.getObstacleExtrusionDistance();
      maxInterRegionConnectionLength = parameters.getMaxInterRegionConnectionLength();
      desiredDistanceFromCliff = parameters.getPreferredObstacleExtrusionDistance() - parameters.getNavigableExtrusionDistance();
      minimumDistanceFromCliff = parameters.getNavigableExtrusionDistance();
      minimumDistanceFromObstacleCluster = 0.0;
      waypointResolution = 0.1;
   }

   public List<Point3DReadOnly> computePathFromNodes(List<VisibilityGraphNode> nodePath, VisibilityMapSolution visibilityMapSolution)
   {
      updateParameters();
      pointInfos.clear();

      List<Point3D> newPathPositions = nodePath.stream().map(node -> new Point3D(node.getPointInWorld())).collect(Collectors.toList());
      newPathPositions.forEach(point ->
                                        {
                                           ObstacleAndCliffAvoidanceInfo info = new ObstacleAndCliffAvoidanceInfo();
                                           info.setOriginalPosition(point);
                                           pointInfos.add(info);
                                        });

      int pathNodeIndex = 0;
      int waypointIndex = 0;
      // don't do the goal node
      while (pathNodeIndex < nodePath.size() - 1 && parameters.getPerformPostProcessingNodeShifting())
      {
         int nextPathNodeIndex = pathNodeIndex + 1;
         int nextWaypointIndex = waypointIndex + 1;

         Point3D startPointInWorld = newPathPositions.get(waypointIndex);
         Point3D endPointInWorld = newPathPositions.get(nextWaypointIndex);

         ObstacleAndCliffAvoidanceInfo endPointInfo = null;
         if (nextWaypointIndex < pointInfos.size() )
            endPointInfo = pointInfos.get(nextWaypointIndex);

         VisibilityGraphNode startVisGraphNode = nodePath.get(pathNodeIndex);
         VisibilityGraphNode endVisGraphNode = nodePath.get(nextPathNodeIndex);

         boolean isGoalNode = nextWaypointIndex > newPathPositions.size() - 2;

         NavigableRegion startingRegion = startVisGraphNode.getVisibilityGraphNavigableRegion().getNavigableRegion();
         NavigableRegion endingRegion = endVisGraphNode.getVisibilityGraphNavigableRegion().getNavigableRegion();
         NavigableRegions allNavigableRegions = visibilityMapSolution.getNavigableRegions();

         if (!isGoalNode)
         {
            adjustNodePositionToAvoidObstaclesAndCliffs(endPointInfo, endPointInWorld, startingRegion, endingRegion, allNavigableRegions);
         }

         /*
         if (parameters.getIntroduceMidpointsInPostProcessing())
         {
            List<Point3D> intermediateWaypointsToAdd = computeIntermediateWaypointsToAddToAvoidObstacles(new Point2D(startPointInWorld),
                                                                                                         new Point2D(endPointInWorld), startVisGraphNode,
                                                                                                         endVisGraphNode);
            removeDuplicated3DPointsFromList(intermediateWaypointsToAdd, waypointResolution);
            removeDuplicateStartOrEndPointsFromList(intermediateWaypointsToAdd, startPointInWorld, endPointInWorld, waypointResolution);

            for (Point3D intermediateWaypointToAdd : intermediateWaypointsToAdd)
            {
               waypointIndex++;
               newPathPositions.add(waypointIndex, intermediateWaypointToAdd);
               ObstacleAndCliffAvoidanceInfo pointInfo = new ObstacleAndCliffAvoidanceInfo();
               pointInfo.setWasIntroduced(true);
               pointInfos.add(waypointIndex, pointInfo);
            }

            // shift all the points around
            for (Point3D intermediateWaypointToAdd : intermediateWaypointsToAdd)
            {
               adjustNodePositionToAvoidObstaclesAndCliffs(null, intermediateWaypointToAdd, startingRegion, endingRegion, allNavigableRegions);
            }

//             prune duplicated points
            removeDuplicated3DPointsFromList(intermediateWaypointsToAdd, waypointResolution);
            removeDuplicateStartOrEndPointsFromList(intermediateWaypointsToAdd, startPointInWorld, endPointInWorld, waypointResolution);

         }
         */

         waypointIndex++;
         pathNodeIndex++;
      }

      return newPathPositions.parallelStream().map(Point3D::new).collect(Collectors.toList());
   }

   private void updateParameters()
   {
      desiredDistanceFromObstacleCluster = parameters.getPreferredObstacleExtrusionDistance() - parameters.getObstacleExtrusionDistance();
      maxInterRegionConnectionLength = parameters.getMaxInterRegionConnectionLength();
      desiredDistanceFromCliff = parameters.getPreferredObstacleExtrusionDistance() - parameters.getNavigableExtrusionDistance();
      minimumDistanceFromCliff = parameters.getNavigableExtrusionDistance();
   }

   private double adjustNodePositionToAvoidObstaclesAndCliffs(ObstacleAndCliffAvoidanceInfo pointInfo, Point3D nodeLocationToPack, NavigableRegion startRegion,
                                                              NavigableRegion endRegion, NavigableRegions allNavigableRegions)
   {
      Point2D nextPointInWorld2D = new Point2D(nodeLocationToPack);

      List<Cluster> obstacleClusters = new ArrayList<>();
      for (Cluster potentialCluster : startRegion.getObstacleClusters())
      {
         if (potentialCluster.getRawPointsInLocal3D().stream().anyMatch(point -> point.getZ() > parameters.getTooHighToStepDistance()))
            obstacleClusters.add(potentialCluster);
      }
      if (!startRegion.equals(endRegion))
      {
         for (Cluster potentialCluster : endRegion.getObstacleClusters())
         {
            if (potentialCluster.getRawPointsInLocal3D().stream().anyMatch(point -> point.getZ() > parameters.getTooHighToStepDistance()))
               obstacleClusters.add(potentialCluster);
         }
      }

      List<Point2DReadOnly> closestObstacleClusterPoints = getClosestPointOnEachCluster(nextPointInWorld2D, obstacleClusters);
      Vector2DReadOnly nodeShiftToAvoidObstacles = PointWiggler.computeBestShiftVectorToAvoidPoints(nextPointInWorld2D, closestObstacleClusterPoints,
                                                                                                    desiredDistanceFromObstacleCluster, minimumDistanceFromObstacleCluster);

      if (pointInfo != null)
      {
         pointInfo.setClosestObstacleClusterPoints(closestObstacleClusterPoints);
         pointInfo.setShiftToAvoidObstacles(nodeShiftToAvoidObstacles);
      }

      if (nodeShiftToAvoidObstacles.containsNaN())
         nodeShiftToAvoidObstacles = new Vector2D();

      Point2D shiftedPoint = new Point2D(nodeLocationToPack);
      shiftedPoint.add(nodeShiftToAvoidObstacles);

      List<NavigableRegion> bothRegions = new ArrayList<>();
      bothRegions.add(startRegion);
      bothRegions.add(endRegion);

      List<LineSegment2DReadOnly> cliffEdges = new ArrayList<>();
      boolean isShiftedPointNearACliff = isNearCliff(shiftedPoint, maxInterRegionConnectionLength, cliffHeightToAvoid, endRegion,
                                                     allNavigableRegions.getNaviableRegionsList(), cliffEdges);

      Vector2D nodeShift = new Vector2D();

      if (isShiftedPointNearACliff)
      {
         List<Point2DReadOnly> closestCliffObstacleClusterPoints = new ArrayList<>();


         closestCliffObstacleClusterPoints.addAll(getPointsAlongEdgeOfClusterClosestToPoint(nextPointInWorld2D, endRegion.getHomeRegionCluster()));
         if (startRegion != endRegion && EuclidGeometryPolygonTools.isPoint2DInsideConvexPolygon2D(nextPointInWorld2D, startRegion.getHomeRegionCluster().getRawPointsInWorld2D(),
                                                                                                   startRegion.getHomeRegionCluster().getNumberOfRawPoints(), true, 0.0))
         {
            closestCliffObstacleClusterPoints.addAll(getPointsAlongEdgeOfClusterClosestToPoint(nextPointInWorld2D, startRegion.getHomeRegionCluster()));
         }


         Vector2DReadOnly newShift = PointWiggler.computeBestShiftVectorToAvoidPoints(nextPointInWorld2D, closestObstacleClusterPoints, closestCliffObstacleClusterPoints,
                                                                                      desiredDistanceFromObstacleCluster, desiredDistanceFromCliff, minimumDistanceFromObstacleCluster, minimumDistanceFromCliff);
         if (newShift.containsNaN())
            newShift = new Vector2D();

         nodeShift.set(newShift);

         if (pointInfo != null)
         {
            pointInfo.setClosestCliffClusterPoints(closestCliffObstacleClusterPoints);
            pointInfo.setShiftToAvoidObstaclesAndCliffs(newShift);
         }
      }
      else
      {
         nodeShift.set(nodeShiftToAvoidObstacles);
      }

      if (nodeShift.length() > minDistanceToMove)
      {
         nextPointInWorld2D.add(nodeShift);
         nodeLocationToPack.set(nextPointInWorld2D, findHeightOfPoint(nextPointInWorld2D, bothRegions));
      }

      double distanceToClosestObstacle = Double.POSITIVE_INFINITY;
      for (Point2DReadOnly obstaclePoint : closestObstacleClusterPoints)
         distanceToClosestObstacle = Math.min(distanceToClosestObstacle, nodeLocationToPack.distanceXY(obstaclePoint));

      return distanceToClosestObstacle + parameters.getObstacleExtrusionDistance();
   }




   private static List<Point2DReadOnly> getPointsAlongEdgeOfClusterClosestToPoint(Point2DReadOnly pointToCheck, Cluster cluster)
   {
      return getPointsAlongEdgeOfClusterClosestToPoint(pointToCheck, cluster.getRawPointsInWorld2D());
   }

   private static List<Point2DReadOnly> getPointsAlongEdgeOfClusterClosestToPoint(Point2DReadOnly pointToCheck, List<Point2DReadOnly> points)
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

   private static List<LineSegment2DReadOnly> getClusterEdges(Cluster cluster)
   {
      return getClusterEdges(cluster.getRawPointsInWorld2D());
   }

   private static List<LineSegment2DReadOnly> getClusterEdges(List<Point2DReadOnly> points)
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


   private List<Point3D> computeIntermediateWaypointsToAddToAvoidObstacles(Point2DReadOnly originPointInWorld, Point2DReadOnly nextPointInWorld,
                                                                           VisibilityGraphNode connectionStartNode, VisibilityGraphNode connectionEndNode)
   {
      List<NavigableRegion> navigableRegionsToSearch = new ArrayList<>();
      NavigableRegion startRegion = connectionStartNode.getVisibilityGraphNavigableRegion().getNavigableRegion();
      NavigableRegion endRegion = connectionEndNode.getVisibilityGraphNavigableRegion().getNavigableRegion();
      navigableRegionsToSearch.add(startRegion);
      if (!startRegion.equals(endRegion))
         navigableRegionsToSearch.add(endRegion);

      return computeIntermediateWaypointsToAddToAvoidObstacles(originPointInWorld, nextPointInWorld, navigableRegionsToSearch);
   }

   private List<Point3D> computeIntermediateWaypointsToAddToAvoidObstacles(Point2DReadOnly originPointInWorld2D, Point2DReadOnly nextPointInWorld2D,
                                                                           List<NavigableRegion> navigableRegionsToSearch)
   {
      List<Point2D> intermediateWaypointsToAdd = new ArrayList<>();

      for (NavigableRegion navigableRegion : navigableRegionsToSearch)
      {
         for (Cluster cluster : navigableRegion.getObstacleClusters())
         {
            List<Point2DReadOnly> clusterPolygon = cluster.getNonNavigableExtrusionsInWorld2D();
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
      comparator.setStartPoint(originPointInWorld2D);
      comparator.setEndPoint(nextPointInWorld2D);
      intermediateWaypointsToAdd.sort(comparator);

      // collapse intermediate waypoints
      int intermediateWaypointIndex = 0;
      while (intermediateWaypointIndex < intermediateWaypointsToAdd.size() - 1)
      {
         Point2D thisWaypoint = intermediateWaypointsToAdd.get(intermediateWaypointIndex);
         Point2D nextWaypoint = intermediateWaypointsToAdd.get(intermediateWaypointIndex + 1);
         if (thisWaypoint.distance(nextWaypoint) < waypointResolution)
         { // collapse with the next one
            thisWaypoint.interpolate(thisWaypoint, nextWaypoint, 0.5);
            intermediateWaypointsToAdd.remove(intermediateWaypointIndex + 1);
         }
         else
         {
            intermediateWaypointIndex++;
         }
      }

      List<Point3D> intermediateWaypoints3DToAdd = new ArrayList<>();
      for (Point2D intermediateWaypoint : intermediateWaypointsToAdd)
      {
         double heightOfPoint = findHeightOfPoint(intermediateWaypoint, navigableRegionsToSearch);
         Point3D pointIn3D = new Point3D(intermediateWaypoint.getX(), intermediateWaypoint.getY(), heightOfPoint);
         intermediateWaypoints3DToAdd.add(pointIn3D);
      }

      return intermediateWaypoints3DToAdd;
   }

   private static List<Point2DReadOnly> getClosestPointOnEachCluster(Point2DReadOnly pointInWorld, List<Cluster> clusters)
   {
      List<Point2DReadOnly> closestClusterPoints = new ArrayList<>();

      for (Cluster cluster : clusters)
      {
         List<Point2DReadOnly> clusterPolygon = cluster.getNonNavigableExtrusionsInWorld2D();

         Point2D closestPointInCluster = new Point2D();
         VisibilityTools.distanceToCluster(pointInWorld, clusterPolygon, closestPointInCluster, null);
         closestClusterPoints.add(closestPointInCluster);
      }

      removeDuplicated2DPointsFromList(closestClusterPoints, samePointEpsilon);

      return closestClusterPoints;
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

   private static double findHeightOfPoint(Point2DReadOnly pointInWorld, List<NavigableRegion> navigableRegions)
   {
      return findHeightOfPoint(pointInWorld.getX(), pointInWorld.getY(), navigableRegions);
   }

   private static double findHeightOfPoint(double pointX, double pointY, List<NavigableRegion> navigableRegions)
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

   // FIXME this is not working, clearly.
   /** This is the point shifted as if it didn't have to worry about cliff avoidance. */
   private boolean isNearCliff(Point2DReadOnly shiftedPoint, double maxConnectionDistance, double maxHeightDelta,
                               NavigableRegion homeRegion, List<NavigableRegion> navigableRegions, List<LineSegment2DReadOnly> cliffEdgesToPack)
   {
      return isNearCliff(shiftedPoint, maxConnectionDistance, maxHeightDelta, parameters.getPreferredObstacleExtrusionDistance(), homeRegion, navigableRegions, cliffEdgesToPack);
   }

   static boolean isNearCliff(Point2DReadOnly shiftedPoint, double maxConnectionDistance, double maxHeightDelta, double desiredDistanceFromCliff,
                                      NavigableRegion homeRegion, List<NavigableRegion> navigableRegions, List<LineSegment2DReadOnly> cliffEdgesToPack)
   {
      // if point is sufficiently inside, it is not near a cliff
      Point2D closestPointToThrowAway = new Point2D();
      Cluster homeCluster = homeRegion.getHomeRegionCluster();
      double distanceToContainingCluster = VisibilityTools
            .distanceToCluster(shiftedPoint, homeCluster.getNavigableExtrusionsInWorld2D(), closestPointToThrowAway, null);

      if (distanceToContainingCluster < -desiredDistanceFromCliff)
         return false;

      List<LineSegment2DReadOnly> homeRegionEdgesContainingPoint = getNearbyEdges(shiftedPoint, homeCluster, desiredDistanceFromCliff);

      List<NavigableRegion> nearbyRegions = filterNavigableRegionsWithBoundingCircle(shiftedPoint, maxConnectionDistance + desiredDistanceFromCliff, navigableRegions);
      List<NavigableRegion> closeEnoughRegions = filterNavigableRegionsConnectionWithDistanceAndHeightChange(homeRegion, nearbyRegions, maxConnectionDistance,
                                                                                                             maxHeightDelta);

      closeEnoughRegions.remove(homeRegion);

      List<LineSegment2DReadOnly> closeEnoughEdges = new ArrayList<>();
      for (NavigableRegion closeEnoughRegion : closeEnoughRegions)
      {
         closeEnoughEdges.addAll(getNearbyEdges(shiftedPoint, closeEnoughRegion.getHomeRegionCluster(), desiredDistanceFromCliff));
      }


      return isAnyEdgeNearACliff(homeRegionEdgesContainingPoint, closeEnoughEdges, cliffEdgesToPack);
   }

   private static boolean isAnyEdgeNearACliff(List<LineSegment2DReadOnly> edgesToCheck, List<LineSegment2DReadOnly> closeEnoughEdges, List<LineSegment2DReadOnly> cliffEdgesToPack)
   {
      if (closeEnoughEdges.isEmpty())
      {
         cliffEdgesToPack.addAll(edgesToCheck);
         return !cliffEdgesToPack.isEmpty();
      }

      HashMap<LineSegment2DReadOnly, Vector2DReadOnly> edgeNormals = computeEdgeNormals(edgesToCheck);

      HashMap<LineSegment2DReadOnly, Vector2DReadOnly> closeEnoughEdgeNormals = computeEdgeNormals(closeEnoughEdges);

      for (LineSegment2DReadOnly edge : edgesToCheck)
      {
         if (isEdgeNearACliff(edgeNormals.get(edge), closeEnoughEdges, closeEnoughEdgeNormals))
            cliffEdgesToPack.add(edge);
      }

      return !cliffEdgesToPack.isEmpty();
   }

   private static boolean isEdgeNearACliff(Vector2DReadOnly edgeNormal, List<LineSegment2DReadOnly> closeEnoughEdges,
                                           HashMap<LineSegment2DReadOnly, Vector2DReadOnly> closeEnoughEdgeNormals)
   {
      boolean homeEdgeIsACliff = false;

      for (LineSegment2DReadOnly closeEnoughEdge : closeEnoughEdges)
      {
         Vector2DReadOnly normalToCloseEnoughCluster = closeEnoughEdgeNormals.get(closeEnoughEdge);
         double angleBetween = normalToCloseEnoughCluster.angle(edgeNormal);
         if (Math.abs(angleBetween) > Math.toRadians(20.0))
         {
            homeEdgeIsACliff = true;
            break;
         }
      }

      return homeEdgeIsACliff;
   }


   static List<LineSegment2DReadOnly> getNearbyEdges(Point2DReadOnly pointToCheck, Cluster cluster, double distance)
   {
      List<LineSegment2DReadOnly> clusterEdges = getClusterEdges(cluster);
      return clusterEdges.stream().filter(edge -> edge.distance(pointToCheck) < distance).collect(Collectors.toList());
   }

   private static HashMap<LineSegment2DReadOnly, Vector2DReadOnly> computeEdgeNormals(List<LineSegment2DReadOnly> edges)
   {
      HashMap<LineSegment2DReadOnly, Vector2DReadOnly> edgeNormals = new HashMap<>();

      for (LineSegment2DReadOnly edge : edges)
      {
         edgeNormals.put(edge, computeEdgeNormal(edge));
      }

      return edgeNormals;
   }

   private static Vector2DReadOnly computeEdgeNormal(LineSegment2DReadOnly edge)
   {
      return computeEdgeNormal(edge.getFirstEndpointX(), edge.getFirstEndpointY(), edge.getSecondEndpointX(), edge.getSecondEndpointY());
   }

   private static Vector2DReadOnly computeEdgeNormal(LineSegment3DReadOnly edge)
   {
      return computeEdgeNormal(edge.getFirstEndpointX(), edge.getFirstEndpointY(), edge.getSecondEndpointX(), edge.getSecondEndpointY());
   }

   private static Vector2DReadOnly computeEdgeNormal(double firstX, double firstY, double secondX, double secondY)
   {
      Vector2D normal = new Vector2D();
      normal.set(secondX, secondY);
      normal.sub(firstX, firstY);
      EuclidGeometryTools.perpendicularVector2D(normal, normal);
      normal.normalize();
      normal.setX(Math.abs(normal.getX()));
      normal.setY(Math.abs(normal.getY()));

      return normal;
   }

   static List<NavigableRegion> filterNavigableRegionsConnectionWithDistanceAndHeightChange(NavigableRegion homeRegion,
                                                                                                    List<NavigableRegion> navigableRegions,
                                                                                                    double maxConnectionDistance, double maxHeightDelta)
   {
      return navigableRegions.stream().filter(
            otherRegion -> isOtherNavigableRegionWithinDistanceAndHeightDifference(homeRegion, otherRegion, maxConnectionDistance, maxHeightDelta))
                             .collect(Collectors.toList());
   }

   private static boolean isOtherNavigableRegionWithinDistanceAndHeightDifference(NavigableRegion regionA, NavigableRegion regionB,
                                                                                  double maxConnectionDistance, double maxHeightDelta)
   {
      return isOtherNavigableRegionWithinDistanceAndHeightDifference(regionA.getHomeRegionCluster(), regionB.getHomeRegionCluster(), maxConnectionDistance, maxHeightDelta);
   }

   private static boolean isOtherNavigableRegionWithinDistanceAndHeightDifference(Cluster regionA, Cluster regionB,
                                                                                  double maxConnectionDistance, double maxHeightDelta)
   {
      for (Point3DReadOnly pointA : regionA.getNavigableExtrusionsInWorld())
      {
         for (Point3DReadOnly pointB : regionB.getNavigableExtrusionsInWorld())
         {
            if (pointA.distance(pointB) < maxConnectionDistance && Math.abs(pointA.getZ() - pointB.getZ()) < maxHeightDelta)
               return true;
         }
      }

      return false;
   }

   static List<NavigableRegion> filterNavigableRegionsWithBoundingCircle(Point2DReadOnly circleOrigin, double circleRadius,
                                                                                 List<NavigableRegion> navigableRegions)
   {
      if (!Double.isFinite(circleRadius) || circleRadius < 0.0)
         return navigableRegions;

      return navigableRegions.stream().filter(
            navigableRegion -> PlanarRegionTools.isPlanarRegionIntersectingWithCircle(circleOrigin, circleRadius, navigableRegion.getHomePlanarRegion()))
                             .collect(Collectors.toList());
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
