package us.ihmc.pathPlanning.visibilityGraphs.postProcessing;

import us.ihmc.euclid.geometry.LineSegment2D;
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

public class ObstacleAndCliffAvoidanceProcessor implements BodyPathPostProcessor
{
   private static final double minDistanceToMove = 0.01;
   private static final double cliffHeightToAvoid = 0.10;
   private static final double waypointResolution = 0.1;

   private double desiredDistanceFromObstacleCluster;
   private double minimumDistanceFromObstacleCluster;
   private double desiredDistanceFromCliff;
   private double minimumDistanceFromCliff; // FIXME this is currently unused
   private double maxInterRegionConnectionLength;
   private final WaypointComparator comparator = new WaypointComparator();

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

         if (parameters.getIntroduceMidpointsInPostProcessing())
         {
            List<Point3D> intermediateWaypointsToAdd = PostProcessingTools.computeIntermediateWaypointsToAddToAvoidObstacles(new Point2D(startPointInWorld),
                                                                                                                             new Point2D(endPointInWorld),
                                                                                                                             startVisGraphNode,
                                                                                                                             endVisGraphNode,
                                                                                                                             desiredDistanceFromObstacleCluster,
                                                                                                                             waypointResolution);
            PostProcessingTools.removeDuplicated3DPointsFromList(intermediateWaypointsToAdd, waypointResolution);
            PostProcessingTools.removeDuplicateStartOrEndPointsFromList(intermediateWaypointsToAdd, startPointInWorld, endPointInWorld, waypointResolution);

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
            PostProcessingTools.removeDuplicated3DPointsFromList(intermediateWaypointsToAdd, waypointResolution);
            PostProcessingTools.removeDuplicateStartOrEndPointsFromList(intermediateWaypointsToAdd, startPointInWorld, endPointInWorld, waypointResolution);

         }

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

      List<Point2DReadOnly> closestObstacleClusterPoints = PostProcessingTools.getClosestPointOnEachCluster(nextPointInWorld2D, obstacleClusters);
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


         closestCliffObstacleClusterPoints.addAll(PostProcessingTools.getPointsAlongEdgeOfClusterClosestToPoint(nextPointInWorld2D, endRegion.getHomeRegionCluster()));
         if (startRegion != endRegion && EuclidGeometryPolygonTools.isPoint2DInsideConvexPolygon2D(nextPointInWorld2D, startRegion.getHomeRegionCluster().getRawPointsInWorld2D(),
                                                                                                   startRegion.getHomeRegionCluster().getNumberOfRawPoints(), true, 0.0))
         {
            closestCliffObstacleClusterPoints.addAll(PostProcessingTools.getPointsAlongEdgeOfClusterClosestToPoint(nextPointInWorld2D, startRegion.getHomeRegionCluster()));
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
         nodeLocationToPack.set(nextPointInWorld2D, PostProcessingTools.findHeightOfPoint(nextPointInWorld2D, bothRegions));
      }

      double distanceToClosestObstacle = Double.POSITIVE_INFINITY;
      for (Point2DReadOnly obstaclePoint : closestObstacleClusterPoints)
         distanceToClosestObstacle = Math.min(distanceToClosestObstacle, nodeLocationToPack.distanceXY(obstaclePoint));

      return distanceToClosestObstacle + parameters.getObstacleExtrusionDistance();
   }

   private static List<LineSegment2DReadOnly> getClusterEdges(Cluster cluster)
   {
      return PostProcessingTools.getClusterEdges(cluster.getRawPointsInWorld2D());
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
            .distanceToCluster(shiftedPoint, homeCluster.getNavigableExtrusionsInWorld2D().getPoints(), closestPointToThrowAway, null);

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
}
