package us.ihmc.pathPlanning.visibilityGraphs.postProcessing;

import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityGraphNode;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapSolution;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.*;
import java.util.stream.Collectors;

/**
 * Newer than ObstacleAndCliffAvoidanceProcessor.
 * Post processing algorithm for modifying a path plan to better avoid obstacles:
 * - Wiggles points to desired clearance of obstacles
 * - Can add intermediary waypoints to assist in avoiding obstacles
 */
public class ObstacleAvoidanceProcessor implements BodyPathPostProcessor
{
   private static final double minDistanceToMove = 0.01;
   private static final double waypointResolution = 0.1;

   private double desiredDistanceFromObstacleCluster;

   private final VisibilityGraphsParametersReadOnly parameters;

   public ObstacleAvoidanceProcessor(VisibilityGraphsParametersReadOnly parameters)
   {
      this.parameters = parameters;
      desiredDistanceFromObstacleCluster = parameters.getPreferredObstacleExtrusionDistance() - parameters.getObstacleExtrusionDistance();
   }

   public List<Point3DReadOnly> computePathFromNodes(List<VisibilityGraphNode> nodePath, VisibilityMapSolution visibilityMapSolution)
   {
      desiredDistanceFromObstacleCluster = parameters.getPreferredObstacleExtrusionDistance() - parameters.getObstacleExtrusionDistance();

      List<Point3D> newPathPositions = nodePath.stream().map(node -> new Point3D(node.getPointInWorld())).collect(Collectors.toList());

      if (!parameters.getPerformPostProcessingNodeShifting())
         return new ArrayList<>(newPathPositions);

      int pathNodeIndex = 0;
      int waypointIndex = 0;
      // don't do the goal node
      while (pathNodeIndex < nodePath.size() - 1)
      {
         int nextPathNodeIndex = pathNodeIndex + 1;
         int nextWaypointIndex = waypointIndex + 1;

         Point3D startPointInWorld = newPathPositions.get(waypointIndex);
         Point3D endPointInWorld = newPathPositions.get(nextWaypointIndex);

         VisibilityGraphNode startVisGraphNode = nodePath.get(pathNodeIndex);
         VisibilityGraphNode endVisGraphNode = nodePath.get(nextPathNodeIndex);

         boolean isGoalNode = nextWaypointIndex > newPathPositions.size() - 2;

         NavigableRegion startingRegion = startVisGraphNode.getVisibilityGraphNavigableRegion().getNavigableRegion();
         NavigableRegion endingRegion = endVisGraphNode.getVisibilityGraphNavigableRegion().getNavigableRegion();

         if (!isGoalNode)
         {
            adjustNodePositionToAvoidObstaclesAndCliffs(endPointInWorld, startingRegion, endingRegion);
         }

         if (parameters.getIntroduceMidpointsInPostProcessing())
         {
            List<Point3D> intermediateWaypointsToAdd = addAndAdjustMidpoints(startPointInWorld, endPointInWorld, startVisGraphNode, endVisGraphNode,
                                                                             startingRegion, endingRegion);

            // add the adjusted points back in
            for (Point3D intermediateWaypointToAdd : intermediateWaypointsToAdd)
            {
               waypointIndex++;
               newPathPositions.add(waypointIndex, intermediateWaypointToAdd);
            }
         }

         waypointIndex++;
         pathNodeIndex++;
      }

      return new ArrayList<>(newPathPositions);
   }

   private List<Point3D> addAndAdjustMidpoints(Point3DReadOnly startPoint, Point3DReadOnly endPoint, VisibilityGraphNode startNode, VisibilityGraphNode endNode,
                                               NavigableRegion startRegion, NavigableRegion endRegion)
   {
      List<Point3D> intermediateWaypointsToAdd = PostProcessingTools.computeIntermediateWaypointsToAddToAvoidObstacles(new Point2D(startPoint),
                                                                                                                       new Point2D(endPoint),
                                                                                                                       startNode,
                                                                                                                       endNode,
                                                                                                                       desiredDistanceFromObstacleCluster,
                                                                                                                       waypointResolution);
      // precautionary checks
      PostProcessingTools.removeDuplicated3DPointsFromList(intermediateWaypointsToAdd, waypointResolution);
      PostProcessingTools.removeDuplicateStartOrEndPointsFromList(intermediateWaypointsToAdd, startPoint, endPoint, waypointResolution);

      // shift all the points around
      for (Point3D intermediateWaypointToAdd : intermediateWaypointsToAdd)
         adjustNodePositionToAvoidObstaclesAndCliffs(intermediateWaypointToAdd, startRegion, endRegion);

      // prune duplicated points
      PostProcessingTools.removeDuplicated3DPointsFromList(intermediateWaypointsToAdd, waypointResolution);
      PostProcessingTools.removeDuplicateStartOrEndPointsFromList(intermediateWaypointsToAdd, startPoint, endPoint, waypointResolution);

      return intermediateWaypointsToAdd;
   }

   private void adjustNodePositionToAvoidObstaclesAndCliffs(Point3DBasics nodeLocationToPack, NavigableRegion startRegion, NavigableRegion endRegion)
   {
      Point2D nextPointInWorld2D = new Point2D(nodeLocationToPack);

      Set<NavigableRegion> bothRegions = new HashSet<>();
      bothRegions.add(startRegion);
      bothRegions.add(endRegion);

      List<Cluster> obstacleClusters = new ArrayList<>();
      for (NavigableRegion navigableRegion : bothRegions)
      {
         for (Cluster potentialCluster : navigableRegion.getObstacleClusters())
         {
            // might be unecessary check, points in here were already filtered for this
            if (potentialCluster.getRawPointsInLocal3D().stream().anyMatch(point -> point.getZ() > parameters.getTooHighToStepDistance()))
               obstacleClusters.add(potentialCluster);
         }
      }

      List<Point2DReadOnly> closestObstacleClusterPoints = PostProcessingTools.getClosestPointOnEachCluster(nextPointInWorld2D, obstacleClusters);
      Vector2DReadOnly nodeShiftToAvoidObstacles = PointWiggler.computeBestShiftVectorToAvoidPoints(nextPointInWorld2D, closestObstacleClusterPoints,
                                                                                                    desiredDistanceFromObstacleCluster, 0.0);

      if (nodeShiftToAvoidObstacles.containsNaN())
         nodeShiftToAvoidObstacles = new Vector2D();

      if (nodeShiftToAvoidObstacles.length() < minDistanceToMove)
         return; // didn't shift significantly or it's NaN, don't need to do following checks

      Point2D shiftedPoint = new Point2D(nodeLocationToPack);
      shiftedPoint.add(nodeShiftToAvoidObstacles);

      List<Point2DReadOnly> closestCliffObstacleClusterPoints = new ArrayList<>(
            PostProcessingTools.getPointsAlongEdgeOfClusterClosestToPoint(nextPointInWorld2D, startRegion.getHomeRegionCluster()));

      if (endRegion != null && endRegion.getHomeRegionCluster() != null)
      {
         boolean pointIsInEndRegion = EuclidGeometryPolygonTools.isPoint2DInsideConvexPolygon2D(nextPointInWorld2D,
                                                                                                endRegion.getHomeRegionCluster().getRawPointsInWorld2D(),
                                                                                                endRegion.getHomeRegionCluster().getNumberOfRawPoints(),
                                                                                                   true, 0.0);
         if (pointIsInEndRegion)
            closestCliffObstacleClusterPoints.addAll(PostProcessingTools.getPointsAlongEdgeOfClusterClosestToPoint(nextPointInWorld2D, endRegion.getHomeRegionCluster()));
      }

      nodeShiftToAvoidObstacles = PointWiggler.computeBestShiftVectorToAvoidPoints(nextPointInWorld2D, closestObstacleClusterPoints,
                                                                                   closestCliffObstacleClusterPoints, desiredDistanceFromObstacleCluster,
                                                                                   parameters.getNavigableExtrusionDistance(), 0.0,
                                                                                   parameters.getNavigableExtrusionDistance());
      if (nodeShiftToAvoidObstacles.containsNaN())
         nodeShiftToAvoidObstacles = new Vector2D();

      if (nodeShiftToAvoidObstacles.length() > minDistanceToMove) // if it moved a significant amount or if it's NaN
      {
         nextPointInWorld2D.add(nodeShiftToAvoidObstacles);
         nodeLocationToPack.set(nextPointInWorld2D, PostProcessingTools.findHeightOfPoint(nextPointInWorld2D, bothRegions));
      }
   }
}
