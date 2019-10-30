package us.ihmc.pathPlanning.visibilityGraphs.postProcessing;

import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityGraphNode;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapSolution;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;

import java.util.*;
import java.util.stream.Collectors;

public class ObstacleAvoidanceProcessor implements BodyPathPostProcessor
{
   private static final double minDistanceToMove = 0.01;
   private static final double waypointResolution = 0.1;


   private double desiredDistanceFromObstacleCluster;

   private final List<ObstacleAndCliffAvoidanceInfo> pointInfos = new ArrayList<>();

   private final VisibilityGraphsParametersReadOnly parameters;
   private final WaypointComparator comparator = new WaypointComparator();


   public ObstacleAvoidanceProcessor(VisibilityGraphsParametersReadOnly parameters)
   {
      this.parameters = parameters;
      desiredDistanceFromObstacleCluster = parameters.getPreferredObstacleExtrusionDistance() - parameters.getObstacleExtrusionDistance();
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

      if (!parameters.getPerformPostProcessingNodeShifting())
         return newPathPositions.parallelStream().map(Point3D::new).collect(Collectors.toList());

      // don't do the goal node
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

         if (!isGoalNode)
         {
            adjustNodePositionToAvoidObstaclesAndCliffs(endPointInfo, endPointInWorld, startingRegion, endingRegion);
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

            // shift all the points around
            for (Point3D intermediateWaypointToAdd : intermediateWaypointsToAdd)
            {
               waypointIndex++;
               newPathPositions.add(waypointIndex, intermediateWaypointToAdd);

               ObstacleAndCliffAvoidanceInfo pointInfo = new ObstacleAndCliffAvoidanceInfo();
               pointInfo.setWasIntroduced(true);
               pointInfos.add(waypointIndex, pointInfo);

               adjustNodePositionToAvoidObstaclesAndCliffs(null, intermediateWaypointToAdd, startingRegion, endingRegion);
            }

            // prune duplicated points
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
   }

   private void adjustNodePositionToAvoidObstaclesAndCliffs(ObstacleAndCliffAvoidanceInfo pointInfo, Point3D nodeLocationToPack, NavigableRegion startRegion,
                                                            NavigableRegion endRegion)
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
                                                                                                    desiredDistanceFromObstacleCluster, 0.0);

      if (pointInfo != null)
      {
         pointInfo.setClosestObstacleClusterPoints(closestObstacleClusterPoints);
         pointInfo.setShiftToAvoidObstacles(nodeShiftToAvoidObstacles);
      }

      if (nodeShiftToAvoidObstacles.containsNaN())
         nodeShiftToAvoidObstacles = new Vector2D();

      if (nodeShiftToAvoidObstacles.length() < minDistanceToMove)
         return;

      Point2D shiftedPoint = new Point2D(nodeLocationToPack);
      shiftedPoint.add(nodeShiftToAvoidObstacles);

      List<Point2DReadOnly> closestCliffObstacleClusterPoints = new ArrayList<>(
            PostProcessingTools.getPointsAlongEdgeOfClusterClosestToPoint(nextPointInWorld2D, endRegion.getHomeRegionCluster()));

      if (startRegion != endRegion)
      {
         boolean pointIsInEndRegion = EuclidGeometryPolygonTools.isPoint2DInsideConvexPolygon2D(nextPointInWorld2D,
                                                                                                   startRegion.getHomeRegionCluster().getRawPointsInWorld2D(),
                                                                                                   startRegion.getHomeRegionCluster().getNumberOfRawPoints(),
                                                                                                   true, 0.0);
         if (pointIsInEndRegion)
            closestCliffObstacleClusterPoints.addAll(PostProcessingTools.getPointsAlongEdgeOfClusterClosestToPoint(nextPointInWorld2D, startRegion.getHomeRegionCluster()));
      }

      nodeShiftToAvoidObstacles = PointWiggler.computeBestShiftVectorToAvoidPoints(nextPointInWorld2D, closestObstacleClusterPoints,
                                                                                   closestCliffObstacleClusterPoints, desiredDistanceFromObstacleCluster,
                                                                                   parameters.getNavigableExtrusionDistance(), 0.0,
                                                                                   parameters.getNavigableExtrusionDistance());
      if (nodeShiftToAvoidObstacles.containsNaN())
         nodeShiftToAvoidObstacles = new Vector2D();

      if (pointInfo != null)
      {
         pointInfo.setClosestCliffClusterPoints(closestCliffObstacleClusterPoints);
         pointInfo.setShiftToAvoidObstaclesAndCliffs(nodeShiftToAvoidObstacles);
      }

      List<NavigableRegion> bothRegions = new ArrayList<>();
      bothRegions.add(startRegion);
      bothRegions.add(endRegion);

      if (nodeShiftToAvoidObstacles.length() > minDistanceToMove)
      {
         nextPointInWorld2D.add(nodeShiftToAvoidObstacles);
         nodeLocationToPack.set(nextPointInWorld2D, PostProcessingTools.findHeightOfPoint(nextPointInWorld2D, bothRegions));
      }
   }
}
