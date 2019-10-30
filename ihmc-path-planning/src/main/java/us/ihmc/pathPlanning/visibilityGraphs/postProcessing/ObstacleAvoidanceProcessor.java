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

public class ObstacleAvoidanceProcessor implements PostProcessor
{
   private static final double minDistanceToMove = 0.01;
   private static final double samePointEpsilon = 0.01;

   private double desiredDistanceFromObstacleCluster;

   private final List<ObstacleAndCliffAvoidanceInfo> pointInfos = new ArrayList<>();

   private final VisibilityGraphsParametersReadOnly parameters;

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
      for (int pathNodeIndex = 0; pathNodeIndex < nodePath.size() - 1; pathNodeIndex++)
      {
         int nextPathNodeIndex = pathNodeIndex + 1;

         Point3D endPointInWorld = newPathPositions.get(nextPathNodeIndex);

         ObstacleAndCliffAvoidanceInfo endPointInfo = null;
         if (nextPathNodeIndex < pointInfos.size() )
            endPointInfo = pointInfos.get(nextPathNodeIndex);

         VisibilityGraphNode startVisGraphNode = nodePath.get(pathNodeIndex);
         VisibilityGraphNode endVisGraphNode = nodePath.get(nextPathNodeIndex);

         NavigableRegion startingRegion = startVisGraphNode.getVisibilityGraphNavigableRegion().getNavigableRegion();
         NavigableRegion endingRegion = endVisGraphNode.getVisibilityGraphNavigableRegion().getNavigableRegion();

         adjustNodePositionToAvoidObstaclesAndCliffs(endPointInfo, endPointInWorld, startingRegion, endingRegion);
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

      List<Point2DReadOnly> closestObstacleClusterPoints = getClosestPointOnEachCluster(nextPointInWorld2D, obstacleClusters);
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

      List<Point2DReadOnly> closestCliffObstacleClusterPoints = new ArrayList<>(getPointsAlongEdgeOfClusterClosestToPoint(nextPointInWorld2D, endRegion.getHomeRegionCluster()));

      if (startRegion != endRegion)
      {
         boolean pointIsInEndRegion = EuclidGeometryPolygonTools.isPoint2DInsideConvexPolygon2D(nextPointInWorld2D,
                                                                                                   startRegion.getHomeRegionCluster().getRawPointsInWorld2D(),
                                                                                                   startRegion.getHomeRegionCluster().getNumberOfRawPoints(),
                                                                                                   true, 0.0);
         if (pointIsInEndRegion)
            closestCliffObstacleClusterPoints.addAll(getPointsAlongEdgeOfClusterClosestToPoint(nextPointInWorld2D, startRegion.getHomeRegionCluster()));
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
         nodeLocationToPack.set(nextPointInWorld2D, findHeightOfPoint(nextPointInWorld2D, bothRegions));
      }
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
}
