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
   private final double realDistanceFromObstacle;
   private final IntermediateComparator comparator = new IntermediateComparator();

   public ObstacleAvoidanceProcessor(VisibilityGraphsParameters parameters)
   {
      realDistanceFromObstacle = parameters.getPreferredObstacleExtrusionDistance() - parameters.getObstacleExtrusionDistance();
   }

   public List<Point3DReadOnly> computePathFromNodes(List<VisibilityGraphNode> nodePath, VisibilityMapSolution visibilityMapSolution)
   {
      List<Point3D> newPath = nodePath.parallelStream().map(node -> new Point3D(node.getPointInWorld())).collect(Collectors.toList());

      int pathNodeIndex = 0;
      // don't do the goal node
      while (pathNodeIndex < nodePath.size() - 2)
      {
         int nextPathNodeIndex = pathNodeIndex + 1;
         Point2D originPointInWorld2D = new Point2D(newPath.get(pathNodeIndex));
         Point2D nextPointInWorld2D = new Point2D(newPath.get(nextPathNodeIndex));

         boolean isGoalNode = pathNodeIndex > nodePath.size() - 2;

         if (!isGoalNode)
         {
            adjustGoalNodePositionToAvoidObstacles(newPath.get(nextPathNodeIndex), nodePath.get(nextPathNodeIndex), visibilityMapSolution.getNavigableRegions());
            // run it again, in case moving it moved it to be within the range of another one.
            adjustGoalNodePositionToAvoidObstacles(newPath.get(nextPathNodeIndex), nodePath.get(nextPathNodeIndex), visibilityMapSolution.getNavigableRegions());
         }

         // add points
         for (NavigableRegion navigableRegion : visibilityMapSolution.getNavigableRegions().getNaviableRegionsList())
         {
            boolean nodeWasAdded = false;

            List<Point2D> intermediateWaypointsToAdd = new ArrayList<>();
            HashMap<Point2D, List<Vector2D>> waypointShifts = new HashMap<>();

            /*
            for (Cluster cluster : navigableRegion.getObstacleClusters())
            {
               List<Point2DReadOnly> clusterPolygon = cluster.getNonNavigableExtrusionsInWorld2D();
               boolean isClosed = cluster.isClosed();

               Point2D closestPointInCluster = new Point2D();
               Point2D closestPointOnConnection = new Point2D();

               Vector2D clusterNormal = new Vector2D();

               double connectionDistanceToObstacle = VisibilityTools
                     .distanceToCluster(originPointInWorld2D, nextPointInWorld2D, clusterPolygon, closestPointOnConnection, closestPointInCluster,
                                        clusterNormal, isClosed);
               if (connectionDistanceToObstacle < realDistanceFromObstacle)
               {
                  double distanceToMove = realDistanceFromObstacle - connectionDistanceToObstacle;

                  Vector2D nodeOffset = new Vector2D();
                  nodeOffset.sub(closestPointOnConnection, closestPointInCluster);
                  nodeOffset.normalize();
                  nodeOffset.scale(distanceToMove);

                  double newHeight = navigableRegion.getPlaneZGivenXY(nextPointInWorld2D.getX(), nextPointInWorld2D.getY());
                  Point3D newNode3D = new Point3D(closestPointOnConnection.getX() + nodeOffset.getX(), closestPointOnConnection.getY() + nodeOffset.getY(),
                                                  newHeight);
                  newPath.add(nodeIndex + 1, newNode3D);
                  nodeWasAdded = true;
               }
            }
            */

            comparator.setStartPoint(originPointInWorld2D);
            comparator.setEndPoint(nextPointInWorld2D);
            intermediateWaypointsToAdd.sort(comparator);

            if (!nodeWasAdded)
               pathNodeIndex++;
         }
      }

      return newPath.parallelStream().map(Point3D::new).collect(Collectors.toList());
   }

   private void adjustGoalNodePositionToAvoidObstacles(Point3D nodeLocationToPack, VisibilityGraphNode node, NavigableRegions navigableRegions)
   {
      Point2D nextPointInWorld2D = new Point2D(node.getPointInWorld());

      NavigableRegion navigableRegion = node.getVisibilityGraphNavigableRegion().getNavigableRegion();

      Vector2DReadOnly nodeShiftToAvoidObstacles = getDirectionAndDistanceToShiftNodeToAvoidObstacles(node);

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

   private Vector2DReadOnly getDirectionAndDistanceToShiftNodeToAvoidObstacles(VisibilityGraphNode node)
   {
      Point2D nextPointInWorld2D = new Point2D(node.getPointInWorld());
      Vector2D nodeShift = new Vector2D();
      int numberOfShifts = 0;

      NavigableRegion navigableRegion = node.getVisibilityGraphNavigableRegion().getNavigableRegion();

      for (Cluster cluster : navigableRegion.getObstacleClusters())
      {
         List<Point2DReadOnly> clusterPolygon = cluster.getNonNavigableExtrusionsInWorld2D();

         Point2D closestPointInCluster = new Point2D();
         double distanceToCluster = VisibilityTools.distanceToCluster(nextPointInWorld2D, clusterPolygon, closestPointInCluster, null);
         if (distanceToCluster < 1.1 * realDistanceFromObstacle)
         {
            double distanceToMove = realDistanceFromObstacle - distanceToCluster;
            Vector2D nodeOffset = new Vector2D();
            nodeOffset.sub(nextPointInWorld2D, closestPointInCluster);
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
      double maxHeight = Double.MIN_VALUE;
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
