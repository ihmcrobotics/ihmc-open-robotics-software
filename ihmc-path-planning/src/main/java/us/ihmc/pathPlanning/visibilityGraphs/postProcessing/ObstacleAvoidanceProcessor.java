package us.ihmc.pathPlanning.visibilityGraphs.postProcessing;

import org.apache.logging.log4j.util.PropertySource;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapSolution;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityTools;
import us.ihmc.robotics.time.TimeIntervalProvider;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;

public class ObstacleAvoidanceProcessor
{
   private static final double minDistanceToMove = 0.01;
   private final double realDistanceFromObstacle;
   private final IntermediateComparator comparator = new IntermediateComparator();

   public ObstacleAvoidanceProcessor(VisibilityGraphsParameters parameters)
   {
      realDistanceFromObstacle = parameters.getPreferredObstacleExtrusionDistance() - parameters.getObstacleExtrusionDistance();
   }

   public List<Point3DReadOnly> pushNodesAwayFromObstacles(List<Point3DReadOnly> oldNodePath, VisibilityMapSolution visibilityMapSolution)
   {
      List<Point3D> newPath = oldNodePath.parallelStream().map(Point3D::new).collect(Collectors.toList());

      for (NavigableRegion navigableRegion : visibilityMapSolution.getNavigableRegions().getNaviableRegionsList())
      {
         int nodeIndex = 0;
         while (nodeIndex < oldNodePath.size() - 1)
         {
            Point2D originPointInWorld2D = new Point2D(newPath.get(nodeIndex));
            Point2D nextPointInWorld2D = new Point2D(newPath.get(nodeIndex + 1));

            boolean isNextPointGoal = nodeIndex == oldNodePath.size() - 1;

            Vector2D nodeShift = new Vector2D();
            int numberOfShifts = 0;

            for (Cluster cluster : navigableRegion.getObstacleClusters())
            {
               // handle and move origin and goal points
               List<Point2DReadOnly> clusterPolygon = cluster.getNonNavigableExtrusionsInWorld2D();

               Point2D closestPointInCluster = new Point2D();

               if (!isNextPointGoal)
               {
                  double distanceToCluster = VisibilityTools.distanceToCluster(nextPointInWorld2D, clusterPolygon, closestPointInCluster, null);
                  if (distanceToCluster < realDistanceFromObstacle)
                  {
                     double distanceToMove = realDistanceFromObstacle - distanceToCluster;
                     Vector2D nodeOffset = new Vector2D();
                     nodeOffset.sub(nextPointInWorld2D, closestPointInCluster);
                     nodeOffset.normalize();
                     nodeOffset.scale(distanceToMove);

//                     nextPointInWorld2D.add(nodeOffset);
                     nodeShift.add(nodeOffset);
//                     double newHeight = navigableRegion.getPlaneZGivenXY(nextPointInWorld2D.getX(), nextPointInWorld2D.getY());

//                     newPath.get(nodeIndex + 1).set(nextPointInWorld2D, newHeight);
                     numberOfShifts += 1;
                  }
               }
            }

            if (nodeShift.length() > minDistanceToMove)
            {
               nodeShift.scale(1.0 / numberOfShifts);
               nextPointInWorld2D.add(nodeShift);
               double newHeight = navigableRegion.getPlaneZGivenXY(nextPointInWorld2D.getX(), nextPointInWorld2D.getY());
               newPath.get(nodeIndex + 1).set(nextPointInWorld2D, newHeight);
            }

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
               nodeIndex++;
         }
      }

      return newPath.parallelStream().map(Point3D::new).collect(Collectors.toList());
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
