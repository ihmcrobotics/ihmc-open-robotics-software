package us.ihmc.pathPlanning.visibilityGraphs.postProcessing;

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

import java.util.List;
import java.util.stream.Collectors;

public class ObstacleAvoidanceProcessor
{
   private final double realDistanceFromObstacle;

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

            for (Cluster cluster : navigableRegion.getObstacleClusters())
            {
               // handle and move origin and goal points
               List<Point2DReadOnly> clusterPolygon = cluster.getNonNavigableExtrusionsInWorld2D();
               boolean isClosed = cluster.isClosed();

               Point2D closestPointInCluster = new Point2D();
               Vector2D clusterNormal = new Vector2D();

               if (!isNextPointGoal)
               {
                  double distanceToCluster = VisibilityTools.distanceToCluster(nextPointInWorld2D, clusterPolygon, closestPointInCluster, clusterNormal);
                  if (distanceToCluster < realDistanceFromObstacle)
                  {
                     double distanceToMove = realDistanceFromObstacle - distanceToCluster;
                     Vector2D nodeOffset = new Vector2D();
                     nodeOffset.sub(nextPointInWorld2D, closestPointInCluster);
                     nodeOffset.normalize();
                     nodeOffset.scale(distanceToMove);

                     nextPointInWorld2D.add(nodeOffset);
                     double newHeight = navigableRegion.getPlaneZGivenXY(nextPointInWorld2D.getX(), nextPointInWorld2D.getY());

                     newPath.get(nodeIndex + 1).set(nextPointInWorld2D, newHeight);
                  }
               }


               Point2D closestPointOnConnection = new Point2D();

               double connectionDistanceToObstacle = VisibilityTools.distanceToCluster(originPointInWorld2D, nextPointInWorld2D, clusterPolygon,
                                                                                       closestPointOnConnection, closestPointInCluster, clusterNormal, isClosed);
               if (connectionDistanceToObstacle < realDistanceFromObstacle)
               {
                  double distanceToMove = realDistanceFromObstacle - connectionDistanceToObstacle;
                  Point2D newPathNode = new Point2D(closestPointOnConnection);

                  Vector2D nodeOffset = new Vector2D();
                  nodeOffset.sub(closestPointOnConnection, closestPointInCluster);
                  nodeOffset.normalize();
                  nodeOffset.scale(distanceToMove);

                  newPathNode.add(nodeOffset);
                  double newHeight = navigableRegion.getPlaneZGivenXY(nextPointInWorld2D.getX(), nextPointInWorld2D.getY());
                  newPath.add(nodeIndex + 1, new Point3D(newPathNode.getX(), newPathNode.getY(), newHeight));
               }
            }

            nodeIndex++;
         }
      }

      return newPath.parallelStream().map(Point3D::new).collect(Collectors.toList());
   }
}
