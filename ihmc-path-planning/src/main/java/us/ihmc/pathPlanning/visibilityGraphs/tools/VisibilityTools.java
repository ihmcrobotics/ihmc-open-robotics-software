package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pathPlanning.visibilityGraphs.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.robotics.geometry.PlanarRegion;

public class VisibilityTools
{
   public static boolean isPointVisible(Point2DReadOnly observer, Point2DReadOnly targetPoint, List<? extends Point2DReadOnly> listOfPointsInCluster)
   {
      for (int i = 0; i < listOfPointsInCluster.size() - 1; i++)
      {
         Point2DReadOnly first = listOfPointsInCluster.get(i);
         Point2DReadOnly second = listOfPointsInCluster.get(i + 1);

         if (EuclidGeometryTools.doLineSegment2DsIntersect(first, second, observer, targetPoint))
         {
            return false;
         }
      }
      return true;
   }

   public static List<Connection> getConnectionsThatAreInsideRegion(List<Connection> connections, PlanarRegion region)
   {
      ArrayList<Connection> filteredConnections = new ArrayList<>();

      for (Connection connection : connections)
      {

         if (PlanarRegionTools.areBothPointsInsidePolygon(new Point2D(connection.getSourcePoint()), new Point2D(connection.getTargetPoint()), region))
         {
            filteredConnections.add(connection);
         }
      }

      return filteredConnections;
   }

   public static List<Connection> getConnectionsThatAreInsideRegion(List<Connection> connections, List<? extends Point2DReadOnly> polygon)
   {
      List<Connection> filteredConnections = new ArrayList<>();

      for (Connection connection : connections)
      {

         if (PlanarRegionTools.areBothPointsInsidePolygon(new Point2D(connection.getSourcePoint()), new Point2D(connection.getTargetPoint()), polygon))
         {
            filteredConnections.add(connection);
         }
      }

      return filteredConnections;
   }

   public static Set<Connection> createStaticVisibilityMap(Point2DReadOnly start, Point2DReadOnly goal, List<Cluster> clusters)
   {
      Set<Connection> connections = new HashSet<>();

      List<Point2DReadOnly> listOfObserverPoints = new ArrayList<>();

      if (start != null)
      {
         listOfObserverPoints.add(start);
      }

      if (goal != null)
      {
         listOfObserverPoints.add(goal);
      }

      // Add all navigable points (including dynamic objects) to a list
      for (Cluster cluster : clusters)
      {
         if (!cluster.isDynamic())
         {
            for (Point2D point : cluster.getNavigableExtrusionsInLocal())
            {
               listOfObserverPoints.add(point);
            }
         }
      }

      for (int i = 0; i < listOfObserverPoints.size(); i++)
      {
         Point2DReadOnly observer = listOfObserverPoints.get(i);

         for (int j = i + 1; j < listOfObserverPoints.size(); j++)
         {
            Point2DReadOnly target = listOfObserverPoints.get(j);

            if (observer.distance(target) > 0.01)
            {
               boolean targetIsVisible = isPointVisibleForStaticMaps(clusters, observer, target);

               if (targetIsVisible)
               {
                  connections.add(new Connection(new Point3D(observer), new Point3D(target)));
               }
            }
         }
      }

      return connections;
   }

   public static Set<Connection> createStaticVisibilityMap(Point2DReadOnly observer, List<Cluster> clusters)
   {
      Set<Connection> connections = new HashSet<>();
      List<Point2D> listOfTargetPoints = new ArrayList<>();

      // Add all navigable points (including dynamic objects) to a list
      for (Cluster cluster : clusters)
      {
         if (!cluster.isDynamic())
         {
            for (Point2D point : cluster.getNavigableExtrusionsInLocal())
            {
               listOfTargetPoints.add(point);
            }
         }
      }

      for (int j = 0; j < listOfTargetPoints.size(); j++)
      {
         Point2D target = listOfTargetPoints.get(j);

         if (observer.distance(target) > 0.01)
         {
            boolean targetIsVisible = isPointVisibleForStaticMaps(clusters, observer, target);

            if (targetIsVisible)
            {
               connections.add(new Connection(new Point3D(observer), new Point3D(target)));
            }
         }
      }

      return connections;
   }

   public static boolean isPointVisibleForStaticMaps(List<Cluster> clusters, Point2DReadOnly observer, Point2DReadOnly targetPoint)
   {
      for (Cluster cluster : clusters)
      {
         if (!VisibilityTools.isPointVisible(observer, targetPoint, cluster.getNonNavigableExtrusionsInLocal()))
         {
            return false;
         }
      }

      return true;
   }

   public static List<Connection> removeConnectionsFromExtrusionsOutsideRegions(List<Connection> connections, PlanarRegion homeRegion)
   {
      return VisibilityTools.getConnectionsThatAreInsideRegion(connections, homeRegion);
   }

   public static List<Connection> removeConnectionsFromExtrusionsInsideNoGoZones(List<Connection> connectionsToClean, List<Cluster> clusters)
   {
      List<Connection> masterListOfConnections = new ArrayList<>();
      List<Cluster> filteredClusters = new ArrayList<>();

      if (clusters.size() > 1)
      {
         for (int i = 0; i < clusters.size() - 1; i++)
         {
            filteredClusters.add(clusters.get(i));
         }

         List<Connection> connectionsToRemove = new ArrayList<>();
         for (Cluster cluster : filteredClusters)
         {

            if (cluster.getNonNavigableExtrusionsInLocal().size() == 0)
            {
               continue;
            }

            List<Connection> filteredConnections = VisibilityTools.getConnectionsThatAreInsideRegion(connectionsToClean,
                                                                                                     cluster.getNonNavigableExtrusionsInLocal());
            for (Connection connection : filteredConnections)
            {
               connectionsToRemove.add(connection);
            }
         }

         List<Connection> connectionsInsideHomeRegion = VisibilityTools.getConnectionsThatAreInsideRegion(connectionsToClean,
                                                                                                          clusters.get(clusters.size() - 1)
                                                                                                                  .getNonNavigableExtrusionsInLocal());

         List<Connection> finalList = new ArrayList<>(connectionsInsideHomeRegion);
         for (Connection connection : connectionsInsideHomeRegion)
         {
            for (Connection connectionToRemove : connectionsToRemove)
            {
               if (connection.epsilonEquals(connectionToRemove, 1E-5))
               {
                  finalList.remove(connection);
               }
            }
         }

         for (Connection connection : finalList)
         {
            masterListOfConnections.add(connection);
         }
      }
      else
      {
         filteredClusters.addAll(clusters);

         for (Cluster cluster : filteredClusters)
         {
            if (cluster.getNonNavigableExtrusionsInLocal().size() == 0)
            {
               continue;
            }

            List<Connection> filteredConnections = VisibilityTools.getConnectionsThatAreInsideRegion(connectionsToClean,
                                                                                                     cluster.getNonNavigableExtrusionsInLocal());
            for (Connection connection : filteredConnections)
            {
               masterListOfConnections.add(connection);
            }
         }
      }

      return masterListOfConnections;
   }

}
