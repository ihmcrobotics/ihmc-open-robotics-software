package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.robotics.geometry.PlanarRegion;

public class VisibilityTools
{
   private static final double MAGIC_NUMBER = MathTools.square(0.01);

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

   public static List<Connection> getConnectionsThatAreInsideRegion(Collection<Connection> connections, PlanarRegion region)
   {
      List<Connection> filteredConnections = new ArrayList<>();

      for (Connection connection : connections)
      {

         if (PlanarRegionTools.areBothPointsInsidePolygon(connection.getSourcePoint2D(), connection.getTargetPoint2D(), region))
         {
            filteredConnections.add(connection);
         }
      }

      return filteredConnections;
   }

   public static List<Connection> getConnectionsThatAreInsideRegion(Collection<Connection> connections, List<? extends Point2DReadOnly> polygon)
   {
      List<Connection> filteredConnections = new ArrayList<>();

      for (Connection connection : connections)
      {

         if (PlanarRegionTools.areBothPointsInsidePolygon(connection.getSourcePoint2D(), connection.getTargetPoint2D(), polygon))
         {
            filteredConnections.add(connection);
         }
      }

      return filteredConnections;
   }

   public static Set<Connection> createStaticVisibilityMap(List<Cluster> clusters, NavigableRegion navigableRegion)
   {
      int regionId = navigableRegion.getMapId();
      Set<Connection> connections = new HashSet<>();

      List<Point2DReadOnly> listOfObserverPoints = new ArrayList<>();

      // Add all navigable points (including dynamic objects) to a list
      for (Cluster cluster : clusters)
      {
         for (Point2D point : cluster.getNavigableExtrusionsInLocal2D())
         {
            if (!PlanarRegionTools.isPointInLocalInsidePlanarRegion(navigableRegion.getHomeRegion(), point))
               continue;

            boolean isNonNavigable = clusters.stream().filter(c -> c.isInsideNonNavigableZone(point)).findFirst().isPresent();
            if (isNonNavigable)
               continue;

            listOfObserverPoints.add(point);
         }
      }

      for (int i = 0; i < listOfObserverPoints.size(); i++)
      {
         Point2DReadOnly observer = listOfObserverPoints.get(i);

         for (int j = i + 1; j < listOfObserverPoints.size(); j++)
         {
            Point2DReadOnly target = listOfObserverPoints.get(j);

            if (observer.distanceSquared(target) > MAGIC_NUMBER)
            {
               boolean targetIsVisible = isPointVisibleForStaticMaps(clusters, observer, target);

               if (targetIsVisible)
               {
                  connections.add(new Connection(observer, regionId, target, regionId));
               }
            }
         }
      }

      return connections;
   }

   public static Set<Connection> createStaticVisibilityMap(Point3DReadOnly observer, int observerRegionId, List<Cluster> clusters, int clustersRegionId,
                                                           boolean ensureConnection)
   {
      Set<Connection> connections = new HashSet<>();
      List<Point2D> listOfTargetPoints = new ArrayList<>();
      Point2D observer2D = new Point2D(observer);

      // Add all navigable points (including dynamic objects) to a list
      for (Cluster cluster : clusters)
      {
         for (Point2D point : cluster.getNavigableExtrusionsInLocal2D())
         {
            listOfTargetPoints.add(point);
         }
      }

      for (int j = 0; j < listOfTargetPoints.size(); j++)
      {
         Point2D target = listOfTargetPoints.get(j);

         if (observer.distanceXYSquared(target) > MAGIC_NUMBER)
         {
            boolean targetIsVisible = isPointVisibleForStaticMaps(clusters, observer2D, target);

            if (targetIsVisible)
            {
               connections.add(new Connection(observer, observerRegionId, new Point3D(target), clustersRegionId));
            }
         }
      }

      if (ensureConnection && connections.isEmpty())
      {
         Point2D closestTarget = null;
         double minDistance = Double.POSITIVE_INFINITY;

         for (Point2D target : listOfTargetPoints)
         {
            double targetDistance = target.distanceXYSquared(observer);
            if (targetDistance < minDistance)
            {
               closestTarget = target;
               minDistance = targetDistance;
            }
         }
         connections.add(new Connection(observer, observerRegionId, new Point3D(closestTarget), clustersRegionId));
      }

      return connections;
   }

   public static boolean isPointVisibleForStaticMaps(List<Cluster> clusters, Point2DReadOnly observer, Point2DReadOnly targetPoint)
   {
      for (Cluster cluster : clusters)
      {
         if (!VisibilityTools.isPointVisible(observer, targetPoint, cluster.getNonNavigableExtrusionsInLocal2D()))
         {
            return false;
         }
      }

      return true;
   }

   public static List<Connection> removeConnectionsFromExtrusionsOutsideRegions(Collection<Connection> connections, PlanarRegion homeRegion)
   {
      return VisibilityTools.getConnectionsThatAreInsideRegion(connections, homeRegion);
   }

   public static List<Connection> removeConnectionsFromExtrusionsInsideNoGoZones(Collection<Connection> connectionsToClean, List<Cluster> clusters)
   {
      List<Connection> validConnections = new ArrayList<>(connectionsToClean);

      for (Cluster cluster : clusters)
      {
         validConnections = getValidConnections(validConnections, cluster);
      }

      return validConnections;
   }

   public static List<Connection> getValidConnections(Collection<Connection> connections, Cluster cluster)
   {
      List<Connection> filteredConnections = new ArrayList<>();

      for (Connection connection : connections)
      {
         Point2D source = connection.getSourcePoint2D();
         Point2D target = connection.getTargetPoint2D();

         if (!cluster.isInsideNonNavigableZone(source) && !cluster.isInsideNonNavigableZone(target))
            filteredConnections.add(connection);
      }

      return filteredConnections;
   }
}
