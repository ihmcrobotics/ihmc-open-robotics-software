package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3D;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.InterRegionVisibilityMap;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.SingleSourceVisibilityMap;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMap;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.InterRegionConnectionFilter;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.NavigableExtrusionDistanceCalculator;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.NavigableRegionFilter;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.ObstacleExtrusionDistanceCalculator;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.ObstacleRegionFilter;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.PlanarRegionFilter;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.tools.ClusterTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PointCloudTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityTools;
import us.ihmc.robotics.geometry.PlanarRegion;

public class VisibilityGraphsFactory
{
   private static final double DEPTH_THRESHOLD_FOR_CONVEX_DECOMPOSITION = 0.05;
   /**
    * I believe these filters are now not useful anymore, but I haven't had the time to make sure
    * they're obsolete. When disabled, everything still looks good.
    * 
    * +++JEP: Pretty sure they are obsolete now. Turning them off.
    */
   private static final boolean ENABLE_GREEDY_FILTERS = false;
   
   // Whether to create the inter regions using the cluster points or , if false, to create them
   // after the inner regions are created, using the inner region maps,
   // The former (true) I believe is preferable since I think we should be creating inter regions first, then inner regions.
   // Also, could then do the two in parallel instead of sequentially.
   private static final boolean CREATE_INTER_REGIONS_USING_CLUSTER_POINTS = true;

   public static List<NavigableRegion> createNavigableRegions(List<PlanarRegion> allRegions, VisibilityGraphsParameters parameters)
   {
      if (allRegions.isEmpty())
         return null;

      List<NavigableRegion> navigableRegions = createNavigableRegionButNotVisibilityMaps(allRegions, parameters);
      createStaticVisibilityMapsForNavigableRegions(navigableRegions);

      return navigableRegions;
   }

   public static List<NavigableRegion> createNavigableRegionButNotVisibilityMaps(List<PlanarRegion> allRegions, VisibilityGraphsParameters parameters)
   {
      if (allRegions.isEmpty())
         return null;

      List<NavigableRegion> navigableRegions = new ArrayList<>(allRegions.size());

      NavigableRegionFilter navigableRegionFilter = parameters.getNavigableRegionFilter();

      for (int candidateIndex = 0; candidateIndex < allRegions.size(); candidateIndex++)
      {
         PlanarRegion candidate = allRegions.get(candidateIndex);

         List<PlanarRegion> otherRegions = new ArrayList<>(allRegions);
         Collections.swap(otherRegions, candidateIndex, otherRegions.size() - 1);
         otherRegions.remove(otherRegions.size() - 1);

         if (!navigableRegionFilter.isPlanarRegionNavigable(candidate, otherRegions))
            continue;

         NavigableRegion navigableRegion = createNavigableRegionButNotVisibilityMaps(candidate, otherRegions, parameters);
         navigableRegions.add(navigableRegion);
      }

      return navigableRegions;
   }

   public static NavigableRegion createNavigableRegionButNotVisibilityMaps(PlanarRegion region, List<PlanarRegion> otherRegions,
                                                                           VisibilityGraphsParameters parameters)
   {
      PlanarRegionFilter planarRegionFilter = parameters.getPlanarRegionFilter();
      double orthogonalAngle = parameters.getRegionOrthogonalAngle();
      double clusterResolution = parameters.getClusterResolution();
      NavigableExtrusionDistanceCalculator navigableCalculator = parameters.getNavigableExtrusionDistanceCalculator();
      ObstacleExtrusionDistanceCalculator obstacleCalculator = parameters.getObstacleExtrusionDistanceCalculator();
      ObstacleRegionFilter obstacleRegionFilter = parameters.getObstacleRegionFilter();
      return createNavigableRegionButNotVisibilityMaps(region, otherRegions, orthogonalAngle, clusterResolution, obstacleRegionFilter, planarRegionFilter,
                                                       navigableCalculator, obstacleCalculator);
   }

   public static NavigableRegion createNavigableRegion(PlanarRegion region, List<PlanarRegion> otherRegions, double orthogonalAngle, double clusterResolution,
                                                       ObstacleRegionFilter obstacleRegionFilter, PlanarRegionFilter filter,
                                                       NavigableExtrusionDistanceCalculator navigableCalculator,
                                                       ObstacleExtrusionDistanceCalculator obstacleCalculator)
   {
      NavigableRegion navigableRegion = createNavigableRegionButNotVisibilityMaps(region, otherRegions, orthogonalAngle, clusterResolution,
                                                                                  obstacleRegionFilter, filter, navigableCalculator, obstacleCalculator);
      createStaticVisibilityMapsForNavigableRegion(navigableRegion);

      return navigableRegion;
   }

   private static NavigableRegion createNavigableRegionButNotVisibilityMaps(PlanarRegion region, List<PlanarRegion> otherRegions, double orthogonalAngle,
                                                                            double clusterResolution, ObstacleRegionFilter obstacleRegionFilter,
                                                                            PlanarRegionFilter filter, NavigableExtrusionDistanceCalculator navigableCalculator,
                                                                            ObstacleExtrusionDistanceCalculator obstacleCalculator)
   {
      NavigableRegion navigableRegion = new NavigableRegion(region);
      PlanarRegion homeRegion = navigableRegion.getHomeRegion();

      List<PlanarRegion> obstacleRegions = otherRegions.stream().filter(candidate -> obstacleRegionFilter.isRegionValidObstacle(candidate, homeRegion))
                                                       .collect(Collectors.toList());

      obstacleRegions = PlanarRegionTools.filterRegionsByTruncatingVerticesBeneathHomeRegion(obstacleRegions, homeRegion,
                                                                                             DEPTH_THRESHOLD_FOR_CONVEX_DECOMPOSITION, filter);

      navigableRegion.setHomeRegionCluster(ClusterTools.createHomeRegionCluster(homeRegion, navigableCalculator));
      navigableRegion.addObstacleClusters(ClusterTools.createObstacleClusters(homeRegion, obstacleRegions, orthogonalAngle, obstacleCalculator));

      for (Cluster cluster : navigableRegion.getAllClusters())
      {
         List<Point2DReadOnly> expandListOf2DPoints = PointCloudTools.addPointsAlongPolygon(cluster.getNavigableExtrusionsInLocal(), clusterResolution);
         cluster.setNavigableExtrusionsInLocal(expandListOf2DPoints);
      }
      return navigableRegion;
   }

   public static void createStaticVisibilityMapsForNavigableRegions(List<NavigableRegion> navigableRegions)
   {
      if (navigableRegions == null)
         return;

      for (int navigableRegionIndex = 0; navigableRegionIndex < navigableRegions.size(); navigableRegionIndex++)
      {
         NavigableRegion navigableRegion = navigableRegions.get(navigableRegionIndex);
         createStaticVisibilityMapsForNavigableRegion(navigableRegion);
      }
   }

   private static void createStaticVisibilityMapsForNavigableRegion(NavigableRegion navigableRegion)
   {
      Collection<Connection> connectionsForMap = VisibilityTools.createStaticVisibilityMap(navigableRegion.getAllClusters(), navigableRegion);

      if (ENABLE_GREEDY_FILTERS)
      {
         PlanarRegion homeRegion = navigableRegion.getHomeRegion();
         connectionsForMap = VisibilityTools.removeConnectionsFromExtrusionsOutsideRegions(connectionsForMap, homeRegion);
         connectionsForMap = VisibilityTools.removeConnectionsFromExtrusionsInsideNoGoZones(connectionsForMap, navigableRegion.getAllClusters());
      }

      VisibilityMap visibilityMap = new VisibilityMap();
      visibilityMap.setConnections(connectionsForMap);
      navigableRegion.setVisibilityMapInLocal(visibilityMap);
   }

   /**
    * Creates a visibility map using the given {@code source} and connect it to all the visibility
    * connection points of the host region's map.
    * <p>
    * The host region is defined as the region that contains the given {@code source}.
    * </p>
    * <p>
    * When the source is located inside non accessible zone on the host region, it then either
    * connected to the closest connection point of the host region's map or the closest connection
    * from the given {@code potentialFallbackMap}, whichever is the closest.
    * </p>
    * 
    * @param source the single source used to build the visibility map.
    * @param navigableRegions the list of navigable regions among which the host is to be found. Not
    *           modified.
    * @param searchHostEpsilon espilon used during the search. When positive, it is equivalent to
    *           growing all the regions before testing if the {@code source} is inside.
    * @param potentialFallbackMap in case the source is located in a non accessible zone, the
    *           fallback map might be used to connect the source. Additional connections may be
    *           added to the map.
    * @return the new map or {@code null} if a host region could not be found.
    */
   public static SingleSourceVisibilityMap createSingleSourceVisibilityMap(Point3DReadOnly source, List<NavigableRegion> navigableRegions,
                                                                           double searchHostEpsilon, VisibilityMap potentialFallbackMap)
   {
      NavigableRegion hostRegion = PlanarRegionTools.getNavigableRegionContainingThisPoint(source, navigableRegions, searchHostEpsilon);

      if (hostRegion == null)
         return null;

      Point3D sourceInLocal = new Point3D(source);
      hostRegion.transformFromWorldToLocal(sourceInLocal);
      int mapId = hostRegion.getMapId();

      Set<Connection> connections = VisibilityTools.createStaticVisibilityMap(sourceInLocal, mapId, hostRegion.getAllClusters(), mapId);

      if (!connections.isEmpty())
         return new SingleSourceVisibilityMap(source, connections, hostRegion);

      return connectSourceToHostOrFallbackMap(source, potentialFallbackMap, hostRegion);
   }

   public static SingleSourceVisibilityMap connectToFallbackMap(Point3DReadOnly source, int sourceId, double maxConnectionLength, VisibilityMap fallbackMap)
   {

      double minDistance = Double.POSITIVE_INFINITY;
      Connection closestConnection = null;

      for (Connection connection : fallbackMap)
      {
         double distance = connection.distanceSquared(source);

         if (distance < minDistance)
         {
            minDistance = distance;
            closestConnection = connection;
         }
      }

      if (minDistance > maxConnectionLength)
         return null;

      Set<Connection> connections = new HashSet<>();
      ConnectionPoint3D sourceConnectionPoint = new ConnectionPoint3D(source, sourceId);
      double percentage = closestConnection.percentageAlongConnection(source);
      double epsilon = 1.0e-3;
      if (percentage <= epsilon)
      {
         connections.add(new Connection(sourceConnectionPoint, closestConnection.getSourcePoint()));
      }
      else if (percentage >= 1.0 - epsilon)
      {
         connections.add(new Connection(sourceConnectionPoint, closestConnection.getTargetPoint()));
      }
      else
      { // Let's create an connection point on the connection.
         ConnectionPoint3D newConnectionPoint = closestConnection.getPointGivenPercentage(percentage, sourceId);

         fallbackMap.addConnection(new Connection(closestConnection.getSourcePoint(), newConnectionPoint));
         fallbackMap.addConnection(new Connection(newConnectionPoint, closestConnection.getTargetPoint()));

         connections.add(new Connection(sourceConnectionPoint, newConnectionPoint));
      }

      return new SingleSourceVisibilityMap(source, sourceId, connections);
   }

   private static SingleSourceVisibilityMap connectSourceToHostOrFallbackMap(Point3DReadOnly sourceInworld, VisibilityMap fallbackMap,
                                                                             NavigableRegion hostRegion)
   {
      Point3D sourceInLocal = new Point3D(sourceInworld);
      hostRegion.transformFromWorldToLocal(sourceInLocal);
      int mapId = hostRegion.getMapId();

      Set<Connection> connections = new HashSet<>();
      double minDistance = Double.POSITIVE_INFINITY;
      ConnectionPoint3D closestHostPoint = null;

      VisibilityMap hostMapInLocal = hostRegion.getVisibilityMapInLocal();
      hostMapInLocal.computeVertices();

      for (ConnectionPoint3D connectionPoint : hostMapInLocal.getVertices())
      {
         double distance = connectionPoint.distanceSquared(sourceInLocal);

         if (distance < minDistance)
         {
            minDistance = distance;
            closestHostPoint = connectionPoint;
         }
      }

      Connection closestFallbackConnection = null;

      for (Connection connection : fallbackMap)
      {
         double distance = connection.distanceSquared(sourceInworld);

         if (distance < minDistance)
         {
            minDistance = distance;
            closestFallbackConnection = connection;
            closestHostPoint = null;
         }
      }

      if (closestHostPoint != null)
      { // Make the connection to the host
         ConnectionPoint3D sourceConnectionPoint = new ConnectionPoint3D(sourceInLocal, mapId);
         connections.add(new Connection(sourceConnectionPoint, closestHostPoint));
         return new SingleSourceVisibilityMap(sourceInworld, connections, hostRegion);
      }
      else
      { // Make the connection to the fallback map
         ConnectionPoint3D sourceConnectionPoint = new ConnectionPoint3D(sourceInworld, mapId);
         double percentage = closestFallbackConnection.percentageAlongConnection(sourceInworld);
         double epsilon = 1.0e-3;
         if (percentage <= epsilon)
         {
            connections.add(new Connection(sourceConnectionPoint, closestFallbackConnection.getSourcePoint()));
         }
         else if (percentage >= 1.0 - epsilon)
         {
            connections.add(new Connection(sourceConnectionPoint, closestFallbackConnection.getTargetPoint()));
         }
         else
         { // Let's create an connection point on the connection.
            ConnectionPoint3D newConnectionPoint = closestFallbackConnection.getPointGivenPercentage(percentage, mapId);

            fallbackMap.addConnection(new Connection(closestFallbackConnection.getSourcePoint(), newConnectionPoint));
            fallbackMap.addConnection(new Connection(newConnectionPoint, closestFallbackConnection.getTargetPoint()));

            connections.add(new Connection(sourceConnectionPoint, newConnectionPoint));
         }

         return new SingleSourceVisibilityMap(sourceInworld, mapId, connections);
      }
   }

   public static SingleSourceVisibilityMap connectToClosestPoints(ConnectionPoint3D source, int maximumNumberOfConnections,
                                                                  List<NavigableRegion> navigableRegions, int mapId)
   {
      List<Connection> allConnections = new ArrayList<>();

      for (int i = 0; i < navigableRegions.size(); i++)
      {
         VisibilityMap targetMap = navigableRegions.get(i).getVisibilityMapInWorld();
         Set<ConnectionPoint3D> targetPoints = targetMap.getVertices();

         for (ConnectionPoint3D targetPoint : targetPoints)
         {
            allConnections.add(new Connection(source, targetPoint));
         }
      }

      Collections.sort(allConnections, (c1, c2) -> {
         double c1LengthSquared = c1.lengthSquared();
         double c2LengthSquared = c2.lengthSquared();

         return c1LengthSquared < c2LengthSquared ? -1 : 1;
      });

      HashSet<Connection> connections = new HashSet<>();
      connections.addAll(allConnections.subList(0, maximumNumberOfConnections));

      return new SingleSourceVisibilityMap(source, mapId, connections);
   }

   public static InterRegionVisibilityMap createInterRegionVisibilityMap(List<NavigableRegion> navigableRegions, InterRegionConnectionFilter filter)
   {
      if (CREATE_INTER_REGIONS_USING_CLUSTER_POINTS)
      {
         return createInterRegionVisibilityMapUsingClusterPoints(navigableRegions, filter);
      }
      else
      {
         return createInterRegionVisibilityMapUsingInnerVisibilityMaps(navigableRegions, filter);
      }
   }

   public static InterRegionVisibilityMap createInterRegionVisibilityMapUsingClusterPoints(List<NavigableRegion> navigableRegions,
                                                                                           InterRegionConnectionFilter filter)
   {
      InterRegionVisibilityMap map = new InterRegionVisibilityMap();

      for (int sourceMapIndex = 0; sourceMapIndex < navigableRegions.size(); sourceMapIndex++)
      {
         NavigableRegion sourceRegion = navigableRegions.get(sourceMapIndex);
         for (int targetMapIndex = sourceMapIndex + 1; targetMapIndex < navigableRegions.size(); targetMapIndex++)
         {
            NavigableRegion targetRegion = navigableRegions.get(targetMapIndex);

            ArrayList<Connection> connectionsBetweenTwoNavigableRegions = createInterRegionVisibilityConnectionsUsingClusterPoints(sourceRegion, targetRegion,
                                                                                                                                   filter);
            map.addConnections(connectionsBetweenTwoNavigableRegions);
         }
      }

      return map;
   }

   public static ArrayList<Connection> createInterRegionVisibilityConnectionsUsingClusterPoints(NavigableRegion sourceRegion, NavigableRegion targetRegion,
                                                                                                InterRegionConnectionFilter filter)
   {
      ArrayList<Connection> connections = new ArrayList<Connection>();

      int sourceId = sourceRegion.getMapId();
      int targetId = targetRegion.getMapId();

      if (sourceId == targetId)
         return connections;

      List<Cluster> sourceClusters = sourceRegion.getAllClusters();
      List<Cluster> sourceObstacleClusters = sourceRegion.getObstacleClusters();
      for (Cluster sourceCluster : sourceClusters)
      {
         List<Point3DReadOnly> sourcePoints = sourceCluster.getNavigableExtrusionsInWorld();

         List<Cluster> targetClusters = targetRegion.getAllClusters();
         List<Cluster> targetObstacleClusters = targetRegion.getObstacleClusters();
         for (Cluster targetCluster : targetClusters)
         {
            List<Point3DReadOnly> targetPoints = targetCluster.getNavigableExtrusionsInWorld();

            for (Point3DReadOnly sourcePoint3D : sourcePoints)
            {
               for (Point3DReadOnly targetPoint3D : targetPoints)
               {
                  ConnectionPoint3D source = new ConnectionPoint3D(sourcePoint3D, targetId);
                  ConnectionPoint3D target = new ConnectionPoint3D(targetPoint3D, targetId);

                  if (filter.isConnectionValid(source, target))
                  {
                     //TODO: +++JEP: Taking up lots of time, but necessary
                     Point2D sourcePoint2DInLocal = getPoint2DInLocal(sourceRegion, sourcePoint3D);

                     PlanarRegion sourceHomeRegion = sourceRegion.getHomeRegion();
                     PlanarRegion targetHomeRegion = targetRegion.getHomeRegion();

                     if (!PlanarRegionTools.isPointInLocalInsidePlanarRegion(sourceHomeRegion, sourcePoint2DInLocal))
                        continue;

                     //TODO: +++JEP: Taking up lots of time, but necessary
                     Point2D targetPoint2DInLocal = getPoint2DInLocal(targetRegion, targetPoint3D);
                     if (!PlanarRegionTools.isPointInLocalInsidePlanarRegion(targetHomeRegion, targetPoint2DInLocal))
                        continue;

                     Connection connection = new Connection(source, target);
                     //+++JEP

                     boolean sourceIsInsideNoGoZone = isInsideANonNavigableZone(sourcePoint2DInLocal, sourceObstacleClusters);
                     if (sourceIsInsideNoGoZone)
                        continue;

                     boolean targetIsInsideNoGoZone = isInsideANonNavigableZone(targetPoint2DInLocal, targetObstacleClusters);
                     if (targetIsInsideNoGoZone)
                        continue;

                     connections.add(connection);
                  }
               }
            }
         }
      }

      return connections;
   }

   private static boolean isInsideANonNavigableZone(Point2D pointInLocal, List<Cluster> clusters)
   {
      for (Cluster cluster : clusters)
      {
         if (cluster.isInsideNonNavigableZone(pointInLocal))
            return true;
      }
      return false;
   }

   private static Point2D getPoint2DInLocal(NavigableRegion region, Point3DReadOnly point3DInWorld)
   {
      Point3D pointInLocal = new Point3D();
      pointInLocal.set(point3DInWorld);
      region.transformFromWorldToLocal(pointInLocal);
      Point2D pointInLocal2D = new Point2D(pointInLocal.getX(), pointInLocal.getY());
      return pointInLocal2D;
   }

   public static InterRegionVisibilityMap createInterRegionVisibilityMapUsingInnerVisibilityMaps(List<NavigableRegion> navigableRegions,
                                                                                                 InterRegionConnectionFilter filter)
   {
      InterRegionVisibilityMap map = new InterRegionVisibilityMap();

      for (int sourceMapIndex = 0; sourceMapIndex < navigableRegions.size(); sourceMapIndex++)
      {
         VisibilityMap sourceMap = navigableRegions.get(sourceMapIndex).getVisibilityMapInWorld();
         Set<ConnectionPoint3D> sourcePoints = sourceMap.getVertices();

         for (ConnectionPoint3D source : sourcePoints)
         {
            for (int targetMapIndex = sourceMapIndex + 1; targetMapIndex < navigableRegions.size(); targetMapIndex++)
            {
               NavigableRegion targetRegion = navigableRegions.get(targetMapIndex);

               VisibilityMap targetMap = targetRegion.getVisibilityMapInWorld();
               Set<ConnectionPoint3D> targetPoints = targetMap.getVertices();

               for (ConnectionPoint3D target : targetPoints)
               {
                  if (source.getRegionId() == target.getRegionId())
                  {
                     continue;
                  }

                  if (filter.isConnectionValid(source, target))
                  {
                     map.addConnection(source, target);
                  }
               }
            }
         }
      }

      return map;
   }
}
