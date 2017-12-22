package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
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

   public static List<NavigableRegion> createNavigableRegions(List<PlanarRegion> allRegions, VisibilityGraphsParameters parameters)
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

         navigableRegions.add(createNavigableRegion(candidate, otherRegions, parameters));
      }

      return navigableRegions;
   }

   public static NavigableRegion createNavigableRegion(PlanarRegion region, List<PlanarRegion> otherRegions, VisibilityGraphsParameters parameters)
   {
      PlanarRegionFilter planarRegionFilter = parameters.getPlanarRegionFilter();
      double orthogonalAngle = parameters.getRegionOrthogonalAngle();
      double clusterResolution = parameters.getClusterResolution();
      NavigableExtrusionDistanceCalculator navigableCalculator = parameters.getNavigableExtrusionDistanceCalculator();
      ObstacleExtrusionDistanceCalculator obstacleCalculator = parameters.getObstacleExtrusionDistanceCalculator();
      ObstacleRegionFilter obstacleRegionFilter = parameters.getObstacleRegionFilter();
      return createNavigableRegion(region, otherRegions, orthogonalAngle, clusterResolution, obstacleRegionFilter, planarRegionFilter, navigableCalculator,
                                   obstacleCalculator);
   }

   public static NavigableRegion createNavigableRegion(PlanarRegion region, List<PlanarRegion> otherRegions, double orthogonalAngle, double clusterResolution,
                                                       ObstacleRegionFilter obstacleRegionFilter, PlanarRegionFilter filter,
                                                       NavigableExtrusionDistanceCalculator navigableCalculator,
                                                       ObstacleExtrusionDistanceCalculator obstacleCalculator)
   {
      NavigableRegion navigableRegion = new NavigableRegion(region);
      PlanarRegion homeRegion = navigableRegion.getHomeRegion();

      List<PlanarRegion> obstacleRegions = otherRegions.stream().filter(candidate -> obstacleRegionFilter.isRegionValidObstacle(candidate, homeRegion))
                                                       .collect(Collectors.toList());

      double depthThresholdForConvexDecomposition = 0.05; // TODO Extract me!
      obstacleRegions = PlanarRegionTools.filterRegionsByTruncatingVerticesBeneathHomeRegion(obstacleRegions, homeRegion, depthThresholdForConvexDecomposition,
                                                                                             filter);

      navigableRegion.setHomeRegionCluster(ClusterTools.createHomeRegionCluster(homeRegion, navigableCalculator));
      navigableRegion.addObstacleClusters(ClusterTools.createObstacleClusters(homeRegion, obstacleRegions, orthogonalAngle, obstacleCalculator));

      for (Cluster cluster : navigableRegion.getAllClusters())
      {
         PointCloudTools.doBrakeDownOn2DPoints(cluster.getNavigableExtrusionsInLocal2D(), clusterResolution);
      }

      Collection<Connection> connectionsForMap = VisibilityTools.createStaticVisibilityMap(navigableRegion.getAllClusters(), navigableRegion);

      connectionsForMap = VisibilityTools.removeConnectionsFromExtrusionsOutsideRegions(connectionsForMap, homeRegion);
      connectionsForMap = VisibilityTools.removeConnectionsFromExtrusionsInsideNoGoZones(connectionsForMap, navigableRegion.getAllClusters());

      VisibilityMap visibilityMap = new VisibilityMap();
      visibilityMap.setConnections(connectionsForMap);
      navigableRegion.setVisibilityMapInLocal(visibilityMap);

      return navigableRegion;
   }

   public static SingleSourceVisibilityMap createSingleSourceVisibilityMap(Point3DReadOnly source, List<NavigableRegion> navigableRegions)
   {
      NavigableRegion hostRegion = PlanarRegionTools.getNavigableRegionContainingThisPoint(source, navigableRegions);
      Point3D sourceInLocal = new Point3D(source);
      hostRegion.transformFromWorldToLocal(sourceInLocal);
      int mapId = hostRegion.getMapId();

      Set<Connection> connections = VisibilityTools.createStaticVisibilityMap(sourceInLocal, mapId, hostRegion.getAllClusters(), mapId);

      if (connections.isEmpty())
      {
         double minDistance = Double.POSITIVE_INFINITY;
         ConnectionPoint3D closestConnectionPoint = null;
         ConnectionPoint3D startConnectionPoint = new ConnectionPoint3D(sourceInLocal, mapId);

         VisibilityMap hostMapInLocal = hostRegion.getVisibilityMapInLocal();
         hostMapInLocal.computeVertices();

         for (ConnectionPoint3D connectionPoint : hostMapInLocal.getVertices())
         {
            double distance = connectionPoint.distanceSquared(sourceInLocal);

            if (distance < minDistance)
            {
               minDistance = distance;
               closestConnectionPoint = connectionPoint;
            }
         }
         connections = new HashSet<>();
         connections.add(new Connection(startConnectionPoint, closestConnectionPoint));
      }

      return new SingleSourceVisibilityMap(source, connections, hostRegion);
   }

   public static InterRegionVisibilityMap createInterRegionVisibilityMap(List<NavigableRegion> navigableRegions, InterRegionConnectionFilter filter)
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
               VisibilityMap targetMap = navigableRegions.get(targetMapIndex).getVisibilityMapInWorld();

               Set<ConnectionPoint3D> targetPoints = targetMap.getVertices();

               for (ConnectionPoint3D target : targetPoints)
               {
                  if (source.getRegionId() == target.getRegionId())
                     continue;

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
