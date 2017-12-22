package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.ExtrusionDistanceCalculator;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.InterRegionConnectionFilter;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.NavigableRegionFilter;
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
      NavigableRegionFilter navigableRegionFilter = parameters.getNavigableRegionFilter();

      return allRegions.stream().filter(navigableRegionFilter::isPlanarRegionNavigable).map(region -> createNavigableRegion(region, allRegions, parameters))
                       .collect(Collectors.toList());
   }

   public static NavigableRegion createNavigableRegion(PlanarRegion region, List<PlanarRegion> allRegions, VisibilityGraphsParameters parameters)
   {
      PlanarRegionFilter planarRegionFilter = parameters.getPlanarRegionFilter();
      double orthogonalAngle = parameters.getRegionOrthogonalAngle();
      double clusterResolution = parameters.getClusterResolution();
      ExtrusionDistanceCalculator extrusionDistanceCalculator = parameters.getExtrusionDistanceCalculator();
      ObstacleRegionFilter obstacleRegionFilter = parameters.getObstacleRegionFilter();
      return createNavigableRegion(region, allRegions, orthogonalAngle, clusterResolution, obstacleRegionFilter, planarRegionFilter,
                                   extrusionDistanceCalculator);
   }

   public static NavigableRegion createNavigableRegion(PlanarRegion region, List<PlanarRegion> allRegions, double orthogonalAngle, double clusterResolution,
                                                       ObstacleRegionFilter obstacleRegionFilter, PlanarRegionFilter filter,
                                                       ExtrusionDistanceCalculator extrusionDistanceCalculator)
   {
      NavigableRegion navigableRegion = new NavigableRegion(region);
      List<PlanarRegion> lineObstacleRegions = new ArrayList<>();
      List<PlanarRegion> polygonObstacleRegions = new ArrayList<>();
      PlanarRegion homeRegion = navigableRegion.getHomeRegion();

      List<PlanarRegion> obstacleRegions = allRegions.stream().filter(candidate -> candidate != homeRegion)
                                                     .filter(candidate -> obstacleRegionFilter.isRegionValidObstacle(candidate, homeRegion))
                                                     .collect(Collectors.toList());

      double depthThresholdForConvexDecomposition = 0.05; // TODO Extract me!
      obstacleRegions = PlanarRegionTools.filterRegionsByTruncatingVerticesBeneathHomeRegion(obstacleRegions, homeRegion, depthThresholdForConvexDecomposition,
                                                                                             filter);

      ClusterTools.classifyExtrusions(obstacleRegions, homeRegion, lineObstacleRegions, polygonObstacleRegions, orthogonalAngle);
      Cluster homeRegionCluster = ClusterTools.createHomeRegionCluster(homeRegion);
      List<Cluster> obstacleClusters = ClusterTools.createObstacleClusters(homeRegion, lineObstacleRegions, polygonObstacleRegions);

      navigableRegion.setHomeRegionCluster(homeRegionCluster);
      navigableRegion.addObstacleClusters(obstacleClusters);

      ClusterTools.performExtrusions(extrusionDistanceCalculator, navigableRegion.getAllClusters());

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

      Set<Connection> connections = VisibilityTools.createStaticVisibilityMap(sourceInLocal, mapId, hostRegion.getAllClusters(), mapId, true);
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
