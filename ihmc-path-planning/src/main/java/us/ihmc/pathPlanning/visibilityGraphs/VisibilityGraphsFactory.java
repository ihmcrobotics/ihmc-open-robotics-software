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
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.NavigableRegionFilter;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.PlanarRegionFilter;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.tools.ClusterTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PointCloudTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityTools;
import us.ihmc.robotics.geometry.PlanarRegion;

public class VisibilityGraphsFactory
{

   public static NavigableRegion createNavigableRegion(PlanarRegion region, List<PlanarRegion> allRegions, double orthogonalAngle, double clusterResolution,
                                                       PlanarRegionFilter filter, ExtrusionDistanceCalculator extrusionDistanceCalculator)
   {
      NavigableRegion navigableRegion = new NavigableRegion(region);
      List<PlanarRegion> lineObstacleRegions = new ArrayList<>();
      List<PlanarRegion> polygonObstacleRegions = new ArrayList<>();
      List<PlanarRegion> regionsInsideHomeRegion = new ArrayList<>();
      PlanarRegion homeRegion = navigableRegion.getHomeRegion();
   
      regionsInsideHomeRegion = PlanarRegionTools.determineWhichRegionsAreInside(homeRegion, allRegions);
      double depthThresholdForConvexDecomposition = 0.05; // TODO Extract me!
      regionsInsideHomeRegion = PlanarRegionTools.filterRegionsByTruncatingVerticesBeneathHomeRegion(regionsInsideHomeRegion, homeRegion,
                                                                                                     depthThresholdForConvexDecomposition, filter);
   
      ClusterTools.classifyExtrusions(regionsInsideHomeRegion, homeRegion, lineObstacleRegions, polygonObstacleRegions, orthogonalAngle);
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

   public static NavigableRegion createNavigableRegion(PlanarRegion region, List<PlanarRegion> allRegions, VisibilityGraphsParameters parameters)
   {
      PlanarRegionFilter planarRegionFilter = parameters.getPlanarRegionFilter();
      double orthogonalAngle = parameters.getRegionOrthogonalAngle();
      double clusterResolution = parameters.getClusterResolution();
      ExtrusionDistanceCalculator extrusionDistanceCalculator = parameters.getExtrusionDistanceCalculator();
      return createNavigableRegion(region, allRegions, orthogonalAngle, clusterResolution, planarRegionFilter, extrusionDistanceCalculator);
   }

   static List<NavigableRegion> createNavigableRegions(List<PlanarRegion> allRegions, VisibilityGraphsParameters parameters)
   {
      NavigableRegionFilter navigableRegionFilter = parameters.getNavigableRegionFilter();
   
      return allRegions.stream().filter(navigableRegionFilter::isPlanarRegionNavigable).map(region -> createNavigableRegion(region, allRegions, parameters))
                       .collect(Collectors.toList());
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

}
