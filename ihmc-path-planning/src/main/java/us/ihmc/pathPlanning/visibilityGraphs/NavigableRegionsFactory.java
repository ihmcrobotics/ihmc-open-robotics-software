package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.ExtrusionHull;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ClusterInfo;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.NavigableExtrusionDistanceCalculator;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.NavigableRegionFilter;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.ObstacleExtrusionDistanceCalculator;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.ObstacleRegionFilter;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionFilter;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.ClusterTools;
import us.ihmc.robotEnvironmentAwareness.planarRegion.REAPlanarRegionTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PointCloudTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.tools.lists.PairList;

public class NavigableRegionsFactory
{
   private static final double DEPTH_THRESHOLD_FOR_CONVEX_DECOMPOSITION = 0.05;
   private static final boolean AUTO_POPULATE_CLUSTERS = false;

   public static List<NavigableRegion> createNavigableRegions(List<PlanarRegion> allRegions, VisibilityGraphsParametersReadOnly parameters)
   {
      List<NavigableRegion> navigableRegions = new ArrayList<>(allRegions.size());

      NavigableRegionFilter navigableRegionFilter = parameters.getNavigableRegionFilter();

      for (int candidateIndex = 0; candidateIndex < allRegions.size(); candidateIndex++)
      {
         PlanarRegion candidate = allRegions.get(candidateIndex);

         List<PlanarRegion> otherRegions = new ArrayList<>(allRegions);
         otherRegions.remove(candidate);

         if (!navigableRegionFilter.isPlanarRegionNavigable(candidate, otherRegions))
            continue;

         NavigableRegion navigableRegion = createNavigableRegion(candidate, otherRegions, parameters);
         navigableRegions.add(navigableRegion);
      }

      return navigableRegions;
   }

   public static NavigableRegion createNavigableRegion(PlanarRegion region, List<PlanarRegion> otherRegions, VisibilityGraphsParametersReadOnly parameters)
   {
      PlanarRegionFilter planarRegionFilter = parameters.getPlanarRegionFilter();
      double orthogonalAngle = parameters.getRegionOrthogonalAngle();
      double clusterResolution = parameters.getClusterResolution();
      NavigableExtrusionDistanceCalculator preferredNavigableCalculator = parameters.getPreferredNavigableExtrusionDistanceCalculator();
      NavigableExtrusionDistanceCalculator navigableCalculator = parameters.getNavigableExtrusionDistanceCalculator();
      ObstacleExtrusionDistanceCalculator preferredObstacleCalculator = parameters.getPreferredObstacleExtrusionDistanceCalculator();
      ObstacleExtrusionDistanceCalculator obstacleCalculator = parameters.getObstacleExtrusionDistanceCalculator();
      ObstacleRegionFilter obstacleRegionFilter = parameters.getObstacleRegionFilter();
      return createNavigableRegion(region, otherRegions, orthogonalAngle, clusterResolution, obstacleRegionFilter, planarRegionFilter,
                                   preferredNavigableCalculator, navigableCalculator, preferredObstacleCalculator, obstacleCalculator,
                                   parameters.includePreferredExtrusions());
   }

   public static NavigableRegion createNavigableRegion(PlanarRegion region, List<PlanarRegion> otherRegions, double orthogonalAngle, double clusterResolution,
                                                       ObstacleRegionFilter obstacleRegionFilter, PlanarRegionFilter filter,
                                                       NavigableExtrusionDistanceCalculator preferredNavigableCalculator,
                                                       NavigableExtrusionDistanceCalculator navigableCalculator,
                                                       ObstacleExtrusionDistanceCalculator preferredObstacleCalculator,
                                                       ObstacleExtrusionDistanceCalculator obstacleCalculator,
                                                       boolean includePreferredExtrusions)
   {
      ClusterInfo clusterInfo = new ClusterInfo(otherRegions, orthogonalAngle, clusterResolution, obstacleRegionFilter, filter,
                                                        preferredNavigableCalculator, navigableCalculator, preferredObstacleCalculator,
                                                        obstacleCalculator, includePreferredExtrusions);
      NavigableRegion navigableRegion = new NavigableRegion(region, clusterInfo);

      if (AUTO_POPULATE_CLUSTERS)
         navigableRegion.populateClusters();

      return navigableRegion;
   }

   public static void populateNavigableRegionCluster(NavigableRegion navigableRegionToPack, List<PlanarRegion> otherRegions, double orthogonalAngle, double clusterResolution,
                                                     ObstacleRegionFilter obstacleRegionFilter, PlanarRegionFilter planarRegionFilter,
                                                     NavigableExtrusionDistanceCalculator preferredNavigableCalculator,
                                                     NavigableExtrusionDistanceCalculator navigableCalculator,
                                                     ObstacleExtrusionDistanceCalculator preferredObstacleCalculator,
                                                     ObstacleExtrusionDistanceCalculator obstacleCalculator, boolean includePreferredExtrusions)
   {
      PlanarRegion homeRegion = navigableRegionToPack.getHomePlanarRegion();

      List<PlanarRegion> obstacleRegions = otherRegions.stream().filter(candidate -> obstacleRegionFilter.isRegionValidObstacle(candidate, homeRegion))
                                                       .collect(Collectors.toList());

      obstacleRegions = REAPlanarRegionTools
            .filterRegionsByTruncatingVerticesBeneathHomeRegion(obstacleRegions, homeRegion, DEPTH_THRESHOLD_FOR_CONVEX_DECOMPOSITION, planarRegionFilter);

      Cluster homeRegionCluster = ClusterTools.createHomeRegionCluster(homeRegion, preferredNavigableCalculator, navigableCalculator, includePreferredExtrusions);
      PairList<Cluster, PlanarRegion> obstacleClusters = ClusterTools.createObstacleClusters(homeRegion, obstacleRegions, orthogonalAngle, preferredObstacleCalculator,
                                                                                             obstacleCalculator, includePreferredExtrusions);

      // fills long edges with interpolated points for visibility graph building later
      addPointsAlongPolygon(homeRegionCluster, clusterResolution);
      obstacleClusters.forEach(cluster -> addPointsAlongPolygon(cluster.getLeft(), clusterResolution));

      navigableRegionToPack.setHomeRegionCluster(homeRegionCluster);
      navigableRegionToPack.addObstacleClusters(obstacleClusters);
   }

   private static void addPointsAlongPolygon(Cluster cluster, double clusterResolution)
   {
      ExtrusionHull expandListOf2DPoints = PointCloudTools.addPointsAlongExtrusionHull(cluster.getNavigableExtrusionsInLocal(), clusterResolution);
      List<ExtrusionHull> currentListOfPreferred2DPoints = cluster.getPreferredNavigableExtrusionsInLocal();
      List<ExtrusionHull> expandedListOfPreferred2DPoints = currentListOfPreferred2DPoints.stream().map(extrusion -> PointCloudTools
            .addPointsAlongExtrusionHull(extrusion, clusterResolution)).collect(Collectors.toList());
      cluster.setNavigableExtrusionsInLocal(expandListOf2DPoints);
      cluster.setPreferredNavigableExtrusionsInLocal(expandedListOfPreferred2DPoints);
   }
}
