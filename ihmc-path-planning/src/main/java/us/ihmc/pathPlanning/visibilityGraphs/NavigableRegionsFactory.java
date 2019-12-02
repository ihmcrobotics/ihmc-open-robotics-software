package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.ExtrusionHull;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
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

public class NavigableRegionsFactory
{
   private static final double DEPTH_THRESHOLD_FOR_CONVEX_DECOMPOSITION = 0.05;

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

         NavigableRegion navigableRegion = createNavigableRegions(candidate, otherRegions, parameters);
         navigableRegions.add(navigableRegion);
      }

      return navigableRegions;
   }

   public static NavigableRegion createNavigableRegions(PlanarRegion region, List<PlanarRegion> otherRegions, VisibilityGraphsParametersReadOnly parameters)
   {
      PlanarRegionFilter planarRegionFilter = parameters.getPlanarRegionFilter();
      double orthogonalAngle = parameters.getRegionOrthogonalAngle();
      double clusterResolution = parameters.getClusterResolution();
      NavigableExtrusionDistanceCalculator preferredNavigableCalculator = parameters.getPreferredNavigableExtrusionDistanceCalculator();
      NavigableExtrusionDistanceCalculator navigableCalculator = parameters.getNavigableExtrusionDistanceCalculator();
      ObstacleExtrusionDistanceCalculator preferredObstacleCalculator = parameters.getPreferredObstacleExtrusionDistanceCalculator();
      ObstacleExtrusionDistanceCalculator obstacleCalculator = parameters.getObstacleExtrusionDistanceCalculator();
      ObstacleRegionFilter obstacleRegionFilter = parameters.getObstacleRegionFilter();
      return createNavigableRegions(region, otherRegions, orthogonalAngle, clusterResolution, obstacleRegionFilter, planarRegionFilter,
                                    preferredNavigableCalculator, navigableCalculator, preferredObstacleCalculator, obstacleCalculator,
                                    parameters.includePreferredExtrusions());
   }

   public static NavigableRegion createNavigableRegions(PlanarRegion region, List<PlanarRegion> otherRegions, double orthogonalAngle, double clusterResolution,
                                                        ObstacleRegionFilter obstacleRegionFilter, PlanarRegionFilter filter,
                                                        NavigableExtrusionDistanceCalculator preferredNavigableCalculator,
                                                        NavigableExtrusionDistanceCalculator navigableCalculator,
                                                        ObstacleExtrusionDistanceCalculator preferredObstacleCalculator,
                                                        ObstacleExtrusionDistanceCalculator obstacleCalculator,
                                                        boolean includePreferredExtrusions)
   {
      NavigableRegion navigableRegion = new NavigableRegion(region);
      PlanarRegion homeRegion = navigableRegion.getHomePlanarRegion();

      List<PlanarRegion> obstacleRegions = otherRegions.stream().filter(candidate -> obstacleRegionFilter.isRegionValidObstacle(candidate, homeRegion))
                                                       .collect(Collectors.toList());

      obstacleRegions = REAPlanarRegionTools.filterRegionsByTruncatingVerticesBeneathHomeRegion(obstacleRegions, homeRegion,
                                                                                                DEPTH_THRESHOLD_FOR_CONVEX_DECOMPOSITION, filter);

      navigableRegion.setHomeRegionCluster(ClusterTools.createHomeRegionCluster(homeRegion, preferredNavigableCalculator, navigableCalculator,
                                                                                includePreferredExtrusions));
      navigableRegion.addObstacleClusters(ClusterTools.createObstacleClusters(homeRegion, obstacleRegions, orthogonalAngle, preferredObstacleCalculator,
                                                                              obstacleCalculator, includePreferredExtrusions));

      for (Cluster cluster : navigableRegion.getAllClusters()) // fills long edges with interpolated points for visibility graph building later
      {
         ExtrusionHull expandListOf2DPoints = PointCloudTools.addPointsAlongExtrusionHull(cluster.getNavigableExtrusionsInLocal(), clusterResolution);
         List<ExtrusionHull> currentListOfPreferred2DPoints = cluster.getPreferredNavigableExtrusionsInLocal();
         List<ExtrusionHull> expandedListOfPreferred2DPoints = currentListOfPreferred2DPoints.stream().map(extrusion -> PointCloudTools
               .addPointsAlongExtrusionHull(extrusion, clusterResolution)).collect(Collectors.toList());
         cluster.setNavigableExtrusionsInLocal(expandListOf2DPoints);
         cluster.setPreferredNavigableExtrusionsInLocal(expandedListOfPreferred2DPoints);
      }

      return navigableRegion;
   }

}
