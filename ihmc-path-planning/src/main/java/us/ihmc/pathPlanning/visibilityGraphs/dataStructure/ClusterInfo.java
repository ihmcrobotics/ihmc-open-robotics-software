package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import us.ihmc.pathPlanning.visibilityGraphs.interfaces.NavigableExtrusionDistanceCalculator;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.ObstacleExtrusionDistanceCalculator;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.ObstacleRegionFilter;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionFilter;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.List;

public class ClusterInfo
{
   private final List<PlanarRegion> otherRegions;
   private final double orthogonalAngle;
   private final double clusterResolution;
   private final ObstacleRegionFilter obstacleRegionFilter;
   private final PlanarRegionFilter planarRegionFilter;
   private final NavigableExtrusionDistanceCalculator navigableCalculator;
   private final ObstacleExtrusionDistanceCalculator obstacleCalculator;

   public ClusterInfo(List<PlanarRegion> otherRegions, double orthogonalAngle, double clusterResolution, ObstacleRegionFilter obstacleRegionFilter,
                      PlanarRegionFilter planarRegionFilter,
                      NavigableExtrusionDistanceCalculator navigableCalculator,
                      ObstacleExtrusionDistanceCalculator obstacleCalculator)
   {
      this.otherRegions = otherRegions;
      this.orthogonalAngle = orthogonalAngle;
      this.clusterResolution = clusterResolution;
      this.obstacleRegionFilter = obstacleRegionFilter;
      this.planarRegionFilter = planarRegionFilter;
      this.navigableCalculator = navigableCalculator;
      this.obstacleCalculator = obstacleCalculator;
   }

   public List<PlanarRegion> getOtherRegions()
   {
      return otherRegions;
   }

   public double getOrthogonalAngle()
   {
      return orthogonalAngle;
   }

   public double getClusterResolution()
   {
      return clusterResolution;
   }

   public ObstacleRegionFilter getObstacleRegionFilter()
   {
      return obstacleRegionFilter;
   }

   public PlanarRegionFilter getPlanarRegionFilter()
   {
      return planarRegionFilter;
   }

   public NavigableExtrusionDistanceCalculator getNavigableCalculator()
   {
      return navigableCalculator;
   }

   public ObstacleExtrusionDistanceCalculator getObstacleCalculator()
   {
      return obstacleCalculator;
   }
}
