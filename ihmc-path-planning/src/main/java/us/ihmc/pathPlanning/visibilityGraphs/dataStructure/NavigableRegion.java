package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionsFactory;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.robotics.geometry.PlanarRegion;

/**
 * A planar region with clusters of points for itself and nearby obstacles.
 * This extra data is gathered with respect to other planar regions in the world.
 */
public class NavigableRegion
{
   private final PlanarRegion homePlanarRegion;
   private ClusterInfo clusterInfo;
   private boolean haveClustersBeenPopulated = false;

   private Cluster homeRegionCluster = null;
   private List<Cluster> obstacleClusters = new ArrayList<>();
   private List<PlanarRegion> obstacleRegions = new ArrayList<>();
   private List<Cluster> allClusters = new ArrayList<>();

   public NavigableRegion(PlanarRegion homePlanarRegion, Cluster homeRegionCluster, List<Cluster> obstacleClusters)
   {
      this.homePlanarRegion = homePlanarRegion;
      this.homeRegionCluster = homeRegionCluster;
      this.obstacleClusters.addAll(obstacleClusters);
      this.allClusters.addAll(obstacleClusters);
      this.allClusters.add(homeRegionCluster);
      haveClustersBeenPopulated = true;
   }

   public NavigableRegion(PlanarRegion homePlanarRegion, ClusterInfo clusterInfo)
   {
      this.homePlanarRegion = homePlanarRegion;
      this.clusterInfo = clusterInfo;
   }

   public void setHomeRegionCluster(Cluster homeCluster)
   {
      this.homeRegionCluster = homeCluster;
      allClusters.add(homeCluster);
   }

   public void addObstacleClusters(Iterable<ImmutablePair<Cluster, PlanarRegion>> obstacleClusters)
   {
      obstacleClusters.forEach(pair -> addObstacleCluster(pair.getLeft(), pair.getRight()));
   }

   public void addObstacleCluster(Cluster obstacleCluster, PlanarRegion associatedRegion)
   {
      obstacleClusters.add(obstacleCluster);
      obstacleRegions.add(associatedRegion);
      allClusters.add(obstacleCluster);
   }

   public PlanarRegion getHomePlanarRegion()
   {
      return homePlanarRegion;
   }

   public RigidBodyTransformReadOnly getTransformToWorld()
   {
      return getHomePlanarRegion().getTransformToWorld();
   }

   public RigidBodyTransformReadOnly getTransformFromWorldToLocal()
   {
      return getHomePlanarRegion().getTransformToLocal();
   }

   public void transformFromLocalToWorld(Transformable objectToTransformToWorld)
   {
      getHomePlanarRegion().transformFromLocalToWorld(objectToTransformToWorld);
   }

   public void transformFromWorldToLocal(Transformable objectToTransformToWorld)
   {
      getHomePlanarRegion().transformFromWorldToLocal(objectToTransformToWorld);
   }

   public double getPlaneZGivenXY(double xWorld, double yWorld)
   {
      return getHomePlanarRegion().getPlaneZGivenXY(xWorld, yWorld);
   }

   public int getMapId()
   {
      return getHomePlanarRegion().getRegionId();
   }

   public Cluster getHomeRegionCluster()
   {
      populateClusters();

      return homeRegionCluster;
   }

   public List<Cluster> getObstacleClusters()
   {
      populateClusters();

      return obstacleClusters;
   }

   public List<PlanarRegion> getObstacleRegions()
   {
      populateClusters();

      return obstacleRegions;
   }

   public List<Cluster> getAllClusters()
   {
      populateClusters();

      return allClusters;
   }

   public void populateClusters()
   {
      if (haveClustersBeenPopulated)
         return;

      NavigableRegionsFactory.populateNavigableRegionCluster(this, clusterInfo.getOtherRegions(), clusterInfo.getOrthogonalAngle(),
                                                             clusterInfo.getClusterResolution(), clusterInfo.getObstacleRegionFilter(),
                                                             clusterInfo.getPlanarRegionFilter(),
                                                             clusterInfo.getNavigableCalculator(),
                                                             clusterInfo.getObstacleCalculator());

      haveClustersBeenPopulated = true;
   }
}
