package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.robotics.geometry.PlanarRegion;

public class NavigableRegion
{
   private final PlanarRegion homePlanarRegion;

   //TODO: +++JEP: Is this transform redundant since we have the homePlanarRegion?
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();

   private Cluster homeRegionCluster = null;
   private List<Cluster> obstacleClusters = new ArrayList<>();
   private List<Cluster> allClusters = new ArrayList<>();

   public NavigableRegion(PlanarRegion homePlanarRegion)
   {
      this.homePlanarRegion = homePlanarRegion;
      homePlanarRegion.getTransformToWorld(transformToWorld);
   }

   public void setHomeRegionCluster(Cluster homeCluster)
   {
      this.homeRegionCluster = homeCluster;
      allClusters.add(homeCluster);
   }

   public void addObstacleClusters(Iterable<Cluster> obstacleClusters)
   {
      obstacleClusters.forEach(this::addObstacleCluster);
   }

   public void addObstacleCluster(Cluster obstacleCluster)
   {
      obstacleClusters.add(obstacleCluster);
      allClusters.add(obstacleCluster);
   }

   public PlanarRegion getHomePlanarRegion()
   {
      return homePlanarRegion;
   }

   public RigidBodyTransform getTransformToWorld()
   {
      return new RigidBodyTransform(transformToWorld);
   }

   public Cluster getHomeRegionCluster()
   {
      return homeRegionCluster;
   }

   public List<Cluster> getObstacleClusters()
   {
      return obstacleClusters;
   }

   public List<Point3DReadOnly> getHomeRegionNavigableExtrusionsInWorld()
   {
      return homeRegionCluster.getNavigableExtrusionsInWorld();
   }

   public List<Cluster> getAllClusters()
   {
      return allClusters;
   }

   public void transformFromLocalToWorld(Transformable objectToTransformToWorld)
   {
      objectToTransformToWorld.applyTransform(transformToWorld);
   }

   public void transformFromWorldToLocal(Transformable objectToTransformToWorld)
   {
      objectToTransformToWorld.applyInverseTransform(transformToWorld);
   }

   public int getMapId()
   {
      return homePlanarRegion.getRegionId();
   }
}
