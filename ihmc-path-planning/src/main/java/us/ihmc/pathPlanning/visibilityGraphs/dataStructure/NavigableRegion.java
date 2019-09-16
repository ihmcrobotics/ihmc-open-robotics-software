package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.robotics.geometry.PlanarRegion;

public class NavigableRegion
{
   private final PlanarRegion homePlanarRegion;

   private Cluster homeRegionCluster = null;
   private List<Cluster> obstacleClusters = new ArrayList<>();
   private List<Cluster> allClusters = new ArrayList<>();

   public NavigableRegion(PlanarRegion homePlanarRegion)
   {
      this.homePlanarRegion = homePlanarRegion;
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

   public RigidBodyTransformReadOnly getTransformToWorld()
   {
      return homePlanarRegion.getTransformToWorld();
   }

   public RigidBodyTransformReadOnly getTransformFromWorldToLocal()
   {
      return homePlanarRegion.getTransformToLocal();
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

   public double getPlaneZGivenXY(double xWorld, double yWorld)
   {
      return homePlanarRegion.getPlaneZGivenXY(xWorld, yWorld);
   }


   public void transformFromLocalToWorld(Transformable objectToTransformToWorld)
   {
      homePlanarRegion.transformFromLocalToWorld(objectToTransformToWorld);
   }

   public void transformFromWorldToLocal(Transformable objectToTransformToWorld)
   {
      homePlanarRegion.transformFromWorldToLocal(objectToTransformToWorld);
   }

   public int getMapId()
   {
      return homePlanarRegion.getRegionId();
   }
}
