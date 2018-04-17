package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
import us.ihmc.robotics.geometry.PlanarRegion;

/**
 * User: Matt Date: 1/14/13
 */
public class NavigableRegion implements VisibilityMapHolder
{
   private final PlanarRegion homeRegion;
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();

   private Cluster homeRegionCluster = null;
   private List<Cluster> obstacleClusters = new ArrayList<>();
   private List<Cluster> allClusters = new ArrayList<>();
   private VisibilityMap visibilityMapInLocal = null;
   private VisibilityMap visibilityMapInWorld = null;

   public NavigableRegion(PlanarRegion homeRegion)
   {
      this.homeRegion = homeRegion;
      homeRegion.getTransformToWorld(transformToWorld);
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

   public void setVisibilityMapInLocal(VisibilityMap visibilityMap)
   {
      visibilityMapInLocal = visibilityMap;
   }

   public PlanarRegion getHomeRegion()
   {
      return homeRegion;
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

   @Override
   public int getMapId()
   {
      return homeRegion.getRegionId();
   }

   @Override
   public VisibilityMap getVisibilityMapInLocal()
   {
      return visibilityMapInLocal;
   }

   @Override
   public VisibilityMap getVisibilityMapInWorld()
   {
      if (visibilityMapInWorld == null)
      {
         visibilityMapInWorld = new VisibilityMap(visibilityMapInLocal.getConnections());
         transformFromLocalToWorld(visibilityMapInWorld);
         visibilityMapInWorld.computeVertices();
      }

      return visibilityMapInWorld;
   }
}