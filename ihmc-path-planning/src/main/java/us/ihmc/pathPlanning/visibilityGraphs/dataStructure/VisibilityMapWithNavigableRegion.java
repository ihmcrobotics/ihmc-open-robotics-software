package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import java.util.List;

import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
import us.ihmc.robotics.geometry.PlanarRegion;

public class VisibilityMapWithNavigableRegion implements VisibilityMapHolder
{
   private VisibilityMap visibilityMapInLocal = null;
   private VisibilityMap visibilityMapInWorld = null;

   private final NavigableRegion navigableRegion;

   public VisibilityMapWithNavigableRegion(VisibilityMapWithNavigableRegion visibilityMapWithNavigableRegion)
   {
      this.navigableRegion = visibilityMapWithNavigableRegion.navigableRegion;
      this.visibilityMapInLocal = visibilityMapWithNavigableRegion.visibilityMapInLocal;
      this.visibilityMapInWorld = visibilityMapWithNavigableRegion.visibilityMapInWorld;
   }

   public VisibilityMapWithNavigableRegion(PlanarRegion homePlanarRegion, ClusterInfo clusterInfo)
   {
      this.navigableRegion = new NavigableRegion(homePlanarRegion, clusterInfo);
   }

   public VisibilityMapWithNavigableRegion(NavigableRegion navigableRegion)
   {
      this.navigableRegion = navigableRegion;
   }

   public NavigableRegion getNavigableRegion()
   {
      return navigableRegion;
   }

   public PlanarRegion getHomePlanarRegion()
   {
      return navigableRegion.getHomePlanarRegion();
   }

   public RigidBodyTransformReadOnly getTransformToWorld()
   {
      return navigableRegion.getTransformToWorld();
   }

   public Cluster getHomeRegionCluster()
   {
      return navigableRegion.getHomeRegionCluster();
   }

   public List<Cluster> getObstacleClusters()
   {
      return navigableRegion.getObstacleClusters();
   }

   public List<Cluster> getAllClusters()
   {
      return navigableRegion.getAllClusters();
   }

   public void transformFromLocalToWorld(Transformable objectToTransformToWorld)
   {
      navigableRegion.transformFromLocalToWorld(objectToTransformToWorld);
   }

   public void transformFromWorldToLocal(Transformable objectToTransformToWorld)
   {
      navigableRegion.transformFromWorldToLocal(objectToTransformToWorld);
   }

   @Override
   public int getMapId()
   {
      return navigableRegion.getMapId();
   }

   public void setVisibilityMapInLocal(VisibilityMap visibilityMap)
   {
      visibilityMapInLocal = visibilityMap;
   }

   public void setVisibilityMapInWorld(VisibilityMap visibilityMap)
   {
      if (visibilityMapInLocal == null)
         visibilityMapInLocal = new VisibilityMap();

      visibilityMapInLocal.copy(visibilityMap);
      transformFromWorldToLocal(visibilityMapInLocal);
      visibilityMapInLocal.computeVertices();
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
         visibilityMapInWorld = new VisibilityMap();
         visibilityMapInWorld.copy(visibilityMapInLocal);
         transformFromLocalToWorld(visibilityMapInWorld);
         visibilityMapInWorld.computeVertices();
      }

      return visibilityMapInWorld;
   }

}