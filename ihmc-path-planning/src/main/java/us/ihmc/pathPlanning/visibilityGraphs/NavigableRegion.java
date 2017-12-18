package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.robotics.geometry.PlanarRegion;

/**
 * User: Matt Date: 1/14/13
 */
public class NavigableRegion
{
   private Cluster homeRegionCluster = null;
   private List<Cluster> obstacleClusters = new ArrayList<>();
   private List<Cluster> allClusters = new ArrayList<>();
   private List<PlanarRegion> lineObstacleRegions = new ArrayList<>();
   private List<PlanarRegion> polygonObstacleRegions = new ArrayList<>();
   private List<PlanarRegion> regionsInsideHomeRegion = new ArrayList<>();
   private VisibilityMap visibilityMapInLocal = null;
   private VisibilityMap visibilityMapInWorld = null;

   private final PlanarRegion homeRegion;
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();

   public NavigableRegion(PlanarRegion homeRegion)
   {
      this.homeRegion = homeRegion;
      homeRegion.getTransformToWorld(transformToWorld);
   }

   public PlanarRegion getHomeRegion()
   {
      return homeRegion;
   }

   public RigidBodyTransform getTransformToWorld()
   {
      return new RigidBodyTransform(transformToWorld);
   }

   public void transformFromLocalToWorld(Transformable objectToTransformToWorld)
   {
      objectToTransformToWorld.applyTransform(transformToWorld);
   }

   public void transformFromWorldToLocal(Transformable objectToTransformToWorld)
   {
      objectToTransformToWorld.applyInverseTransform(transformToWorld);
   }

   public void setRegionsInsideHomeRegion(List<PlanarRegion> regionsInsideHomeRegion)
   {
      this.regionsInsideHomeRegion = regionsInsideHomeRegion;
   }

   public List<PlanarRegion> getRegionsInside()
   {
      return regionsInsideHomeRegion;
   }

   public void setPolygonObstacleRegions(List<PlanarRegion> polygonObstacleRegions)
   {
      this.polygonObstacleRegions = polygonObstacleRegions;
   }

   public List<PlanarRegion> getPolygonObstacleRegions()
   {
      return polygonObstacleRegions;
   }

   public void setLineObstacleRegions(List<PlanarRegion> lineObstacleRegions)
   {
      this.lineObstacleRegions = lineObstacleRegions;
   }

   public List<PlanarRegion> getLineObstacleRegions()
   {
      return lineObstacleRegions;
   }

   public void setVisibilityMapInLocal(VisibilityMap visibilityMap)
   {
      visibilityMapInLocal = visibilityMap;
   }

   public VisibilityMap getVisibilityGraphInLocal()
   {
      return visibilityMapInLocal;
   }

   public VisibilityMap getVisibilityGraphInWorld()
   {
      if (visibilityMapInWorld == null)
      {
         visibilityMapInWorld = new VisibilityMap(visibilityMapInLocal.getConnections());
         transformFromLocalToWorld(visibilityMapInWorld);
         visibilityMapInWorld.computeVertices();
      }

      return visibilityMapInWorld;
   }

   public int getRegionId()
   {
      return homeRegion.getRegionId();
   }

   public void setClusters(List<Cluster> clusters)
   {
      this.allClusters = clusters;
      homeRegionCluster = allClusters.stream().filter(cluster -> !cluster.isHomeRegion()).findFirst().orElse(null);
      allClusters.stream().filter(cluster -> !cluster.isHomeRegion()).forEach(obstacleClusters::add);
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
}