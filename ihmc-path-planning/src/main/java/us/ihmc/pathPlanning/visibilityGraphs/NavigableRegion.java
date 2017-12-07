package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.robotics.geometry.PlanarRegion;

/**
 * User: Matt Date: 1/14/13
 */
public class NavigableRegion
{
   private static final boolean debug = false;

   private List<Cluster> clusters = new ArrayList<>();
   private List<PlanarRegion> lineObstacleRegions = new ArrayList<>();
   private List<PlanarRegion> polygonObstacleRegions = new ArrayList<>();
   private List<PlanarRegion> regionsInsideHomeRegion = new ArrayList<>();
   private ReferenceFrame localReferenceFrame;
   private PlanarRegion homeRegion;
   private VisibilityMap localVisibilityMap;

   public NavigableRegion(PlanarRegion homeRegion)
   {
      this.homeRegion = homeRegion;

      createLocalReferenceFrame();
   }

   private void createLocalReferenceFrame()
   {
      localReferenceFrame = new ReferenceFrame("regionLocalFrame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            homeRegion.getTransformToWorld(transformToParent);
         }
      };

      localReferenceFrame.update();
   }

   public PlanarRegion getHomeRegion()
   {
      return homeRegion;
   }

   public ReferenceFrame getLocalReferenceFrame()
   {
      return localReferenceFrame;
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
   
   public void setVisibilityMap(VisibilityMap visibilityMap)
   {
      localVisibilityMap = visibilityMap;
   }

   public VisibilityMap getLocalVisibilityGraph()
   {
      return localVisibilityMap;
   }

   public int getRegionId()
   {
      return homeRegion.getRegionId();
   }
   
   public void setClusters(List<Cluster> clusters)
   {
      this.clusters = clusters;
   }

   public List<Cluster> getClusters()
   {
      return clusters;
   }
}