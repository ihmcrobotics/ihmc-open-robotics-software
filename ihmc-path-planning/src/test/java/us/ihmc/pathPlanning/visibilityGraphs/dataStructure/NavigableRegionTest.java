package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import org.junit.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.pathPlanning.visibilityGraphs.VisibilityGraphRandomTools;
import us.ihmc.pathPlanning.visibilityGraphs.VisibilityGraphTestTools;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionToolsTest;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTestTools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class NavigableRegionTest
{
   private final static int iters = 100;

   private static final double epsilon = 1e-12;

   @Test(timeout = 30000)
   public void test()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         PlanarRegion planarRegion = PlanarRegion
               .generatePlanarRegionFromRandomPolygonsWithRandomTransform(random, random.nextInt(10), RandomNumbers.nextDouble(random, 0.0, 30.0),
                                                                          random.nextInt(10));
         VisibilityMapHolder visibilityMapHolder = VisibilityGraphRandomTools.getRandomSingleSourceVisibilityMap(random);
         Cluster homeRegionCluster = VisibilityGraphRandomTools.getRandomCluster(random);
         int numberOfObstacleClusters = random.nextInt(100);
         List<Cluster> obstacleClusters = new ArrayList<>();
         for (int i = 0; i < numberOfObstacleClusters; i++)
         {
            obstacleClusters.add(VisibilityGraphRandomTools.getRandomCluster(random));
         }

         NavigableRegion navigableRegion = new NavigableRegion(planarRegion);
         navigableRegion.setVisibilityMapInLocal(visibilityMapHolder.getVisibilityMapInLocal());
         navigableRegion.setHomeRegionCluster(homeRegionCluster);
         for (int i = 0; i < numberOfObstacleClusters; i++)
            navigableRegion.addObstacleCluster(obstacleClusters.get(i));

         PlanarRegionTestTools.assertPlanarRegionsEqual(planarRegion, navigableRegion.getHomePlanarRegion(), epsilon);
         VisibilityGraphTestTools.assertVisibilityMapsEqual(navigableRegion.getVisibilityMapInLocal(), navigableRegion.getVisibilityMapInLocal(), epsilon);
         VisibilityGraphTestTools.assertVisibilityMapsEqual(navigableRegion.getVisibilityMapInWorld(), navigableRegion.getVisibilityMapInWorld(), epsilon);
         VisibilityGraphTestTools.assertClustersEqual(homeRegionCluster, navigableRegion.getHomeRegionCluster(), epsilon);
         for (int i = 0; i < numberOfObstacleClusters; i++)
            VisibilityGraphTestTools.assertClustersEqual(obstacleClusters.get(i), navigableRegion.getObstacleClusters().get(i), epsilon);

         navigableRegion = new NavigableRegion(planarRegion);
         navigableRegion.setVisibilityMapInWorld(visibilityMapHolder.getVisibilityMapInWorld());
         navigableRegion.setHomeRegionCluster(homeRegionCluster);
         for (int i = 0; i < numberOfObstacleClusters; i++)
            navigableRegion.addObstacleCluster(obstacleClusters.get(i));

         PlanarRegionTestTools.assertPlanarRegionsEqual(planarRegion, navigableRegion.getHomePlanarRegion(), epsilon);
         VisibilityGraphTestTools.assertVisibilityMapsEqual(navigableRegion.getVisibilityMapInLocal(), navigableRegion.getVisibilityMapInLocal(), epsilon);
         VisibilityGraphTestTools.assertVisibilityMapsEqual(navigableRegion.getVisibilityMapInWorld(), navigableRegion.getVisibilityMapInWorld(), epsilon);
         VisibilityGraphTestTools.assertClustersEqual(homeRegionCluster, navigableRegion.getHomeRegionCluster(), epsilon);
         for (int i = 0; i < numberOfObstacleClusters; i++)
            VisibilityGraphTestTools.assertClustersEqual(obstacleClusters.get(i), navigableRegion.getObstacleClusters().get(i), epsilon);
      }

   }

}
