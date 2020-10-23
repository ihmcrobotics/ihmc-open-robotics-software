package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.pathPlanning.visibilityGraphs.VisibilityGraphRandomTools;
import us.ihmc.pathPlanning.visibilityGraphs.VisibilityGraphTestTools;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTestTools;

public class NavigableRegionTest
{
   private final static int iterations = 20;

   private static final double epsilon = 1e-12;

   @Test
   public void testNavigableRegions()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iterations; iter++)
      {
         PlanarRegion planarRegion = PlanarRegion
               .generatePlanarRegionFromRandomPolygonsWithRandomTransform(random, random.nextInt(10), RandomNumbers.nextDouble(random, 0.0, 30.0),
                                                                          3 + random.nextInt(10));
         VisibilityMapHolder visibilityMapHolder = VisibilityGraphRandomTools.getRandomSingleSourceVisibilityMap(random);
         Cluster homeRegionCluster = VisibilityGraphRandomTools.getRandomCluster(random);

         int numberOfObstacleClusters = random.nextInt(100);
         List<Cluster> obstacleClusters = new ArrayList<>();

         for (int i = 0; i < numberOfObstacleClusters; i++)
         {
            obstacleClusters.add(VisibilityGraphRandomTools.getRandomCluster(random));
         }

         NavigableRegion navigableRegion = new NavigableRegion(planarRegion, homeRegionCluster, obstacleClusters);
         VisibilityMapWithNavigableRegion visibilityMapWithNavigableRegion = new VisibilityMapWithNavigableRegion(navigableRegion);
         visibilityMapWithNavigableRegion.setVisibilityMapInLocal(visibilityMapHolder.getVisibilityMapInLocal());

         PlanarRegionTestTools.assertPlanarRegionsEqual(planarRegion, visibilityMapWithNavigableRegion.getHomePlanarRegion(), epsilon);
         VisibilityGraphTestTools.assertVisibilityMapsEqual(visibilityMapWithNavigableRegion.getVisibilityMapInLocal(), visibilityMapWithNavigableRegion.getVisibilityMapInLocal(), epsilon);
         VisibilityGraphTestTools.assertVisibilityMapsEqual(visibilityMapWithNavigableRegion.getVisibilityMapInWorld(), visibilityMapWithNavigableRegion.getVisibilityMapInWorld(), epsilon);
         VisibilityGraphTestTools.assertClustersEqual(homeRegionCluster, visibilityMapWithNavigableRegion.getHomeRegionCluster(), epsilon);

         for (int i = 0; i < numberOfObstacleClusters; i++)
            VisibilityGraphTestTools.assertClustersEqual(obstacleClusters.get(i), visibilityMapWithNavigableRegion.getObstacleClusters().get(i), epsilon);

         navigableRegion = new NavigableRegion(planarRegion, homeRegionCluster, obstacleClusters);
         visibilityMapWithNavigableRegion = new VisibilityMapWithNavigableRegion(navigableRegion);
         visibilityMapWithNavigableRegion.setVisibilityMapInWorld(visibilityMapHolder.getVisibilityMapInWorld());

         PlanarRegionTestTools.assertPlanarRegionsEqual(planarRegion, visibilityMapWithNavigableRegion.getHomePlanarRegion(), epsilon);
         VisibilityGraphTestTools.assertVisibilityMapsEqual(visibilityMapWithNavigableRegion.getVisibilityMapInLocal(), visibilityMapWithNavigableRegion.getVisibilityMapInLocal(), epsilon);
         VisibilityGraphTestTools.assertVisibilityMapsEqual(visibilityMapWithNavigableRegion.getVisibilityMapInWorld(), visibilityMapWithNavigableRegion.getVisibilityMapInWorld(), epsilon);
         VisibilityGraphTestTools.assertClustersEqual(homeRegionCluster, visibilityMapWithNavigableRegion.getHomeRegionCluster(), epsilon);

         for (int i = 0; i < numberOfObstacleClusters; i++)
            VisibilityGraphTestTools.assertClustersEqual(obstacleClusters.get(i), visibilityMapWithNavigableRegion.getObstacleClusters().get(i), epsilon);
      }

   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(NavigableRegion.class, NavigableRegionTest.class);
   }

}
