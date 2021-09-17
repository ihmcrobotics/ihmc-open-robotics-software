package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ClusterType;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ExtrusionSide;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

public class VisibilityGraphNodeTest
{
   private static final int iters = 1000;

   @Test
   public void testCaching()
   {
      HashSet<VisibilityGraphNode> visibilityGraphNodes = new HashSet<>();

      Random random = new Random(1738L);
      for (int i = 0; i < iters; i++)
      {
         Point3D pointInWorld = EuclidCoreRandomTools.nextPoint3D(random, 100.0);
         NavigableRegion navigableRegion = new NavigableRegion(new PlanarRegion(), new Cluster(ExtrusionSide.INSIDE, ClusterType.POLYGON), new ArrayList<>());
         VisibilityGraphNavigableRegion visibilityGraphNavigableRegion = new VisibilityGraphNavigableRegion(navigableRegion, false);
         VisibilityGraphNode nodeA = new VisibilityGraphNode(pointInWorld, new Point2D(pointInWorld), visibilityGraphNavigableRegion);
         VisibilityGraphNode nodeB = new VisibilityGraphNode(pointInWorld, new Point2D(pointInWorld), visibilityGraphNavigableRegion);

         assertFalse(visibilityGraphNodes.contains(nodeB));
         visibilityGraphNodes.add(nodeA);
         assertTrue(visibilityGraphNodes.contains(nodeB));
      }

   }
}
