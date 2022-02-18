package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

public class VisibilityGraphEdgeTest
{
   private static final int iters = 1000;

   @Test
   public void testCaching()
   {
      HashSet<VisibilityGraphEdge> visibilityGraphEdges = new HashSet<>();

      Random random = new Random(1738L);
      for (int i = 0; i < iters; i++)
      {
         Point3D pointInWorld1 = EuclidCoreRandomTools.nextPoint3D(random, 100.0);
         Point3D pointInWorld2 = EuclidCoreRandomTools.nextPoint3D(random, 100.0);
         NavigableRegion navigableRegion = new NavigableRegion(new PlanarRegion(), new Cluster(Cluster.ExtrusionSide.INSIDE, Cluster.ClusterType.POLYGON), new ArrayList<>());
         VisibilityGraphNavigableRegion visibilityGraphNavigableRegion = new VisibilityGraphNavigableRegion(navigableRegion, false);
         VisibilityGraphNode nodeA = new VisibilityGraphNode(pointInWorld1, new Point2D(pointInWorld1), visibilityGraphNavigableRegion);
         VisibilityGraphNode nodeB = new VisibilityGraphNode(pointInWorld1, new Point2D(pointInWorld1), visibilityGraphNavigableRegion);
         VisibilityGraphNode nodeC = new VisibilityGraphNode(pointInWorld2, new Point2D(pointInWorld2), visibilityGraphNavigableRegion);
         VisibilityGraphNode nodeD = new VisibilityGraphNode(pointInWorld2, new Point2D(pointInWorld2), visibilityGraphNavigableRegion);

         VisibilityGraphEdge edge1 = new VisibilityGraphEdge(nodeA, nodeC);
         VisibilityGraphEdge edge2 = new VisibilityGraphEdge(nodeA, nodeD);
         VisibilityGraphEdge edge3 = new VisibilityGraphEdge(nodeB, nodeC);
         VisibilityGraphEdge edge4 = new VisibilityGraphEdge(nodeB, nodeD);

         assertFalse(visibilityGraphEdges.contains(edge1));
         visibilityGraphEdges.add(edge1);
         assertTrue(visibilityGraphEdges.contains(edge2));
         assertTrue(visibilityGraphEdges.contains(edge3));
         assertTrue(visibilityGraphEdges.contains(edge4));
      }
   }
}
