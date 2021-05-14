package us.ihmc.footstepPlanning.graphSearch.graph;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;

public class LatticePointTest
{
   @Test
   public void testEqualsAndHashMethodsWithRandomTransforms()
   {
      Random random = new Random(3823L);
      int numTrials = 100;
      LatticePoint nodeA, nodeB;

      for (int i = 0; i < numTrials; i++)
      {
         // test for exact same transform
         double x = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double y = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double yaw = EuclidCoreRandomTools.nextDouble(random, 1.0);

         nodeA = new LatticePoint(x, y, yaw);
         nodeB = new LatticePoint(x, y, yaw);

         assertTrue(nodeA.equals(nodeB));
         assertTrue(nodeA.hashCode() == nodeB.hashCode());
      }
   }

   @Test
   public void testPiToPiRollOver()
   {
      LatticePoint nodeA = new LatticePoint(0.0, 0.0, Math.PI);
      LatticePoint nodeB = new LatticePoint(0.0, 0.0, -Math.PI);

      assertTrue(nodeA.equals(nodeB));
      assertTrue(nodeA.hashCode() == nodeB.hashCode());

      nodeA = new LatticePoint(0.0, 0.0, Math.PI - 1e-5);
      nodeB = new LatticePoint(0.0, 0.0, -Math.PI + 1e-5);

      assertTrue(nodeA.equals(nodeB));
      assertTrue(nodeA.hashCode() == nodeB.hashCode());

      nodeA = new LatticePoint(0.0, 0.0, Math.PI + 1e-5);
      nodeB = new LatticePoint(0.0, 0.0, -Math.PI - 1e-5);

      assertTrue(nodeA.equals(nodeB));
      assertTrue(nodeA.hashCode() == nodeB.hashCode());
   }
}
