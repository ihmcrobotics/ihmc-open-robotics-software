package us.ihmc.footstepPlanning.flatGroundPlanning;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;

public class LatticeNodeTest
{
   @Test
   public void testEqualsAndHashMethodsWithRandomTransforms()
   {
      Random random = new Random(3823L);
      int numTrials = 100;
      LatticeNode nodeA, nodeB;

      for (int i = 0; i < numTrials; i++)
      {
         // test for exact same transform
         double x = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double y = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double yaw = EuclidCoreRandomTools.nextDouble(random, 1.0);

         nodeA = new LatticeNode(x, y, yaw);
         nodeB = new LatticeNode(x, y, yaw);

         assertTrue(nodeA.equals(nodeB));
         assertTrue(nodeA.hashCode() == nodeB.hashCode());
      }
   }

}
