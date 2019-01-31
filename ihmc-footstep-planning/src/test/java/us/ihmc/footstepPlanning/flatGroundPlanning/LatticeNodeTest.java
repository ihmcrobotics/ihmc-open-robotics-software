package us.ihmc.footstepPlanning.flatGroundPlanning;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Random;

import static org.junit.Assert.assertTrue;

public class LatticeNodeTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
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
