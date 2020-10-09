package us.ihmc.footstepPlanning.graphSearch.graph;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class FootstepGraphNodeTest
{
   @Test
   public void testEqualsAndHashcode()
   {
      Random random = new Random(3823L);
      int numTrials = 100;
      FootstepGraphNode nodeA, nodeB;

      for (int i = 0; i < numTrials; i++)
      {
         RobotSide stanceSide = RobotSide.generateRandomRobotSide(random);
         double xStance = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double yStance = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double yawStance = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double xSwing = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double ySwing = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double yawSwing = EuclidCoreRandomTools.nextDouble(random, 1.0);

         nodeA = new FootstepGraphNode(new FootstepNode(xStance, yStance, yawStance, stanceSide), new FootstepNode(xSwing, ySwing, yawSwing, stanceSide.getOppositeSide()));
         nodeB = new FootstepGraphNode(new FootstepNode(xStance, yStance, yawStance, stanceSide), new FootstepNode(xSwing, ySwing, yawSwing, stanceSide.getOppositeSide()));

         assertTrue(nodeA.equals(nodeB));
         assertTrue(nodeA.hashCode() == nodeB.hashCode());

         nodeB = new FootstepGraphNode(new FootstepNode(xStance, yStance, yawStance, stanceSide), new FootstepNode(xSwing + 0.5, ySwing, yawSwing, stanceSide.getOppositeSide()));
         assertFalse(nodeA.equals(nodeB));
         nodeB = new FootstepGraphNode(new FootstepNode(xStance, yStance, yawStance, stanceSide), new FootstepNode(xSwing, ySwing + 0.5, yawSwing, stanceSide.getOppositeSide()));
         assertFalse(nodeA.equals(nodeB));
         nodeB = new FootstepGraphNode(new FootstepNode(xStance, yStance, yawStance, stanceSide), new FootstepNode(xSwing, ySwing, yawSwing + 0.5, stanceSide.getOppositeSide()));
         assertFalse(nodeA.equals(nodeB));

         // assert exception thrown
         try
         {
            nodeB = new FootstepGraphNode(new FootstepNode(xStance, yStance, yawStance, stanceSide), new FootstepNode(xSwing, ySwing, yawSwing + 0.5, stanceSide));
            fail();
         }
         catch (Exception e)
         {
         }

         nodeB = new FootstepGraphNode(nodeA.getStartStart(), nodeA.getEndStep());
         assertFalse(nodeA.equals(nodeB));
      }
   }

}
