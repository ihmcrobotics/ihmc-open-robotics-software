package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph;

import gnu.trove.list.array.TIntArrayList;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.FootstepPlanningRandomTools;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

public class PawNodeTest
{
   @Test
   public void testEqualsAndHashMethodsWithRandomTransforms()
   {
      Random random = new Random(3823L);
      int numTrials = 100;
      PawNode nodeA, nodeB;

      for (int i = 0; i < numTrials; i++)
      {
         // test for exact same transform
         nodeA = FootstepPlanningRandomTools.createRandomFootstepNode(random);
         nodeB = new PawNode(nodeA);

         assertTrue(nodeA.equals(nodeB));
         assertTrue(nodeA.hashCode() == nodeB.hashCode());
      }
   }

   @Test
   public void testQuadrantEquals()
   {
      Random random = new Random(3823L);
      int numTrials = 100;
      PawNode nodeA, nodeB;

      for (int i = 0; i < numTrials; i++)
      {
         // test for exact same transform
         RobotQuadrant robotQuadrant = RobotQuadrant.FRONT_LEFT;
         Point2DReadOnly frontLeft = EuclidCoreRandomTools.nextPoint2D(random, 1.0);
         Point2DReadOnly frontRight= EuclidCoreRandomTools.nextPoint2D(random, 1.0);
         Point2DReadOnly otherFrontRight= EuclidCoreRandomTools.nextPoint2D(random, 1.0);
         Point2DReadOnly hindLeft = EuclidCoreRandomTools.nextPoint2D(random, 1.0);
         Point2DReadOnly otherHindLeft = EuclidCoreRandomTools.nextPoint2D(random, 1.0);
         Point2DReadOnly hindRight = EuclidCoreRandomTools.nextPoint2D(random, 1.0);
         Point2DReadOnly otherHindRight = EuclidCoreRandomTools.nextPoint2D(random, 1.0);


         double yaw = PawNode.computeNominalYaw(frontLeft.getX(), frontLeft.getY(), frontRight.getX(), frontRight.getY(), hindLeft.getX(), hindLeft.getY(),
                                                hindRight.getX(), hindRight.getY());

         nodeA = new PawNode(robotQuadrant, frontLeft, frontRight, hindLeft, hindRight, yaw, 1.5, 0.5);
         nodeB = new PawNode(robotQuadrant, frontLeft, otherFrontRight, otherHindLeft, otherHindRight, yaw, 1.5, 0.5);

         assertTrue("number : " + i, nodeA.quadrantGeometricallyEquals(nodeB));

         TIntArrayList expandedNodes = new TIntArrayList();

         assertFalse("number : " + i, expandedNodes.contains(nodeB.hashCode()));

         expandedNodes.add(nodeA.hashCode());

         if (nodeA.getYawIndex() == nodeB.getYawIndex())
         {
            assertEquals("number : " + i, nodeA.hashCode(), nodeB.hashCode());
            assertTrue("number : " + i, expandedNodes.contains(nodeB.hashCode()));
         }
         else
         {
            assertFalse("number : " + i, nodeA.hashCode() == nodeB.hashCode());
            assertFalse("number : " + i, expandedNodes.contains(nodeB.hashCode()));

         }
      }
   }
}
