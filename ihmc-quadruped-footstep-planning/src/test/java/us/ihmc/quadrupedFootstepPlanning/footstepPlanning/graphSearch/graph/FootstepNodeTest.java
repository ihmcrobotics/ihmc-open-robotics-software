package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.FootstepPlanningRandomTools;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class FootstepNodeTest
{
   @Test
   public void testEqualsAndHashMethodsWithRandomTransforms()
   {
      Random random = new Random(3823L);
      int numTrials = 100;
      FootstepNode nodeA, nodeB;

      for (int i = 0; i < numTrials; i++)
      {
         // test for exact same transform
         nodeA = FootstepPlanningRandomTools.createRandomFootstepNode(random);
         nodeB = new FootstepNode(nodeA);

         assertTrue(nodeA.equals(nodeB));
         assertTrue(nodeA.hashCode() == nodeB.hashCode());
      }
   }

   @Test
   public void testQuadrantEquals()
   {
      Random random = new Random(3823L);
      int numTrials = 100;
      FootstepNode nodeA, nodeB;

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

         double yaw = RandomNumbers.nextDouble(random, Math.PI);
         double otherYaw = RandomNumbers.nextDouble(random, Math.PI);

         nodeA = new FootstepNode(robotQuadrant, frontLeft, frontRight, hindLeft, hindRight, yaw, 1.5, 0.5);
         nodeB = new FootstepNode(robotQuadrant, frontLeft, otherFrontRight, otherHindLeft, otherHindRight, otherYaw, 1.5, 0.5);

         assertTrue(nodeA.quadrantGeometricallyEquals(nodeB));
      }
   }
}
