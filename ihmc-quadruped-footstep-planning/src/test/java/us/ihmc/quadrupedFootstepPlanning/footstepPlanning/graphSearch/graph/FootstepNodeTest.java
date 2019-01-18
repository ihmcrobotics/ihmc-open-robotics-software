package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.Random;

import static org.junit.Assert.assertTrue;

public class FootstepNodeTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testEqualsAndHashMethodsWithRandomTransforms()
   {
      Random random = new Random(3823L);
      int numTrials = 100;
      FootstepNode nodeA, nodeB;

      for (int i = 0; i < numTrials; i++)
      {
         // test for exact same transform
         RobotQuadrant robotQuadrant = RobotQuadrant.generateRandomRobotQuadrant(random);
         Point2DReadOnly frontLeft = EuclidCoreRandomTools.nextPoint2D(random, 1.0);
         Point2DReadOnly frontRight= EuclidCoreRandomTools.nextPoint2D(random, 1.0);
         Point2DReadOnly hindLeft = EuclidCoreRandomTools.nextPoint2D(random, 1.0);
         Point2DReadOnly hindRight = EuclidCoreRandomTools.nextPoint2D(random, 1.0);

         nodeA = new FootstepNode(robotQuadrant, frontLeft, frontRight, hindLeft, hindRight);
         nodeB = new FootstepNode(robotQuadrant, frontLeft, frontRight, hindLeft, hindRight);

         assertTrue(nodeA.equals(nodeB));
         assertTrue(nodeA.hashCode() == nodeB.hashCode());
      }
   }
}
