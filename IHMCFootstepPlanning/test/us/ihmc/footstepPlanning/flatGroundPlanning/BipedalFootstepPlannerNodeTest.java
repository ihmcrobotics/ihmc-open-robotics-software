package us.ihmc.footstepPlanning.flatGroundPlanning;

import org.junit.Test;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerNode;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations;

import java.util.HashMap;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.assertTrue;

public class BipedalFootstepPlannerNodeTest
{
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testEqualsAndHashMethodsWithHardCodedTransforms()
   {
      BipedalFootstepPlannerNode nodeA, nodeB;

      RigidBodyTransform transformA = new RigidBodyTransform();
      RigidBodyTransform transformB = new RigidBodyTransform();

      transformA.setTranslation(1.799, 0.499, 5.35);
      transformB.setTranslation(1.801, 0.501, -4.43);

      nodeA = new BipedalFootstepPlannerNode(RobotSide.LEFT, transformA);
      nodeB = new BipedalFootstepPlannerNode(RobotSide.LEFT, transformB);

      assertTrue(nodeA.equals(nodeB));
      assertTrue(nodeA.hashCode() == nodeB.hashCode());

      transformA.setRotationEulerAndZeroTranslation(1.5001, -2.0001, 399.3);
      transformB.setRotationEulerAndZeroTranslation(1.4999, -1.9999, -30.2);

      nodeA = new BipedalFootstepPlannerNode(RobotSide.RIGHT, transformA);
      nodeB = new BipedalFootstepPlannerNode(RobotSide.RIGHT, transformB);

      assertTrue(nodeA.equals(nodeB));
      assertTrue(nodeA.hashCode() == nodeB.hashCode());

      double positionThreshold = BipedalFootstepPlannerNode.getXyDistanceThresholdToConsiderNodesEqual();
      double rotationThreshold = BipedalFootstepPlannerNode.getYawRotationThresholdToConsiderNodesEqual();

      transformA.setTranslation(1.496, -2.52, 499.2);
      transformB.setTranslation(1.496 + 1.01 * positionThreshold, -2.52 + 1.01 * positionThreshold, -30.3);

      nodeA = new BipedalFootstepPlannerNode(RobotSide.RIGHT, transformA);
      nodeB = new BipedalFootstepPlannerNode(RobotSide.RIGHT, transformB);

      assertTrue(! nodeA.equals(nodeB));
      assertTrue(nodeA.hashCode() != nodeB.hashCode());
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testEqualsAndHashMethodsWithRandomTransforms()
   {
      Random random = new Random(3823L);
      int numTrials = 100;
      BipedalFootstepPlannerNode nodeA, nodeB;

      RigidBodyTransform transformA = new RigidBodyTransform();

      for(int i = 0; i < numTrials; i++)
      {
         RobotSide robotSide = RobotSide.generateRandomRobotSide(random);

         // test for exact same transform
         transformA.setRotationEulerAndZeroTranslation(RandomTools.generateRandomVector(random));
         transformA.setTranslation(RandomTools.generateRandomVector(random));

         nodeA = new BipedalFootstepPlannerNode(robotSide, transformA);
         nodeB = new BipedalFootstepPlannerNode(robotSide, transformA);

         assertTrue(nodeA.equals(nodeB));
         assertTrue(nodeA.hashCode() == nodeB.hashCode());
      }
   }
}
