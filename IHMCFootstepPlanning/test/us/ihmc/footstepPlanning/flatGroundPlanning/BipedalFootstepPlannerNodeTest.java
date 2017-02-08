package us.ihmc.footstepPlanning.flatGroundPlanning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerNode;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.testing.JUnitTools;

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

      transformA.setRotationEulerAndZeroTranslation(1.5001, -2.0001, -30.2);
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

      assertTrue(!nodeA.equals(nodeB));
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

      for (int i = 0; i < numTrials; i++)
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

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testRemovePitchAndRoll()
   {
      Random random = new Random(3823L);
      int numTrials = 100;
      BipedalFootstepPlannerNode nodeA, nodeB;

      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < numTrials; i++)
      {
         RobotSide robotSide = RobotSide.generateRandomRobotSide(random);

         // test for exact same transform
         transform.setRotationEulerAndZeroTranslation(RandomTools.generateRandomVector(random));
         transform.setTranslation(RandomTools.generateRandomVector(random));

         nodeA = new BipedalFootstepPlannerNode(robotSide, transform);
         nodeB = new BipedalFootstepPlannerNode(nodeA);

         nodeB.removePitchAndRoll();

         RigidBodyTransform transformA = new RigidBodyTransform();
         RigidBodyTransform transformB = new RigidBodyTransform();
         nodeA.getSoleTransform(transformA);
         nodeB.getSoleTransform(transformB);

         Vector3d xAxisAInWorld = new Vector3d(1.0, 0.0, 0.0);
         Vector3d xAxisBInWorld = new Vector3d(1.0, 0.0, 0.0);

         transformA.transform(xAxisAInWorld);
         transformB.transform(xAxisBInWorld);

         assertEquals(1.0, xAxisAInWorld.length(), 1e-7);
         assertEquals(1.0, xAxisBInWorld.length(), 1e-7);

         assertEquals(0.0, xAxisBInWorld.getZ(), 1e-7);
         xAxisAInWorld.setZ(0.0);
         xAxisAInWorld.normalize();

         assertEquals(1.0, xAxisAInWorld.dot(xAxisBInWorld), 1e-7);

         assertTrue(nodeA.equals(nodeB));
         assertTrue(nodeA.hashCode() == nodeB.hashCode());
      }

   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testShiftInSoleFrame()
   {
      RigidBodyTransform soleTransform = new RigidBodyTransform();
      BipedalFootstepPlannerNode node = new BipedalFootstepPlannerNode(RobotSide.LEFT, soleTransform);
      
      Vector2d shiftVector = new Vector2d(1.0, 2.0);
      node.shiftInSoleFrame(shiftVector);

      Point3d solePosition = node.getSolePosition();
      JUnitTools.assertTuple3dEquals(new Point3d(1.0, 2.0, 0.0), solePosition, 1e-7);
      
      soleTransform = new RigidBodyTransform();
      soleTransform.setRotationEulerAndZeroTranslation(0.0, 0.0, Math.PI/2.0);
      node = new BipedalFootstepPlannerNode(RobotSide.LEFT, soleTransform);
      
      shiftVector = new Vector2d(1.0, 2.0);
      node.shiftInSoleFrame(shiftVector);

      solePosition = node.getSolePosition();
      JUnitTools.assertTuple3dEquals(new Point3d(-2.0, 1.0, 0.0), solePosition, 1e-7);
   }
}
