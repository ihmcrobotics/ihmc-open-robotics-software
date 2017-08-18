package us.ihmc.footstepPlanning.flatGroundPlanning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerNode;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerNodeUtils;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.robotSide.RobotSide;

public class BipedalFootstepPlannerNodeTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
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
         transformA.setRotationEulerAndZeroTranslation(RandomGeometry.nextVector3D(random));
         transformA.setTranslation(RandomGeometry.nextVector3D(random));

         nodeA = new BipedalFootstepPlannerNode(robotSide, transformA);
         nodeB = new BipedalFootstepPlannerNode(robotSide, transformA);

         assertTrue(nodeA.equals(nodeB));
         assertTrue(nodeA.hashCode() == nodeB.hashCode());
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
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
         transform.setRotationEulerAndZeroTranslation(RandomGeometry.nextVector3D(random));
         transform.setTranslation(RandomGeometry.nextVector3D(random));

         nodeA = new BipedalFootstepPlannerNode(robotSide, transform);
         nodeB = new BipedalFootstepPlannerNode(nodeA);

         BipedalFootstepPlannerNodeUtils.removePitchAndRoll(nodeB);

         RigidBodyTransform transformA = new RigidBodyTransform();
         RigidBodyTransform transformB = new RigidBodyTransform();
         nodeA.getSoleTransform(transformA);
         nodeB.getSoleTransform(transformB);

         Vector3D xAxisAInWorld = new Vector3D(1.0, 0.0, 0.0);
         Vector3D xAxisBInWorld = new Vector3D(1.0, 0.0, 0.0);

         transformA.transform(xAxisAInWorld);
         transformB.transform(xAxisBInWorld);

         assertEquals(1.0, xAxisAInWorld.length(), 1e-7);
         assertEquals(1.0, xAxisBInWorld.length(), 1e-7);

         assertEquals(0.0, xAxisBInWorld.getZ(), 1e-7);
         xAxisAInWorld.setZ(0.0);
         xAxisAInWorld.normalize();

         assertEquals(1.0, xAxisAInWorld.dot(xAxisBInWorld), 1e-7);

         assertTrue(nodeA.epsilonEquals(nodeB, 1e-7));
         assertTrue(nodeA.hashCode() == nodeB.hashCode());
      }

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testShiftInSoleFrame()
   {
      RigidBodyTransform soleTransform = new RigidBodyTransform();
      BipedalFootstepPlannerNode node = new BipedalFootstepPlannerNode(RobotSide.LEFT, soleTransform);
      
      Vector2D shiftVector = new Vector2D(1.0, 2.0);
      BipedalFootstepPlannerNodeUtils.shiftInSoleFrame(shiftVector, node);

      Point3D solePosition = BipedalFootstepPlannerNodeUtils.getSolePosition(node);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(1.0, 2.0, 0.0), solePosition, 1e-7);
      
      soleTransform = new RigidBodyTransform();
      soleTransform.setRotationEulerAndZeroTranslation(0.0, 0.0, Math.PI/2.0);
      node = new BipedalFootstepPlannerNode(RobotSide.LEFT, soleTransform);
      
      shiftVector = new Vector2D(1.0, 2.0);
      BipedalFootstepPlannerNodeUtils.shiftInSoleFrame(shiftVector, node);

      solePosition = BipedalFootstepPlannerNodeUtils.getSolePosition(node);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(-2.0, 1.0, 0.0), solePosition, 1e-7);
   }
}
