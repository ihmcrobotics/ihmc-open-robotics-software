package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph;

import org.junit.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.robotics.robotSide.RobotSide;

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
         Point2DReadOnly frontLeft = EuclidCoreRandomTools.nextPoint2D(random, 1.0);
         Point2DReadOnly frontRight= EuclidCoreRandomTools.nextPoint2D(random, 1.0);
         Point2DReadOnly hindLeft = EuclidCoreRandomTools.nextPoint2D(random, 1.0);
         Point2DReadOnly hindRight = EuclidCoreRandomTools.nextPoint2D(random, 1.0);

         nodeA = new FootstepNode(frontLeft, frontRight, hindLeft, hindRight);
         nodeB = new FootstepNode(frontLeft, frontRight, hindLeft, hindRight);

         assertTrue(nodeA.equals(nodeB));
         assertTrue(nodeA.hashCode() == nodeB.hashCode());
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testShiftInSoleFrame()
   {
      Vector3D soleTranslation = new Vector3D();
      double yaw = 0.0;
      RigidBodyTransform soleTransform = new RigidBodyTransform(new AxisAngle(0.0, 0.0, 1.0, yaw), soleTranslation);
      FootstepNode node = new FootstepNode(soleTranslation.getX(), soleTranslation.getY(), yaw, RobotSide.LEFT);

      Vector2D shiftVector = new Vector2D(1.0, 2.0);
      RigidBodyTransform shiftedSoleTransform = FootstepNodeTools.shiftInSoleFrame(shiftVector, soleTransform);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(1.0, 2.0, 0.0), shiftedSoleTransform.getTranslationVector(), 1e-7);
      assertTrue(MathTools.epsilonEquals(node.getYaw(), yaw, 1e-7));

      soleTranslation = new Vector3D();
      yaw = Math.PI/2.0;
      soleTransform = new RigidBodyTransform(new AxisAngle(0.0, 0.0, 1.0, yaw), soleTranslation);
      soleTransform.setRotationEulerAndZeroTranslation(0.0, 0.0, Math.PI/2.0);
      node = new FootstepNode(soleTranslation.getX(), soleTranslation.getY(), yaw, RobotSide.LEFT);

      shiftVector = new Vector2D(1.0, 2.0);
      shiftedSoleTransform = FootstepNodeTools.shiftInSoleFrame(shiftVector, soleTransform);

      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(-2.0, 1.0, 0.0), shiftedSoleTransform.getTranslationVector(), 1e-7);
      assertTrue(MathTools.epsilonEquals(node.getYaw(), yaw, 1e-7));
   }
}
