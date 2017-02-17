package us.ihmc.robotics.geometry.shapes;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.robotics.math.Epsilons;
import us.ihmc.tools.testing.MutationTestingTools;

public class TwoWayRigidBodyTransformTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testSomeBasicTransformations() throws Exception
   {
      TwoWayRigidBodyTransform twoWayRigidBodyTransform = new TwoWayRigidBodyTransform();
      
      Random random = new Random(832498235L);
      
      Point3D translation = new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      RotationMatrix rotation = EuclidCoreRandomTools.generateRandomRotationMatrix(random);
      
      RigidBodyTransform forwardTransformUnsafe = twoWayRigidBodyTransform.getForwardTransformUnsafe();
      forwardTransformUnsafe.setTranslation(translation);
      forwardTransformUnsafe.setRotation(rotation);
      twoWayRigidBodyTransform.setBackwardTransform(forwardTransformUnsafe);
      
      Point3D originalPosition = new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Vector4D originalOrientation = new Vector4D(random.nextDouble(), random.nextDouble(), random.nextDouble(), 1.0);
      
      Point3D resultPosition = new Point3D(originalPosition);
      Vector4D resultOrientation = new Vector4D(originalOrientation);
      
      twoWayRigidBodyTransform.transformForward(resultPosition);
      twoWayRigidBodyTransform.transformForward(resultOrientation);
      twoWayRigidBodyTransform.transformBackward(resultPosition);
      twoWayRigidBodyTransform.transformBackward(resultOrientation);
      
      EuclidCoreTestTools.assertTuple3DEquals("not equal", originalPosition, resultPosition, Epsilons.ONE_TRILLIONTH);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSomeMoreTransformations() throws Exception
   {
      TwoWayRigidBodyTransform twoWayRigidBodyTransform = new TwoWayRigidBodyTransform();
      assertFalse(twoWayRigidBodyTransform.isForwardTransformOutOfDate());
      assertFalse(twoWayRigidBodyTransform.isBackwardTransformOutOfDate());
      
      twoWayRigidBodyTransform.setForwardTransform(new RigidBodyTransform());
      assertFalse(twoWayRigidBodyTransform.isForwardTransformOutOfDate());
      assertTrue(twoWayRigidBodyTransform.isBackwardTransformOutOfDate());
      
      twoWayRigidBodyTransform.setBackwardTransform(new RigidBodyTransform());
      assertTrue(twoWayRigidBodyTransform.isForwardTransformOutOfDate());
      assertFalse(twoWayRigidBodyTransform.isBackwardTransformOutOfDate());
      
      twoWayRigidBodyTransform.setForwardTransform(new RigidBodyTransform());
      assertFalse(twoWayRigidBodyTransform.isForwardTransformOutOfDate());
      assertTrue(twoWayRigidBodyTransform.isBackwardTransformOutOfDate());
      
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.appendRollRotation(0.9);
      
      twoWayRigidBodyTransform.setForwardTransform(transform);
      assertFalse(twoWayRigidBodyTransform.isForwardTransformOutOfDate());
      assertTrue(twoWayRigidBodyTransform.isBackwardTransformOutOfDate());
      
      RigidBodyTransform backwardTransformUnsafe = twoWayRigidBodyTransform.getBackwardTransformUnsafe();
      transform.invert();
      assertTrue(transform.epsilonEquals(backwardTransformUnsafe, 1e-10));
      
      transform = new RigidBodyTransform();
      transform.appendPitchRotation(0.9);
      twoWayRigidBodyTransform.setBackwardTransform(transform);
      assertTrue(twoWayRigidBodyTransform.isForwardTransformOutOfDate());
      assertFalse(twoWayRigidBodyTransform.isBackwardTransformOutOfDate());
      
      RigidBodyTransform forwardTransformUnsafe = twoWayRigidBodyTransform.getForwardTransformUnsafe();
      transform.invert();
      assertTrue(transform.epsilonEquals(forwardTransformUnsafe, 1e-10));
   }
   
   public static void main(String[] args)
   {
      String targetTests = TwoWayRigidBodyTransformTest.class.getName();
      String targetClassesInSamePackage = TwoWayRigidBodyTransform.class.getName();
      MutationTestingTools.doPITMutationTestAndOpenResult(targetTests, targetClassesInSamePackage);
   }
   
}
