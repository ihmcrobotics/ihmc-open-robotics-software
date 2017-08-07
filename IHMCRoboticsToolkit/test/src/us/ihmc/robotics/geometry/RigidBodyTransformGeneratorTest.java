package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.Axis;

public class RigidBodyTransformGeneratorTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testSimpleTransformGeneration()
   {
      RigidBodyTransformGenerator generator = new RigidBodyTransformGenerator();
      RigidBodyTransform identity = new RigidBodyTransform();

      RigidBodyTransform transform = generator.getRigidBodyTransformCopy();
      assertTrue(transform.epsilonEquals(identity, 1e-10));

      generator.translate(1.0, 2.0, 3.0);
      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      expectedTransform.setTranslation(1.0, 2.0, 3.0);
      generator.getRigidyBodyTransform(transform);
      assertTrue(transform.epsilonEquals(expectedTransform, 1e-10));

      generator.translate(4.0, 5.0, 6.0);
      expectedTransform = new RigidBodyTransform();
      expectedTransform.setTranslation(5.0, 7.0, 9.0);
      generator.getRigidyBodyTransform(transform);
      assertTrue(transform.epsilonEquals(expectedTransform, 1e-10));

      generator.identity();
      generator.translate(new Vector3D(-3.0, 9.0, 11.0));
      expectedTransform = new RigidBodyTransform();
      expectedTransform.setTranslation(-3.0, 9.0, 11.0);
      generator.getRigidyBodyTransform(transform);
      assertTrue(transform.epsilonEquals(expectedTransform, 1e-10));

      generator.identity();
      generator.rotate(1.7, Axis.Z);
      expectedTransform = new RigidBodyTransform();
      expectedTransform.setRotationEulerAndZeroTranslation(0.0, 0.0, 1.7);
      generator.getRigidyBodyTransform(transform);
      assertTrue(transform.epsilonEquals(expectedTransform, 1e-10));

      generator.rotate(1.3, Axis.Y);
      expectedTransform = new RigidBodyTransform();
      expectedTransform.setRotationEulerAndZeroTranslation(0.0, 1.3, 1.7);
      generator.getRigidyBodyTransform(transform);
      assertTrue(transform.epsilonEquals(expectedTransform, 1e-10));

      generator.rotate(0.2, Axis.X);
      expectedTransform = new RigidBodyTransform();
      expectedTransform.setRotationEulerAndZeroTranslation(0.2, 1.3, 1.7);
      generator.getRigidyBodyTransform(transform);
      assertTrue(transform.epsilonEquals(expectedTransform, 1e-10));

      generator.identity();
      generator.rotateEuler(0.11, 0.12, 0.13);
      expectedTransform = new RigidBodyTransform();
      expectedTransform.setRotationEulerAndZeroTranslation(0.11, 0.12, 0.13);
      generator.getRigidyBodyTransform(transform);
      assertTrue(transform.epsilonEquals(expectedTransform, 1e-10));

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testTranslateThenRotateTransformGeneration()
   {
      RigidBodyTransformGenerator generator = new RigidBodyTransformGenerator();

      generator.translate(3.3, 4.4, 5.5);
      generator.rotateEuler(new Vector3D(0.67, 0.89, 0.34));
      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      expectedTransform.setRotationEulerAndZeroTranslation(0.67, 0.89, 0.34);
      expectedTransform.setTranslation(3.3, 4.4, 5.5);
      RigidBodyTransform transform = generator.getRigidBodyTransformCopy();
      assertTrue(transform.epsilonEquals(expectedTransform, 1e-10));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testMultiStepTransformGeneration()
   {
      RigidBodyTransformGenerator generator = new RigidBodyTransformGenerator();

      generator.translate(1.0, 0.0, 0.0);
      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      expectedTransform.setTranslation(1.0, 0.0, 0.0);
      RigidBodyTransform transform = generator.getRigidBodyTransformCopy();
      assertTrue(transform.epsilonEquals(expectedTransform, 1e-10));

      generator.rotate(-Math.PI / 2.0, Axis.Y);
      generator.translate(2.0, 0.0, 0.0);
      expectedTransform = new RigidBodyTransform();
      expectedTransform.setRotationEulerAndZeroTranslation(0.0, -Math.PI / 2.0, 0.0);
      expectedTransform.setTranslation(1.0, 0.0, 2.0);
      transform = generator.getRigidBodyTransformCopy();
      assertTrue(transform.epsilonEquals(expectedTransform, 1e-10));

      generator.rotate(Math.PI / 2.0, Axis.Z);
      generator.translate(3.0, 0.0, 0.0);
      expectedTransform = new RigidBodyTransform();
      expectedTransform.setRotationEulerAndZeroTranslation(-Math.PI / 2.0, 0.0, Math.PI / 2.0);
      expectedTransform.setTranslation(1.0, 3.0, 2.0);
      transform = generator.getRigidBodyTransformCopy();
      assertTrue(transform.epsilonEquals(expectedTransform, 1e-10));

      generator.identity();
      generator.translateThenRotateEuler(new Vector3D(1.0, 0.0, 0.0), new Vector3D(0.0, -Math.PI / 2.0, 0.0));

      RigidBodyTransform translateThenRotate = new RigidBodyTransform();
      translateThenRotate.setRotationEuler(0.0, -0.0, Math.PI / 2.0);
      translateThenRotate.setTranslation(2.0, 0.0, 0.0);
      generator.translateThenRotate(translateThenRotate);

      generator.translateThenRotateEuler(new Vector3D(3.0, 0.0, 0.0), new Vector3D(0.0, 0.0, 0.0));
      transform = generator.getRigidBodyTransformCopy();
      assertTrue(transform.epsilonEquals(expectedTransform, 1e-10));

   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(RigidBodyTransformGenerator.class, RigidBodyTransformGeneratorTest.class);
   }
}
