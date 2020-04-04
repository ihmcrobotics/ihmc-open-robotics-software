package us.ihmc.robotics.geometry;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.MutationTestFacilitator;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;

public class RigidBodyTransformGeneratorTest
{
   @Test
   public void testSimpleTransformGeneration()
   {
      RigidBodyTransformGenerator generator = new RigidBodyTransformGenerator();
      RigidBodyTransform identity = new RigidBodyTransform();

      RigidBodyTransform transform = generator.getRigidBodyTransformCopy();
      assertTrue(transform.epsilonEquals(identity, 1e-10));

      generator.translate(1.0, 2.0, 3.0);
      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      expectedTransform.getTranslation().set(1.0, 2.0, 3.0);
      generator.getRigidyBodyTransform(transform);
      assertTrue(transform.epsilonEquals(expectedTransform, 1e-10));

      generator.translate(4.0, 5.0, 6.0);
      expectedTransform = new RigidBodyTransform();
      expectedTransform.getTranslation().set(5.0, 7.0, 9.0);
      generator.getRigidyBodyTransform(transform);
      assertTrue(transform.epsilonEquals(expectedTransform, 1e-10));

      generator.identity();
      generator.translate(new Vector3D(-3.0, 9.0, 11.0));
      expectedTransform = new RigidBodyTransform();
      expectedTransform.getTranslation().set(-3.0, 9.0, 11.0);
      generator.getRigidyBodyTransform(transform);
      assertTrue(transform.epsilonEquals(expectedTransform, 1e-10));

      generator.identity();
      generator.rotate(1.7, Axis3D.Z);
      expectedTransform = new RigidBodyTransform();
      expectedTransform.setRotationEulerAndZeroTranslation(0.0, 0.0, 1.7);
      generator.getRigidyBodyTransform(transform);
      assertTrue(transform.epsilonEquals(expectedTransform, 1e-10));

      generator.rotate(1.3, Axis3D.Y);
      expectedTransform = new RigidBodyTransform();
      expectedTransform.setRotationEulerAndZeroTranslation(0.0, 1.3, 1.7);
      generator.getRigidyBodyTransform(transform);
      assertTrue(transform.epsilonEquals(expectedTransform, 1e-10));

      generator.rotate(0.2, Axis3D.X);
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

   @Test
   public void testTranslateThenRotateTransformGeneration()
   {
      RigidBodyTransformGenerator generator = new RigidBodyTransformGenerator();

      generator.translate(3.3, 4.4, 5.5);
      generator.rotateEuler(new Vector3D(0.67, 0.89, 0.34));
      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      expectedTransform.setRotationEulerAndZeroTranslation(0.67, 0.89, 0.34);
      expectedTransform.getTranslation().set(3.3, 4.4, 5.5);
      RigidBodyTransform transform = generator.getRigidBodyTransformCopy();
      assertTrue(transform.epsilonEquals(expectedTransform, 1e-10));
   }

   @Test
   public void testMultiStepTransformGeneration()
   {
      RigidBodyTransformGenerator generator = new RigidBodyTransformGenerator();

      generator.translate(1.0, 0.0, 0.0);
      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      expectedTransform.getTranslation().set(1.0, 0.0, 0.0);
      RigidBodyTransform transform = generator.getRigidBodyTransformCopy();
      assertTrue(transform.epsilonEquals(expectedTransform, 1e-10));

      generator.rotate(-Math.PI / 2.0, Axis3D.Y);
      generator.translate(2.0, 0.0, 0.0);
      expectedTransform = new RigidBodyTransform();
      expectedTransform.setRotationEulerAndZeroTranslation(0.0, -Math.PI / 2.0, 0.0);
      expectedTransform.getTranslation().set(1.0, 0.0, 2.0);
      transform = generator.getRigidBodyTransformCopy();
      assertTrue(transform.epsilonEquals(expectedTransform, 1e-10));

      generator.rotate(Math.PI / 2.0, Axis3D.Z);
      generator.translate(3.0, 0.0, 0.0);
      expectedTransform = new RigidBodyTransform();
      expectedTransform.setRotationEulerAndZeroTranslation(-Math.PI / 2.0, 0.0, Math.PI / 2.0);
      expectedTransform.getTranslation().set(1.0, 3.0, 2.0);
      transform = generator.getRigidBodyTransformCopy();
      assertTrue(transform.epsilonEquals(expectedTransform, 1e-10));

      generator.identity();
      generator.translateThenRotateEuler(new Vector3D(1.0, 0.0, 0.0), new Vector3D(0.0, -Math.PI / 2.0, 0.0));

      RigidBodyTransform translateThenRotate = new RigidBodyTransform();
      translateThenRotate.getRotation().setEuler(0.0, -0.0, Math.PI / 2.0);
      translateThenRotate.getTranslation().set(2.0, 0.0, 0.0);
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
