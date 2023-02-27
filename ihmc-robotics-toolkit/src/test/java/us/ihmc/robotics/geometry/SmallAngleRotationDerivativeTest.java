package us.ihmc.robotics.geometry;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.random.RandomGeometry;

public class SmallAngleRotationDerivativeTest
{

	@Test
   public void test()
   {
      Random random = new Random(125123412L);
      AxisAngle nominalAxisAngle = RandomGeometry.nextAxisAngle(random);
      Quaternion nominalQuaternion = new Quaternion();
      nominalQuaternion.set(nominalAxisAngle);
      RotationMatrix nominalRotation = new RotationMatrix();
      nominalRotation.set(nominalQuaternion);

      Vector3D vector = RandomGeometry.nextVector3D(random);
      Vector3D transformedVector = new Vector3D(vector);
      nominalRotation.transform(transformedVector);

      Matrix3D jacobian = new Matrix3D();
      jacobian.setToTildeForm(vector);
      jacobian.scale(-1.0);
      jacobian.preMultiply(nominalRotation);

      double perturbationMagnitude = 1e-6;
      double delta = 1e-12;
      for (Axis3D axis : Axis3D.values())
      {
         Vector3D perturbationRotationVector = new Vector3D();
         perturbationRotationVector.setElement(axis, perturbationMagnitude);

         // expected
         Vector3D expected = new Vector3D(perturbationRotationVector);
         jacobian.transform(expected);

         // actual
         AxisAngle perturbationAxisAngle = new AxisAngle(perturbationRotationVector);
         Quaternion perturbationQuaternion = new Quaternion();
         perturbationQuaternion.set(perturbationAxisAngle);
         Quaternion perturbedQuaternion = new Quaternion();
         perturbedQuaternion.multiply(nominalQuaternion, perturbationQuaternion);

         RotationMatrix perturbedMatrix = new RotationMatrix();
         perturbedMatrix.set(perturbedQuaternion);
         Vector3D actual = new Vector3D(vector);
         perturbedMatrix.transform(actual);
         actual.sub(transformedVector);

         // compare
         EuclidCoreTestTools.assertEquals(expected, actual, delta);
      }
   }
}
