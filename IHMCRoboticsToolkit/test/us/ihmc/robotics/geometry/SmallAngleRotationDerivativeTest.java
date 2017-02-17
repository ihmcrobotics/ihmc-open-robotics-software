package us.ihmc.robotics.geometry;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.random.RandomTools;

public class SmallAngleRotationDerivativeTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void test()
   {
      Random random = new Random(125123412L);
      AxisAngle nominalAxisAngle = RandomTools.generateRandomRotation(random);
      Quaternion nominalQuaternion = new Quaternion();
      nominalQuaternion.set(nominalAxisAngle);
      RotationMatrix nominalRotation = new RotationMatrix();
      nominalRotation.set(nominalQuaternion);

      Vector3D vector = RandomTools.generateRandomVector(random);
      Vector3D transformedVector = new Vector3D(vector);
      nominalRotation.transform(transformedVector);

      Matrix3D jacobian = new Matrix3D();
      jacobian.setToTildeForm(vector);
      jacobian.scale(-1.0);
      jacobian.preMultiply(nominalRotation);

      double perturbationMagnitude = 1e-6;
      double delta = 1e-12;
      for (Direction direction : Direction.values())
      {
         Vector3D perturbationRotationVector = new Vector3D();
         MathTools.set(perturbationRotationVector, direction, perturbationMagnitude);

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
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, delta);
      }
   }
}
