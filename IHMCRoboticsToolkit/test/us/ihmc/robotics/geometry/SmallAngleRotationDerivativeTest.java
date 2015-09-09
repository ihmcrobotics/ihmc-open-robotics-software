package us.ihmc.robotics.geometry;

import org.junit.Test;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.util.Random;

public class SmallAngleRotationDerivativeTest
{

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void test()
   {
      Random random = new Random(125123412L);
      AxisAngle4d nominalAxisAngle = RandomTools.generateRandomRotation(random);
      Quat4d nominalQuaternion = new Quat4d();
      nominalQuaternion.set(nominalAxisAngle);
      Matrix3d nominalRotation = new Matrix3d();
      nominalRotation.set(nominalQuaternion);

      Vector3d vector = RandomTools.generateRandomVector(random);
      Vector3d transformedVector = new Vector3d(vector);
      nominalRotation.transform(transformedVector);

      Matrix3d jacobian = new Matrix3d();
      MatrixTools.toTildeForm(jacobian, vector);
      jacobian.mul(-1.0);
      jacobian.mul(nominalRotation, jacobian);

      double perturbationMagnitude = 1e-6;
      double delta = 1e-12;
      for (Direction direction : Direction.values())
      {
         Vector3d perturbationRotationVector = new Vector3d();
         MathTools.set(perturbationRotationVector, direction, perturbationMagnitude);

         // expected
         Vector3d expected = new Vector3d(perturbationRotationVector);
         jacobian.transform(expected);

         // actual
         AxisAngle4d perturbationAxisAngle = new AxisAngle4d();
         RotationFunctions.setAxisAngleBasedOnRotationVector(perturbationAxisAngle, perturbationRotationVector);
         Quat4d perturbationQuaternion = new Quat4d();
         perturbationQuaternion.set(perturbationAxisAngle);
         Quat4d perturbedQuaternion = new Quat4d();
         perturbedQuaternion.mul(nominalQuaternion, perturbationQuaternion);

         Matrix3d perturbedMatrix = new Matrix3d();
         perturbedMatrix.set(perturbedQuaternion);
         Vector3d actual = new Vector3d(vector);
         perturbedMatrix.transform(actual);
         actual.sub(transformedVector);

         // compare
         JUnitTools.assertTuple3dEquals(expected, actual, delta);
      }
   }
}
