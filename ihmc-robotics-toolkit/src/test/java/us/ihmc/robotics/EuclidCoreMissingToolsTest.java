package us.ihmc.robotics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class EuclidCoreMissingToolsTest
{
   @Test
   public void testRoundToGivenPrecision()
   {
      double longDouble = 0.12345678910111213;

      double roundedNumber = MathTools.floorToPrecision(longDouble, 1e-7);
      assertEquals(roundedNumber, 0.1234567, 1e-14);

      roundedNumber = MathTools.floorToPrecision(longDouble, 1e-3);
      assertEquals(roundedNumber, 0.123, 1e-14);

      Vector3D preciseVector = new Vector3D(0.12345678910111213, 100.12345678910111213, 1000.12345678910111213);
      Vector3D roundedVector = new Vector3D(preciseVector);

      EuclidCoreMissingTools.floorToGivenPrecision(roundedVector, 1e-7);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(0.1234567, 100.1234567, 1000.1234567), roundedVector, 1e-12);

      EuclidCoreMissingTools.floorToGivenPrecision(roundedVector, 1e-3);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(0.123, 100.123, 1000.123), roundedVector, 1e-14);
   }

   @Test
   public void testProjectRotationOnAxis()
   {
      Random random = new Random(9429424L);
      Quaternion fullRotation = new Quaternion();
      Quaternion actualRotation = new Quaternion();

      for (int i = 0; i < 10000; i++)
      {
         // Create random axis and a rotation around that axis.
         Vector3D axis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         double angle = EuclidCoreRandomTools.nextDouble(random, -Math.PI, Math.PI);
         Quaternion expectedRotation = new Quaternion(new AxisAngle(axis, angle));

         // Create an orthogonal rotation.
         Vector3D orthogonalAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, axis, true);
         double orthogonalAngle = EuclidCoreRandomTools.nextDouble(random, -Math.PI, Math.PI);
         Quaternion orthogonalRotation = new Quaternion(new AxisAngle(orthogonalAxis, orthogonalAngle));

         // From the combined rotation and the original axis back out the rotation around the original axis.
         fullRotation.multiply(orthogonalRotation, expectedRotation);
         EuclidCoreMissingTools.projectRotationOnAxis(fullRotation, axis, actualRotation);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expectedRotation, actualRotation, 1.0e-10);
      }
   }

   @Test
   public void testRotationMatrix3DFromFirstToSecondVector3D()
   {
      Random random = new Random(43634);

      for (int i = 0; i < 10000; i++)
      {
         Vector3D firstVector = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D secondVector = EuclidCoreRandomTools.nextVector3D(random);

         RotationMatrix actualRotationMatrix = new RotationMatrix();
         AxisAngle actualAxisAngle = new AxisAngle();
         AxisAngle expectedAxisAngle = new AxisAngle();

         EuclidCoreMissingTools.rotationMatrix3DFromFirstToSecondVector3D(firstVector, secondVector, actualRotationMatrix);
         actualAxisAngle.set(actualRotationMatrix);
         EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(firstVector, secondVector, expectedAxisAngle);
         EuclidCoreTestTools.assertAxisAngleGeometricallyEquals(expectedAxisAngle, actualAxisAngle, 1.0e-7);
      }
   }
}
