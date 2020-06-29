package us.ihmc.robotics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class EuclidCoreMissingToolsTest
{
   private static final double EPSILON = 1.0e-12;
   private static final int iters = 1000;

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

   @Test
   public void testDistanceBetweenTwoLineSegment2Ds()
   {
      Point2D closestPointOnLineSegment1 = new Point2D();
      Point2D closestPointOnLineSegment2 = new Point2D();

      Vector2D lineSegmentDirection1 = new Vector2D();
      Vector2D lineSegmentDirection2 = new Vector2D();

      Random random = new Random(11762L);

      // Parallel case, expecting expectedPointOnLineSegment1 =
      // lineSegmentStart1
      for (int i = 0; i < iters; i++)
      {
         Point2D lineSegmentStart1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // expectedPointOnLineSegment1 = lineSegmentStart1
         closestPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2
         Vector2D orthogonalToLineSegment1 = nextOrthogonalVector2D(random, lineSegmentDirection1, true);
         double expectedMinimumDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         closestPointOnLineSegment2.scaleAdd(expectedMinimumDistance, orthogonalToLineSegment1, closestPointOnLineSegment1);

         // Set the lineSegmentDirection2 = lineSegmentDirection1
         lineSegmentDirection2.set(lineSegmentDirection1);

         // Set the end points of the line segment 2 around the expected
         // closest point.
         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineSegmentDirection2, closestPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineSegmentDirection2, closestPointOnLineSegment2);

         double actualMinimumDistance = EuclidCoreMissingTools.distanceBetweenTwoLineSegment2Ds(lineSegmentStart1,
                                                                                                lineSegmentEnd1,
                                                                                                lineSegmentStart2,
                                                                                                lineSegmentEnd2);
         Assert.assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);

         // Set the end points of the line segment 2 before the expected
         // closest point, so we have expectedClosestPointOnLineSegment2 =
         // lineSegmentEnd2
         double shiftStartFromExpected = EuclidCoreRandomTools.nextDouble(random, -20.0, -10.0);
         double shiftEndFromExpected = EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0);
         lineSegmentStart2.scaleAdd(shiftStartFromExpected, lineSegmentDirection2, closestPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(shiftEndFromExpected, lineSegmentDirection2, closestPointOnLineSegment2);
         closestPointOnLineSegment2.set(lineSegmentEnd2);
         expectedMinimumDistance = closestPointOnLineSegment1.distance(closestPointOnLineSegment2);

         actualMinimumDistance = EuclidCoreMissingTools.distanceBetweenTwoLineSegment2Ds(lineSegmentStart1,
                                                                                         lineSegmentEnd1,
                                                                                         lineSegmentStart2,
                                                                                         lineSegmentEnd2);
         Assert.assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);

         actualMinimumDistance = EuclidCoreMissingTools.distanceBetweenTwoLineSegment2Ds(lineSegmentStart1,
                                                                                         lineSegmentEnd1,
                                                                                         lineSegmentEnd2,
                                                                                         lineSegmentStart2);
         Assert.assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
      }

      // Case: on closest point on lineSegment1 outside end points.
      for (int i = 0; i < iters; i++)
      {
         Point2D lineSegmentStart1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // Put the first closest to the start of line segment 1
         closestPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2 such that it reaches
         // out of line segment 1
         Vector2D oppositeOflineSegmentDirection1 = new Vector2D();
         oppositeOflineSegmentDirection1.setAndNegate(lineSegmentDirection1);
         Vector2D orthogonalToLineSegment1 = nextOrthogonalVector2D(random, lineSegmentDirection1, true);
         Vector2D shiftVector = new Vector2D();
         shiftVector.interpolate(orthogonalToLineSegment1, oppositeOflineSegmentDirection1, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         closestPointOnLineSegment2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), shiftVector, closestPointOnLineSegment1);

         // Set the line direction 2 to orthogonal to the shift vector
         lineSegmentDirection2 = nextOrthogonalVector2D(random, shiftVector, true);

         // Set the end points of the line segment 2 around the expected
         // closest point.
         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineSegmentDirection2, closestPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineSegmentDirection2, closestPointOnLineSegment2);
         double expectedMinimumDistance = closestPointOnLineSegment1.distance(closestPointOnLineSegment2);

         double actualMinimumDistance = EuclidCoreMissingTools.distanceBetweenTwoLineSegment2Ds(lineSegmentStart1,
                                                                                                lineSegmentEnd1,
                                                                                                lineSegmentStart2,
                                                                                                lineSegmentEnd2);
         Assert.assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         actualMinimumDistance = EuclidCoreMissingTools.distanceBetweenTwoLineSegment2Ds(lineSegmentStart1,
                                                                                         lineSegmentEnd1,
                                                                                         lineSegmentEnd2,
                                                                                         lineSegmentStart2);
         Assert.assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         actualMinimumDistance = EuclidCoreMissingTools.distanceBetweenTwoLineSegment2Ds(lineSegmentEnd1,
                                                                                         lineSegmentStart1,
                                                                                         lineSegmentStart2,
                                                                                         lineSegmentEnd2);
         Assert.assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         actualMinimumDistance = EuclidCoreMissingTools.distanceBetweenTwoLineSegment2Ds(lineSegmentEnd1,
                                                                                         lineSegmentStart1,
                                                                                         lineSegmentEnd2,
                                                                                         lineSegmentStart2);
         Assert.assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
      }

      // Edge case: both closest points are outside bounds of each line
      // segment
      for (int i = 0; i < iters; i++)
      {
         Point2D lineSegmentStart1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // Put the first closest to the start of line segment 1
         closestPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2 such that it reaches
         // out of line segment 1
         Vector2D oppositeOflineSegmentDirection1 = new Vector2D();
         oppositeOflineSegmentDirection1.setAndNegate(lineSegmentDirection1);
         Vector2D orthogonalToLineSegment1 = nextOrthogonalVector2D(random, lineSegmentDirection1, true);
         Vector2D shiftVector = new Vector2D();
         double alpha = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         shiftVector.interpolate(orthogonalToLineSegment1, oppositeOflineSegmentDirection1, alpha);
         closestPointOnLineSegment2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), shiftVector, closestPointOnLineSegment1);

         // set the start of the second line segment to the expected closest
         // point
         Point2D lineSegmentStart2 = new Point2D(closestPointOnLineSegment2);

         // Set the line direction 2 to point somewhat in the same direction
         // as the shift vector
         Vector2D orthogonalToShiftVector = nextOrthogonalVector2D(random, shiftVector, true);
         lineSegmentDirection2.interpolate(shiftVector, orthogonalToShiftVector, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         // Set the end points of the line segment 2 around the expected
         // closest point.
         Point2D lineSegmentEnd2 = new Point2D();
         double alpha2 = EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0);
         lineSegmentEnd2.scaleAdd(alpha2, lineSegmentDirection2, closestPointOnLineSegment2);

         double expectedMinimumDistance = closestPointOnLineSegment1.distance(closestPointOnLineSegment2);
         double actualMinimumDistance = EuclidCoreMissingTools.distanceBetweenTwoLineSegment2Ds(lineSegmentStart1,
                                                                                                lineSegmentEnd1,
                                                                                                lineSegmentStart2,
                                                                                                lineSegmentEnd2);
         Assert.assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         actualMinimumDistance = EuclidCoreMissingTools.distanceBetweenTwoLineSegment2Ds(lineSegmentStart1,
                                                                                         lineSegmentEnd1,
                                                                                         lineSegmentEnd2,
                                                                                         lineSegmentStart2);
         Assert.assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         actualMinimumDistance = EuclidCoreMissingTools.distanceBetweenTwoLineSegment2Ds(lineSegmentEnd1,
                                                                                         lineSegmentStart1,
                                                                                         lineSegmentStart2,
                                                                                         lineSegmentEnd2);
         Assert.assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         actualMinimumDistance = EuclidCoreMissingTools.distanceBetweenTwoLineSegment2Ds(lineSegmentEnd1,
                                                                                         lineSegmentStart1,
                                                                                         lineSegmentEnd2,
                                                                                         lineSegmentStart2);
         Assert.assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
      }
   }

   @Test
   public void testExtractNormalPart()
   {
      Random random = new Random(6457);

      for (int i = 0; i < iters; i++)
      { // We build the normal and tangential parts of the vector and then assemble it and expect to get the normal part pack.
         Vector3D normalAxis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Vector3D tangentialAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, normalAxis, true);
         Vector3D normalPart = new Vector3D();
         Vector3D tangentialPart = new Vector3D();

         double normalMagnitude = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double tangentialMagnitude = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         normalPart.setAndScale(normalMagnitude, normalAxis);
         tangentialPart.setAndScale(tangentialMagnitude, tangentialAxis);

         Vector3D input = new Vector3D();
         input.add(normalPart, tangentialPart);

         Vector3D actualNormalPart = new Vector3D();
         EuclidCoreMissingTools.extractNormalPart(input, normalAxis, actualNormalPart);
         EuclidCoreTestTools.assertTuple3DEquals(normalPart, actualNormalPart, EPSILON);

         // Randomize the axis
         normalAxis.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));
         EuclidCoreMissingTools.extractNormalPart(input, normalAxis, actualNormalPart);
         EuclidCoreTestTools.assertTuple3DEquals(normalPart, actualNormalPart, EPSILON);
      }
   }

   @Test
   public void testExtractTangentialPart()
   {
      Random random = new Random(63457);

      for (int i = 0; i < iters; i++)
      { // We build the normal and tangential parts of the vector and then assemble it and expect to get the normal part pack.
         // Setting trivial setup using Axis3D
         Axis3D normalAxis = EuclidCoreRandomTools.nextAxis3D(random);
         Axis3D tangentialAxis = random.nextBoolean() ? normalAxis.next() : normalAxis.previous();
         Vector3D normalPart = new Vector3D();
         Vector3D tangentialPart = new Vector3D();

         double normalMagnitude = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double tangentialMagnitude = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         normalPart.setAndScale(normalMagnitude, normalAxis);
         tangentialPart.setAndScale(tangentialMagnitude, tangentialAxis);

         Vector3D input = new Vector3D();
         input.add(normalPart, tangentialPart);

         Vector3D actualTangentialPart = new Vector3D();
         EuclidCoreMissingTools.extractTangentialPart(input, normalAxis, actualTangentialPart);
         EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, tangentialPart, actualTangentialPart, EPSILON);
      }

      for (int i = 0; i < iters; i++)
      { // We build the normal and tangential parts of the vector and then assemble it and expect to get the normal part pack.
         Vector3D normalAxis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Vector3D tangentialAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, normalAxis, true);
         Vector3D normalPart = new Vector3D();
         Vector3D tangentialPart = new Vector3D();

         double normalMagnitude = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double tangentialMagnitude = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         normalPart.setAndScale(normalMagnitude, normalAxis);
         tangentialPart.setAndScale(tangentialMagnitude, tangentialAxis);

         Vector3D input = new Vector3D();
         input.add(normalPart, tangentialPart);

         Vector3D actualTangentialPart = new Vector3D();
         EuclidCoreMissingTools.extractTangentialPart(input, normalAxis, actualTangentialPart);
         EuclidCoreTestTools.assertTuple3DEquals(tangentialPart, actualTangentialPart, EPSILON);

         // Randomize the axis
         normalAxis.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));
         EuclidCoreMissingTools.extractTangentialPart(input, normalAxis, actualTangentialPart);
         EuclidCoreTestTools.assertTuple3DEquals(tangentialPart, actualTangentialPart, EPSILON);
      }
   }

   /**
    * Generates a random vector that is perpendicular to {@code vectorToBeOrthogonalTo}.
    *
    * @param random                 the random generator to use.
    * @param vectorToBeOrthogonalTo the vector to be orthogonal to. Not modified.
    * @param normalize              whether to normalize the generated vector or not.
    * @return the random vector.
    */
   public static Vector2D nextOrthogonalVector2D(Random random, Vector2DReadOnly vectorToBeOrthogonalTo, boolean normalize)
   {
      Vector2D v1 = new Vector2D(vectorToBeOrthogonalTo.getY(), -vectorToBeOrthogonalTo.getX());

      Vector2D randomPerpendicular = new Vector2D();
      double a = nextDouble(random, 1.0);
      randomPerpendicular.scaleAdd(a, v1, randomPerpendicular);

      if (normalize)
         randomPerpendicular.normalize();

      return randomPerpendicular;
   }

   public static double nextDouble(Random random, double minMaxValue)
   {
      return nextDouble(random, -minMaxValue, minMaxValue);
   }

   public static double nextDouble(Random random, double minValue, double maxValue)
   {
      if (minValue > maxValue)
         throw new RuntimeException("Min is greater than max: min = " + minValue + ", max = " + maxValue);

      return minValue + random.nextDouble() * (maxValue - minValue);
   }

}
