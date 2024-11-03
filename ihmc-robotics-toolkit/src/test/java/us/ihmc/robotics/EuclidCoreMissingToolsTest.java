package us.ihmc.robotics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
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
      EuclidCoreTestTools.assertEquals(new Vector3D(0.1234567, 100.1234567, 1000.1234567), roundedVector, 1e-12);

      EuclidCoreMissingTools.floorToGivenPrecision(roundedVector, 1e-3);
      EuclidCoreTestTools.assertEquals(new Vector3D(0.123, 100.123, 1000.123), roundedVector, 1e-14);
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
         EuclidCoreTestTools.assertOrientation3DGeometricallyEquals(expectedRotation, actualRotation, 1.0e-10);
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
         EuclidCoreTestTools.assertOrientation3DGeometricallyEquals(expectedAxisAngle, actualAxisAngle, 1.0e-7);
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
         EuclidCoreTestTools.assertEquals(normalPart, actualNormalPart, EPSILON);

         // Randomize the axis
         normalAxis.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));
         EuclidCoreMissingTools.extractNormalPart(input, normalAxis, actualNormalPart);
         EuclidCoreTestTools.assertEquals(normalPart, actualNormalPart, EPSILON);
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
         EuclidCoreTestTools.assertEquals("Iteration: " + i, tangentialPart, actualTangentialPart, EPSILON);
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
         EuclidCoreTestTools.assertEquals(tangentialPart, actualTangentialPart, EPSILON);

         // Randomize the axis
         normalAxis.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));
         EuclidCoreMissingTools.extractTangentialPart(input, normalAxis, actualTangentialPart);
         EuclidCoreTestTools.assertEquals(tangentialPart, actualTangentialPart, EPSILON);
      }
   }

   @Test
   public void testIntersectionBetweenRay2DAndLine2D()
   {
      Point2D rayOrigin = new Point2D(8.689, 0.5687);
      Point2D pointOnRay = new Point2D(8.6432, 0.4951);
      Vector2D rayDirection = new Vector2D();
      rayDirection.sub(pointOnRay, rayOrigin);

      Point2D lineStart = new Point2D(8.4521, 0.4323);
      Point2D lineEnd = new Point2D(8.776, 0.5267);
      Vector2D lineDirection = new Vector2D();
      lineDirection.sub(lineEnd, lineStart);

      Point2D intersectionToPac = new Point2D();
      assertTrue(EuclidCoreMissingTools.intersectionBetweenRay2DAndLine2D(rayOrigin, rayDirection, lineStart, lineDirection, intersectionToPac));

      Point2D intersectionExpected = new Point2D();
      EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(rayOrigin, rayDirection, lineStart, lineEnd, intersectionExpected);

      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(intersectionExpected, intersectionToPac, 1e-5);
   }

   @Test
   public void testSetNormalPart()
   {
      Random random = new Random(897632);

      for (int i = 0; i < iters; i++)
      {
         // Setting trivial setup using Axis3D
         Axis3D normalAxis = EuclidCoreRandomTools.nextAxis3D(random);
         Axis3D tangentialAxis = random.nextBoolean() ? normalAxis.next() : normalAxis.previous();
         Vector3D normalPart = new Vector3D();
         Vector3D tangentialPart = new Vector3D();
         double normalMagnitude = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double tangentialMagnitude = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         normalPart.setAndScale(normalMagnitude, normalAxis);
         tangentialPart.setAndScale(tangentialMagnitude, tangentialAxis);

         Point3D input = new Point3D(normalPart);
         input.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), tangentialPart, input);

         Vector3D tupleToModify = new Vector3D(tangentialPart);
         tupleToModify.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), normalPart, tupleToModify);

         Vector3D expected = new Vector3D();
         expected.add(normalPart, tangentialPart);

         EuclidCoreMissingTools.setNormalPart(input, normalAxis, tupleToModify);
         EuclidCoreTestTools.assertEquals(expected, tupleToModify, EPSILON);
      }

      for (int i = 0; i < iters; i++)
      {
         Vector3D normalAxis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Vector3D tangentialAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, normalAxis, true);
         Vector3D normalPart = new Vector3D();
         Vector3D tangentialPart = new Vector3D();
         double normalMagnitude = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double tangentialMagnitude = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         normalPart.setAndScale(normalMagnitude, normalAxis);
         tangentialPart.setAndScale(tangentialMagnitude, tangentialAxis);

         Point3D input = new Point3D(normalPart);
         input.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), tangentialPart, input);

         Vector3D tupleToModify = new Vector3D(tangentialPart);
         tupleToModify.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), normalPart, tupleToModify);

         Vector3D expected = new Vector3D();
         expected.add(normalPart, tangentialPart);

         EuclidCoreMissingTools.setNormalPart(input, normalAxis, tupleToModify);
         EuclidCoreTestTools.assertEquals(expected, tupleToModify, EPSILON);

         // Randomize the axis
         normalAxis.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));
         EuclidCoreMissingTools.setNormalPart(input, normalAxis, tupleToModify);
         EuclidCoreTestTools.assertEquals(expected, tupleToModify, EPSILON);
      }
   }

   @Test
   public void testDifferentiateOrientation()
   {
      Random random = new Random(23423);

      for (int i = 0; i < iters; i++)
      {
         Quaternion qStart = EuclidCoreRandomTools.nextQuaternion(random);
         double duration = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0e-2);
         double angle = EuclidCoreRandomTools.nextDouble(random, 0.0, Math.PI);
         UnitVector3D velocityAxis = EuclidCoreRandomTools.nextUnitVector3D(random);
         Vector3D expectedAngularVelocity = new Vector3D();
         expectedAngularVelocity.setAndScale(angle / duration, velocityAxis);

         Quaternion qEnd = new Quaternion();
         qEnd.setRotationVector(velocityAxis.getX() * angle, velocityAxis.getY() * angle, velocityAxis.getZ() * angle);
         qEnd.prepend(qStart);

         Vector3D actualAngularVelocity = new Vector3D();
         EuclidCoreMissingTools.differentiateOrientation(qStart, qEnd, duration, actualAngularVelocity);

         EuclidCoreTestTools.assertEquals(expectedAngularVelocity, actualAngularVelocity, EPSILON * Math.max(1.0, expectedAngularVelocity.norm()));
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

   @Test
   public void testNextPositiveDefiniteMatrix3D() throws Exception
   {
      Random random = new Random(23452);

      for (int i = 0; i < iters; i++) { // Test nextPositiveDefiniteMatrix3D(Random random)
         Matrix3D matrix3D = EuclidCoreMissingTools.nextPositiveDefiniteMatrix3D(random);

         for (int row = 0; row < 3; row++) {
            for (int column = 0; column < 3; column++) {
               assertTrue(matrix3D.getElement(row, column) <= 1.0);
               assertTrue(matrix3D.getElement(row, column) >= -1.0);
               assertTrue(Double.isFinite(matrix3D.getElement(row, column)));
            }
         }

         // Using Sylvester's criterion of all the leading principal minors having positive determinant to verify that matrix is positive definite
         double firstPrincipalMinorDeterminant = matrix3D.getM00();
         double secondPrincipalMinorDeterminant = matrix3D.getM00() * matrix3D.getM11() - matrix3D.getM01() * matrix3D.getM10();
         double thirdPrincipalMinorDeterminant = matrix3D.determinant();
         assertTrue(firstPrincipalMinorDeterminant > 0.0);
         assertTrue(secondPrincipalMinorDeterminant > 0.0);
         assertTrue(thirdPrincipalMinorDeterminant > 0.0);
      }

      for (int i = 0; i < iters; i++) { // Test nextPositiveDefiniteMatrix3D(Random random, double minMaxValue)
         double minMaxValue = EuclidCoreRandomTools.nextDouble(random, 0.0, 100.0);
         Matrix3D matrix3D = EuclidCoreMissingTools.nextPositiveDefiniteMatrix3D(random, minMaxValue);

         for (int row = 0; row < 3; row++) {
            for (int column = 0; column < 3; column++) {
               assertTrue(matrix3D.getElement(row, column) <= minMaxValue);
               assertTrue(matrix3D.getElement(row, column) >= -minMaxValue);
               assertTrue(Double.isFinite(matrix3D.getElement(row, column)));
            }
         }

         // Using Sylvester's criterion of all the leading principal minors having positive determinant to verify that matrix is positive definite
         double firstPrincipalMinorDeterminant = matrix3D.getM00();
         double secondPrincipalMinorDeterminant = matrix3D.getM00() * matrix3D.getM11() - matrix3D.getM01() * matrix3D.getM10();
         double thirdPrincipalMinorDeterminant = matrix3D.determinant();
         assertTrue(firstPrincipalMinorDeterminant > 0.0);
         assertTrue(secondPrincipalMinorDeterminant > 0.0);
         assertTrue(thirdPrincipalMinorDeterminant > 0.0);
      }

      // Test exceptions:
      try {
         EuclidCoreMissingTools.nextPositiveDefiniteMatrix3D(random, -0.1);
         fail("Should have thrown an exception.");
      } catch (RuntimeException e) {
         // Good
      }
   }
}
