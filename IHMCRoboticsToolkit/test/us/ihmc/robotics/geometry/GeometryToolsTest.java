package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2006</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class GeometryToolsTest
{
   private static final int ITERATIONS = 1000;

   @Before
   public void setUp() throws Exception
   {
   }

   @After
   public void tearDown() throws Exception
   {
   }

   private static final double EPSILON = 1e-6;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAveragePoints()
   {
      Point3D a = new Point3D(5.8, 9.9, 4.5);
      Point3D b = new Point3D(5.6, 8.1, 5.5);
      double expectedReturn1 = 5.7;
      double expectedReturn2 = 9.0;
      double expectedReturn3 = 5;
      Point3D actualReturn = GeometryTools.averagePoints(a, b);
      double actualReturn1 = actualReturn.getX();
      double actualReturn2 = actualReturn.getY();
      double actualReturn3 = actualReturn.getZ();
      assertEquals("return value", expectedReturn1, actualReturn1, EPSILON);
      assertEquals("return value", expectedReturn2, actualReturn2, EPSILON);
      assertEquals("return value", expectedReturn3, actualReturn3, EPSILON);

      Point3D a1 = new Point3D(-5, -5, -5);
      Point3D b1 = new Point3D(-5, -5, -5);
      double expectedReturn11 = -5;
      double expectedReturn12 = -5;
      double expectedReturn13 = -5;
      Point3D actualReturn01 = GeometryTools.averagePoints(a1, b1);
      double actualReturn11 = actualReturn01.getX();
      double actualReturn12 = actualReturn01.getY();
      double actualReturn13 = actualReturn01.getZ();
      assertEquals("return value", expectedReturn11, actualReturn11, EPSILON);
      assertEquals("return value", expectedReturn12, actualReturn12, EPSILON);
      assertEquals("return value", expectedReturn13, actualReturn13, EPSILON);

      Point3D a2 = new Point3D(0, 0, 0);
      Point3D b2 = new Point3D(0, 0, 0);
      double expectedReturn21 = 0;
      double expectedReturn22 = 0;
      double expectedReturn23 = 0;
      Point3D actualReturn02 = GeometryTools.averagePoints(a2, b2);
      double actualReturn21 = actualReturn02.getX();
      double actualReturn22 = actualReturn02.getY();
      double actualReturn23 = actualReturn02.getZ();
      assertEquals("return value", expectedReturn21, actualReturn21, EPSILON);
      assertEquals("return value", expectedReturn22, actualReturn22, EPSILON);
      assertEquals("return value", expectedReturn23, actualReturn23, EPSILON);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAveragePoints1()
   {
      ArrayList<Point2D> points = new ArrayList<Point2D>();
      Point2D a = new Point2D(1.0, 4.6);
      Point2D b = new Point2D(5.2, 6.0);
      Point2D c = new Point2D(3.7, 2.0);
      points.add(a);
      points.add(b);
      points.add(c);
      double expectedReturn1 = 3.3;
      double expectedReturn2 = 4.2;
      Point2D actualReturn = GeometryTools.averagePoint2ds(points);
      double actualReturn1 = actualReturn.getX();
      double actualReturn2 = actualReturn.getY();
      assertEquals("return value", expectedReturn1, actualReturn1, EPSILON);
      assertEquals("return value", expectedReturn2, actualReturn2, EPSILON);

      ArrayList<Point2D> points1 = new ArrayList<Point2D>();
      Point2D a1 = new Point2D(0.0, 0.0);
      Point2D b1 = new Point2D(0.0, 0.0);
      Point2D c1 = new Point2D(0.0, 0.0);
      points1.add(a1);
      points1.add(b1);
      points1.add(c1);
      double expectedReturn11 = 0.0;
      double expectedReturn12 = 0.0;
      Point2D actualReturn01 = GeometryTools.averagePoint2ds(points1);
      double actualReturn11 = actualReturn01.getX();
      double actualReturn12 = actualReturn01.getY();
      assertEquals("return value", expectedReturn11, actualReturn11, EPSILON);
      assertEquals("return value", expectedReturn12, actualReturn12, EPSILON);

      ArrayList<Point2D> points2 = new ArrayList<Point2D>();
      Point2D a2 = new Point2D(-1.0, -4.6);
      Point2D b2 = new Point2D(-5.2, -6.0);
      Point2D c2 = new Point2D(-3.7, -2.0);
      points2.add(a2);
      points2.add(b2);
      points2.add(c2);
      double expectedReturn21 = -3.3;
      double expectedReturn22 = -4.2;
      Point2D actualReturn02 = GeometryTools.averagePoint2ds(points2);
      double actualReturn21 = actualReturn02.getX();
      double actualReturn22 = actualReturn02.getY();
      assertEquals("return value", expectedReturn21, actualReturn21, EPSILON);
      assertEquals("return value", expectedReturn22, actualReturn22, EPSILON);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAveragePoints2()
   {
      ArrayList<Point3D> points = new ArrayList<Point3D>();
      Point3D a = new Point3D(4.3, 5.6, 3.6);
      Point3D b = new Point3D(8.1, 8.4, 0.0);
      Point3D c = new Point3D(5.6, 1.0, 4.5);
      points.add(a);
      points.add(b);
      points.add(c);
      double expectedReturn1 = 6.0;
      double expectedReturn2 = 5.0;
      double expectedReturn3 = 2.7;
      Point3D actualReturn = GeometryTools.averagePoint3ds(points);
      double actualReturn1 = actualReturn.getX();
      double actualReturn2 = actualReturn.getY();
      double actualReturn3 = actualReturn.getZ();
      assertEquals("return value", expectedReturn1, actualReturn1, EPSILON);
      assertEquals("return value", expectedReturn2, actualReturn2, EPSILON);
      assertEquals("return value", expectedReturn3, actualReturn3, EPSILON);

      ArrayList<Point3D> points1 = new ArrayList<Point3D>();
      Point3D a1 = new Point3D(0.0, 0.0, 0.0);
      Point3D b1 = new Point3D(0.0, 0.0, 0.0);
      Point3D c1 = new Point3D(0.0, 0.0, 0.0);
      points1.add(a1);
      points1.add(b1);
      points1.add(c1);
      double expectedReturn11 = 0.0;
      double expectedReturn12 = 0.0;
      double expectedReturn13 = 0.0;
      Point3D actualReturn01 = GeometryTools.averagePoint3ds(points1);
      double actualReturn11 = actualReturn01.getX();
      double actualReturn12 = actualReturn01.getY();
      double actualReturn13 = actualReturn01.getZ();
      assertEquals("return value", expectedReturn11, actualReturn11, EPSILON);
      assertEquals("return value", expectedReturn12, actualReturn12, EPSILON);
      assertEquals("return value", expectedReturn13, actualReturn13, EPSILON);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceFromPointToLine()
   {
      Point2D point = new Point2D(10, 2);
      Point2D lineStart = new Point2D(4, 2);
      Point2D lineEnd = new Point2D(10, 10);
      double expectedReturn = 4.8;
      double actualReturn = GeometryTools.distanceFromPointToLine(point, lineStart, lineEnd);
      assertEquals("return value", expectedReturn, actualReturn, Double.MIN_VALUE);

      Point2D point1 = new Point2D(10, 2);
      Point2D lineStart1 = new Point2D(10, 1);
      Point2D lineEnd1 = new Point2D(10, 10);
      double expectedReturn1 = 0.0;
      double actualReturn1 = GeometryTools.distanceFromPointToLine(point1, lineStart1, lineEnd1);
      assertEquals("return value", expectedReturn1, actualReturn1, Double.MIN_VALUE);

      Point2D point2 = new Point2D(1, 2);
      Point2D lineStart2 = new Point2D(4, 2);
      Point2D lineEnd2 = new Point2D(10, 10);
      double expectedReturn2 = 2.4;
      double actualReturn2 = GeometryTools.distanceFromPointToLine(point2, lineStart2, lineEnd2);
      assertEquals("return value", expectedReturn2, actualReturn2, EPSILON);

      Point2D point3 = new Point2D(10, 10);
      Point2D lineStart3 = new Point2D(4, 2);
      Point2D lineEnd3 = new Point2D(4, 2);
      double expectedReturn3 = 10;
      double actualReturn3 = GeometryTools.distanceFromPointToLine(point3, lineStart3, lineEnd3);
      assertEquals("return value", expectedReturn3, actualReturn3, Double.MIN_VALUE);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceFromPointToLine3D()
   {
      Random random = new Random(1176L);
      Point3D point = new Point3D(10, 2, 0);
      Point3D lineStart = new Point3D(4, 2, 0);
      Point3D lineEnd = new Point3D(10, 10, 0);
      double expectedReturn = 4.8;
      double actualReturn = GeometryTools.distanceFromPointToLine(point, lineStart, lineEnd);
      assertEquals("return value", expectedReturn, actualReturn, Double.MIN_VALUE);

      Point3D point2 = new Point3D(3, 3, 0);
      Point3D lineStart2 = new Point3D(0, 0, 0);
      Point3D lineEnd2 = new Point3D(3, 3, 3);
      double expectedReturn2 = 2.44948974278;
      double actualReturn2 = GeometryTools.distanceFromPointToLine(point2, lineStart2, lineEnd2);
      assertEquals("return value", expectedReturn2, actualReturn2, EPSILON);

      Point3D point1 = new Point3D(10, 10, 0);
      Point3D lineStart1 = new Point3D(4, 2, 0);
      Point3D lineEnd1 = new Point3D(4, 2, 0);
      double expectedReturn1 = 10.0;
      double actualReturn1 = GeometryTools.distanceFromPointToLine(point1, lineStart1, lineEnd1);
      assertEquals("return value", expectedReturn1, actualReturn1, Double.MIN_VALUE);

      for (int i = 0; i < ITERATIONS; i++)
      {
         // Generate a random line
         Point3D start = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Point3D end = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Vector3D lineDirection = new Vector3D();
         lineDirection.sub(end, start);
         // Generate a random vector orthogonal to the line
         Vector3D orthogonalVector = RandomGeometry.nextOrthogonalVector3D(random, lineDirection, true);
         double expectedDistance = RandomNumbers.nextDouble(random, 0.0, 10.0);
         // Generate a random point located at an expected distance from the line
         Point3D randomPoint = new Point3D();
         // Randomize on the line
         randomPoint.interpolate(start, end, RandomNumbers.nextDouble(random, 10.0));
         // Move the point away from the line by the expected distance
         randomPoint.scaleAdd(expectedDistance, orthogonalVector, randomPoint);

         double actualDistance = GeometryTools.distanceFromPointToLine(randomPoint, start, end);
         assertEquals(expectedDistance, actualDistance, 1.0e-12);

         actualDistance = GeometryTools.distanceFromPointToLine(randomPoint, start, lineDirection);
         assertEquals(expectedDistance, actualDistance, 1.0e-12);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceFromPointToLineSegment2D()
   {
      Point2D point = new Point2D(10, 2);
      Point2D lineStart = new Point2D(4, 2);
      Point2D lineEnd = new Point2D(10, 10);
      double expectedReturn = 4.8;
      double actualReturn = GeometryTools.distanceFromPointToLineSegment(point, lineStart, lineEnd);
      assertEquals("return value", expectedReturn, actualReturn, Double.MIN_VALUE);

      Point2D point1 = new Point2D(10, 10);
      Point2D lineStart1 = new Point2D(4, 2);
      Point2D lineEnd1 = new Point2D(4, 2);
      double expectedReturn1 = 10.0;
      double actualReturn1 = GeometryTools.distanceFromPointToLineSegment(point1, lineStart1, lineEnd1);
      assertEquals("return value", expectedReturn1, actualReturn1, Double.MIN_VALUE);

      Point2D point2 = new Point2D(1, 1);
      Point2D lineStart2 = new Point2D(4, 2);
      Point2D lineEnd2 = new Point2D(5, 5);
      double expectedReturn2 = 3.16227766017;
      double actualReturn2 = GeometryTools.distanceFromPointToLineSegment(point2, lineStart2, lineEnd2);
      assertEquals("return value", expectedReturn2, actualReturn2, EPSILON);

      point2 = new Point2D(1, 1);
      lineStart2 = new Point2D(5, 5);
      lineEnd2 = new Point2D(5, 5);
      expectedReturn2 = lineStart2.distance(point2);
      actualReturn2 = GeometryTools.distanceFromPointToLineSegment(point2, lineStart2, lineEnd2);
      assertEquals("return value", expectedReturn2, actualReturn2, EPSILON);

      Random random = new Random(32423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D lineSegmentEnd = RandomGeometry.nextPoint2D(random, 10.0, 10.0);

         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(lineSegmentEnd, lineSegmentStart);
         GeometryTools.getPerpendicularVector(orthogonal, orthogonal);
         orthogonal.normalize();
         Point2D projection = new Point2D();
         Point2D testPoint = new Point2D();
         double expectedDistance, actualDistance;

         // Between end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, RandomNumbers.nextDouble(random, 0.0, 1.0));
         testPoint.scaleAdd(RandomNumbers.nextDouble(random, 10.0), orthogonal, projection);
         expectedDistance = projection.distance(testPoint);
         actualDistance = GeometryTools.distanceFromPointToLineSegment(testPoint, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedDistance, actualDistance, Epsilons.ONE_TRILLIONTH);

         // Before end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, RandomNumbers.nextDouble(random, -10.0, 0.0));
         testPoint.scaleAdd(RandomNumbers.nextDouble(random, 10.0), orthogonal, projection);
         projection.set(lineSegmentStart);
         expectedDistance = projection.distance(testPoint);
         actualDistance = GeometryTools.distanceFromPointToLineSegment(testPoint, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedDistance, actualDistance, Epsilons.ONE_TRILLIONTH);

         // After end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, RandomNumbers.nextDouble(random, 1.0, 10.0));
         testPoint.scaleAdd(RandomNumbers.nextDouble(random, 10.0), orthogonal, projection);
         projection.set(lineSegmentEnd);
         expectedDistance = projection.distance(testPoint);
         actualDistance = GeometryTools.distanceFromPointToLineSegment(testPoint, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedDistance, actualDistance, Epsilons.ONE_TRILLIONTH);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceFromPointToLineSegment3D()
   {
      Random random = new Random(32423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Point3D lineSegmentEnd = RandomGeometry.nextPoint3D(random, -10.0, 10.0);

         Vector3D lineSegmentDirection = new Vector3D();
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineSegmentDirection.normalize();
         Vector3D orthogonal = RandomGeometry.nextOrthogonalVector3D(random, lineSegmentDirection, true);
         Point3D projection = new Point3D();
         Point3D testPoint = new Point3D();
         double expectedDistance, actualDistance;

         // Between end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, RandomNumbers.nextDouble(random, 0.0, 1.0));
         testPoint.scaleAdd(RandomNumbers.nextDouble(random, 10.0), orthogonal, projection);
         expectedDistance = projection.distance(testPoint);
         actualDistance = GeometryTools.distanceFromPointToLineSegment(testPoint, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedDistance, actualDistance, Epsilons.ONE_TRILLIONTH);

         // Before end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, RandomNumbers.nextDouble(random, -10.0, 0.0));
         testPoint.scaleAdd(RandomNumbers.nextDouble(random, 10.0), orthogonal, projection);
         projection.set(lineSegmentStart);
         expectedDistance = projection.distance(testPoint);
         actualDistance = GeometryTools.distanceFromPointToLineSegment(testPoint, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedDistance, actualDistance, Epsilons.ONE_TRILLIONTH);

         // After end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, RandomNumbers.nextDouble(random, 1.0, 10.0));
         testPoint.scaleAdd(RandomNumbers.nextDouble(random, 10.0), orthogonal, projection);
         projection.set(lineSegmentEnd);
         expectedDistance = projection.distance(testPoint);
         actualDistance = GeometryTools.distanceFromPointToLineSegment(testPoint, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedDistance, actualDistance, Epsilons.ONE_TRILLIONTH);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testDistanceBetweenTwoLines() throws Exception
   {
      Point3D closestPointOnLine1 = new Point3D();
      Point3D closestPointOnLine2 = new Point3D();

      Random random = new Random(176L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineStart1 = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Vector3D lineDirection1 = RandomGeometry.nextVector3D(random, RandomNumbers.nextDouble(random, 0.5, 10.0));

         // Make line2 == line1
         Point3D lineStart2 = new Point3D(lineStart1);
         Vector3D lineDirection2 = new Vector3D(lineDirection1);

         // Shift orthogonally line2 away from line1.
         Vector3D orthogonalToLine1 = RandomGeometry.nextOrthogonalVector3D(random, lineDirection1, true);
         double expectedMinimumDistance = RandomNumbers.nextDouble(random, 0.0, 10.0);
         lineStart2.scaleAdd(expectedMinimumDistance, orthogonalToLine1, lineStart1);

         // Rotate line2 around the vector we shifted it along, so it preserves the minimum distance.
         AxisAngle axisAngleAroundShiftVector = new AxisAngle(orthogonalToLine1, RandomNumbers.nextDouble(random, Math.PI));
         RotationMatrix rotationMatrixAroundShiftVector = new RotationMatrix();
         rotationMatrixAroundShiftVector.set(axisAngleAroundShiftVector);
         rotationMatrixAroundShiftVector.transform(lineDirection2);

         // At this point, lineStart1 and lineStart2 are expected to be the closest points.
         closestPointOnLine1.set(lineStart1);
         closestPointOnLine2.set(lineStart2);

         double actualMinimumDistance = GeometryTools.distanceBetweenTwoLines(lineStart1, lineDirection1, lineStart2, lineDirection2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);

         // Let's shift lineStart1 and lineStart2 along their respective line direction so they're not the closest points.
         lineStart1.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection1, lineStart1);
         lineStart2.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection2, lineStart2);

         actualMinimumDistance = GeometryTools.distanceBetweenTwoLines(lineStart1, lineDirection1, lineStart2, lineDirection2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
      }

      // Test the parallel case. There's an infinite number of solutions but only one minimum distance between the two lines.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineStart1 = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Vector3D lineDirection1 = RandomGeometry.nextVector3D(random, 1.0);

         // Make line2 == line1
         Point3D lineStart2 = new Point3D(lineStart1);
         Vector3D lineDirection2 = new Vector3D(lineDirection1);

         // Shift orthogonally line2 away from line1.
         Vector3D orthogonalToLine1 = RandomGeometry.nextOrthogonalVector3D(random, lineDirection1, true);
         double expectedMinimumDistance = RandomNumbers.nextDouble(random, 0.0, 10.0);
         lineStart2.scaleAdd(expectedMinimumDistance, orthogonalToLine1, lineStart1);

         double actualMinimumDistance = GeometryTools.distanceBetweenTwoLines(lineStart1, lineDirection1, lineStart2, lineDirection2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);

         // Let's shift lineStart1 and lineStart2 along their respective line direction (the minimum distance should remain the same).
         lineStart1.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection1, lineStart1);
         lineStart2.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection2, lineStart2);

         actualMinimumDistance = GeometryTools.distanceBetweenTwoLines(lineStart1, lineDirection1, lineStart2, lineDirection2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testDistanceBetweenTwoLineSegments() throws Exception
   {
      Point3D closestPointOnLineSegment1 = new Point3D();
      Point3D closestPointOnLineSegment2 = new Point3D();

      Vector3D lineSegmentDirection1 = new Vector3D();
      Vector3D lineSegmentDirection2 = new Vector3D();

      Random random = new Random(11762L);

      // Easy case, the closest points on inside each line segment bounds.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart1 = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Point3D lineSegmentEnd1 = RandomGeometry.nextPoint3D(random, -10.0, 10.0);

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // Put the first closest within bounds of line segment 1
         closestPointOnLineSegment1.interpolate(lineSegmentStart1, lineSegmentEnd1, RandomNumbers.nextDouble(random, 0.0, 1.0));

         // Create the closest point of line segment 2
         Vector3D orthogonalToLineSegment1 = RandomGeometry.nextOrthogonalVector3D(random, lineSegmentDirection1, true);
         double expectedMinimumDistance = RandomNumbers.nextDouble(random, 0.0, 10.0);
         closestPointOnLineSegment2.scaleAdd(expectedMinimumDistance, orthogonalToLineSegment1, closestPointOnLineSegment1);

         // Set the line direction 2 to be the rotation of 1 around the shift direction used to create the expectedPointOnLineSegment2
         double rotationAngle = RandomNumbers.nextDouble(random, 2.0 * Math.PI);
         AxisAngle rotationAroundShiftVector = new AxisAngle(orthogonalToLineSegment1, rotationAngle);
         rotationAroundShiftVector.transform(lineSegmentDirection1, lineSegmentDirection2);

         // Set the end points of the line segment 2 around the expected closest point.
         Point3D lineSegmentStart2 = new Point3D();
         Point3D lineSegmentEnd2 = new Point3D();
         lineSegmentStart2.scaleAdd(RandomNumbers.nextDouble(random, -10.0, 0.0), lineSegmentDirection2, closestPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineSegmentDirection2, closestPointOnLineSegment2);

         double actualMinimumDistance = GeometryTools.distanceBetweenTwoLineSegments(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
      }

      // Parallel case, expecting expectedPointOnLineSegment1 = lineSegmentStart1
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart1 = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Point3D lineSegmentEnd1 = RandomGeometry.nextPoint3D(random, -10.0, 10.0);

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // expectedPointOnLineSegment1 = lineSegmentStart1
         closestPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2
         Vector3D orthogonalToLineSegment1 = RandomGeometry.nextOrthogonalVector3D(random, lineSegmentDirection1, true);
         double expectedMinimumDistance = RandomNumbers.nextDouble(random, 0.0, 10.0);
         closestPointOnLineSegment2.scaleAdd(expectedMinimumDistance, orthogonalToLineSegment1, closestPointOnLineSegment1);

         // Set the lineSegmentDirection2 = lineSegmentDirection1
         lineSegmentDirection2.set(lineSegmentDirection1);

         // Set the end points of the line segment 2 around the expected closest point.
         Point3D lineSegmentStart2 = new Point3D();
         Point3D lineSegmentEnd2 = new Point3D();
         lineSegmentStart2.scaleAdd(RandomNumbers.nextDouble(random, -10.0, 0.0), lineSegmentDirection2, closestPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineSegmentDirection2, closestPointOnLineSegment2);

         double actualMinimumDistance = GeometryTools.distanceBetweenTwoLineSegments(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);

         // Set the end points of the line segment 2 before the expected closest point, so we have expectedClosestPointOnLineSegment2 = lineSegmentEnd2
         double shiftStartFromExpected = RandomNumbers.nextDouble(random, -20.0, -10.0);
         double shiftEndFromExpected = RandomNumbers.nextDouble(random, -10.0, 0.0);
         lineSegmentStart2.scaleAdd(shiftStartFromExpected, lineSegmentDirection2, closestPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(shiftEndFromExpected, lineSegmentDirection2, closestPointOnLineSegment2);
         closestPointOnLineSegment2.set(lineSegmentEnd2);
         expectedMinimumDistance = closestPointOnLineSegment1.distance(closestPointOnLineSegment2);

         actualMinimumDistance = GeometryTools.distanceBetweenTwoLineSegments(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);

         actualMinimumDistance = GeometryTools.distanceBetweenTwoLineSegments(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
      }

      // Case: on closest point on lineSegment1 outside end points.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart1 = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Point3D lineSegmentEnd1 = RandomGeometry.nextPoint3D(random, -10.0, 10.0);

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // Put the first closest to the start of line segment 1
         closestPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2 such that it reaches out of line segment 1
         Vector3D oppositeOflineSegmentDirection1 = new Vector3D();
         oppositeOflineSegmentDirection1.setAndNegate(lineSegmentDirection1);
         Vector3D orthogonalToLineSegment1 = RandomGeometry.nextOrthogonalVector3D(random, lineSegmentDirection1, true);
         Vector3D shiftVector = new Vector3D();
         shiftVector.interpolate(orthogonalToLineSegment1, oppositeOflineSegmentDirection1, RandomNumbers.nextDouble(random, 0.0, 1.0));
         closestPointOnLineSegment2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), shiftVector, closestPointOnLineSegment1);

         // Set the line direction 2 to orthogonal to the shift vector
         lineSegmentDirection2 = RandomGeometry.nextOrthogonalVector3D(random, shiftVector, true);

         // Set the end points of the line segment 2 around the expected closest point.
         Point3D lineSegmentStart2 = new Point3D();
         Point3D lineSegmentEnd2 = new Point3D();
         lineSegmentStart2.scaleAdd(RandomNumbers.nextDouble(random, -10.0, 0.0), lineSegmentDirection2, closestPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineSegmentDirection2, closestPointOnLineSegment2);
         double expectedMinimumDistance = closestPointOnLineSegment1.distance(closestPointOnLineSegment2);

         double actualMinimumDistance = GeometryTools.distanceBetweenTwoLineSegments(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         actualMinimumDistance = GeometryTools.distanceBetweenTwoLineSegments(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         actualMinimumDistance = GeometryTools.distanceBetweenTwoLineSegments(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         actualMinimumDistance = GeometryTools.distanceBetweenTwoLineSegments(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
      }
      
      // Edge case: both closest points are outside bounds of each line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart1 = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Point3D lineSegmentEnd1 = RandomGeometry.nextPoint3D(random, -10.0, 10.0);

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // Put the first closest to the start of line segment 1
         closestPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2 such that it reaches out of line segment 1
         Vector3D oppositeOflineSegmentDirection1 = new Vector3D();
         oppositeOflineSegmentDirection1.setAndNegate(lineSegmentDirection1);
         Vector3D orthogonalToLineSegment1 = RandomGeometry.nextOrthogonalVector3D(random, lineSegmentDirection1, true);
         Vector3D shiftVector = new Vector3D();
         shiftVector.interpolate(orthogonalToLineSegment1, oppositeOflineSegmentDirection1, RandomNumbers.nextDouble(random, 0.0, 1.0));
         closestPointOnLineSegment2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), shiftVector, closestPointOnLineSegment1);

         // set the start of the second line segment to the expected closest point
         Point3D lineSegmentStart2 = new Point3D(closestPointOnLineSegment2);
         

         // Set the line direction 2 to point somewhat in the same direction as the shift vector
         Vector3D orthogonalToShiftVector = RandomGeometry.nextOrthogonalVector3D(random, shiftVector, true);
         lineSegmentDirection2.interpolate(shiftVector, orthogonalToShiftVector, RandomNumbers.nextDouble(random, 0.0, 1.0));

         // Set the end points of the line segment 2 around the expected closest point.
         Point3D lineSegmentEnd2 = new Point3D();
         lineSegmentEnd2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineSegmentDirection2, closestPointOnLineSegment2);

         double expectedMinimumDistance = closestPointOnLineSegment1.distance(closestPointOnLineSegment2);
         double actualMinimumDistance = GeometryTools.distanceBetweenTwoLineSegments(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         actualMinimumDistance = GeometryTools.distanceBetweenTwoLineSegments(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         actualMinimumDistance = GeometryTools.distanceBetweenTwoLineSegments(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         actualMinimumDistance = GeometryTools.distanceBetweenTwoLineSegments(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
      }
   }

   /*
    * public void testGetClosestPointsForTwoLines() { Point3D p1 = new
    * Point3D(5, 5, 0); FramePoint point1 = new
    * FramePoint(ReferenceFrame.getWorldFrame(), p1.x, p1.y, p1.z); FrameVector
    * vector1 = null; FramePoint point2 = null; FrameVector vector2 = null;
    * FramePoint pointOnLine1 = null; FramePoint pointOnLine2 = null;
    * geometryTools.getClosestPointsForTwoLines(point1, vector1, point2,
    * vector2, pointOnLine1, pointOnLine2); }
    */

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetDistanceBetweenPointAndPlane1()
   {
      FramePoint pointOnPlane = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      FrameVector planeNormal = new FrameVector(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      FramePoint point = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 3);
      double actual = GeometryTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      double expected = 3.0;
      assertEquals("FAILED: Distance from point to plane", expected, actual, EPSILON);

      pointOnPlane = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      point = new FramePoint(ReferenceFrame.getWorldFrame(), 3, 3, -3);
      actual = GeometryTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      expected = 3.0;
      assertEquals("FAILED: Distance from point to plane", expected, actual, EPSILON);

      pointOnPlane = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      point = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, -3);
      actual = GeometryTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      expected = 3.0;
      assertEquals("FAILED: Distance from point to plane", expected, actual, EPSILON);

      pointOnPlane = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 3);
      planeNormal = new FrameVector(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      point = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, -3);
      actual = GeometryTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      expected = 6.0;
      assertEquals("FAILED: Distance from point to plane", expected, actual, EPSILON);

      pointOnPlane = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector(pointOnPlane.getReferenceFrame(), 1, 0, 0);
      point = new FramePoint(ReferenceFrame.getWorldFrame(), 3, 0, 0);
      actual = GeometryTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      expected = 3.0;
      assertEquals("FAILED: Distance from point to plane", expected, actual, EPSILON);

      pointOnPlane = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector(pointOnPlane.getReferenceFrame(), 0, 1, 0);
      point = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 3, 0);
      actual = GeometryTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      expected = 3.0;
      assertEquals("FAILED: Distance from point to plane", expected, actual, EPSILON);

      pointOnPlane = new FramePoint(ReferenceFrame.getWorldFrame(), 1, 1, 1);
      planeNormal = new FrameVector(pointOnPlane.getReferenceFrame(), 0, 1, 0);
      point = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 3, 0);
      actual = GeometryTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      expected = 2.0;
      assertEquals("FAILED: Distance from point to plane", expected, actual, EPSILON);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetDistanceBetweenPointAndPlane2()
   {
      Random random = new Random(1176L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointOnPlane = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Vector3D planeNormal = RandomGeometry.nextVector3D(random, RandomNumbers.nextDouble(random, 0.0, 10.0));

         Vector3D parallelToPlane = RandomGeometry.nextOrthogonalVector3D(random, planeNormal, true);
         Point3D secondPointOnPlane = new Point3D();
         secondPointOnPlane.scaleAdd(RandomNumbers.nextDouble(random, 10.0), parallelToPlane, pointOnPlane);

         double expectedDistance = RandomNumbers.nextDouble(random, 0.0, 10.0);
         Point3D point = new Point3D();
         point.scaleAdd(expectedDistance / planeNormal.length(), planeNormal, secondPointOnPlane);

         double actualDistance = GeometryTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
         assertEquals(expectedDistance, actualDistance, Epsilons.ONE_TRILLIONTH);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsLineSegmentIntersectingPlane1()
   {
      FramePoint pointOnPlane = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      FrameVector planeNormal = new FrameVector(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      FramePoint lineStart = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, -1);
      FramePoint lineEnd = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 3);
      assertTrue(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, lineStart, lineEnd));

      pointOnPlane = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector(pointOnPlane.getReferenceFrame(), 1, 0, 0);
      lineStart = new FramePoint(ReferenceFrame.getWorldFrame(), -6, 3, -3);
      lineEnd = new FramePoint(ReferenceFrame.getWorldFrame(), 6, 3, 6);
      assertTrue(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, lineStart, lineEnd));

      pointOnPlane = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector(pointOnPlane.getReferenceFrame(), 0, 1, 0);
      lineStart = new FramePoint(ReferenceFrame.getWorldFrame(), 6, -3, -3);
      lineEnd = new FramePoint(ReferenceFrame.getWorldFrame(), 6, 3, 6);
      assertTrue(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, lineStart, lineEnd));

      pointOnPlane = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      lineStart = new FramePoint(ReferenceFrame.getWorldFrame(), 6, -3, 3);
      lineEnd = new FramePoint(ReferenceFrame.getWorldFrame(), 6, 3, 6);
      assertFalse(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, lineStart, lineEnd));

      pointOnPlane = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      lineStart = new FramePoint(ReferenceFrame.getWorldFrame(), 6, -3, -3);
      lineEnd = new FramePoint(ReferenceFrame.getWorldFrame(), 6, 3, -1);
      assertFalse(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, lineStart, lineEnd));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsLineSegmentIntersectingPlane2()
   {
      Random random = new Random(1176L);
      Point3D endPoint0 = new Point3D();
      Point3D endPoint1 = new Point3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointOnPlane = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Vector3D planeNormal = RandomGeometry.nextVector3D(random, RandomNumbers.nextDouble(random, 0.0, 10.0));

         Vector3D parallelToPlane = RandomGeometry.nextOrthogonalVector3D(random, planeNormal, true);
         Point3D randomLinePlaneIntersection = new Point3D();
         randomLinePlaneIntersection.scaleAdd(RandomNumbers.nextDouble(random, 10.0), parallelToPlane, pointOnPlane);

         Vector3D lineDirection = RandomGeometry.nextVector3D(random, 1.0);

         // Create the two endPoints on each side of the plane:
         endPoint0.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection, randomLinePlaneIntersection);
         endPoint1.scaleAdd(RandomNumbers.nextDouble(random, -10.0, 0.0), lineDirection, randomLinePlaneIntersection);
         assertTrue(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, endPoint0, endPoint1));
         assertTrue(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, endPoint1, endPoint0));

         // Create the two endPoints on one side of the plane:
         endPoint0.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection, randomLinePlaneIntersection);
         endPoint1.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection, randomLinePlaneIntersection);
         assertFalse(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, endPoint0, endPoint1));
         assertFalse(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, endPoint1, endPoint0));

         // Create the two endPoints on the other side of the plane:
         endPoint0.scaleAdd(RandomNumbers.nextDouble(random, -10.0, 0.0), lineDirection, randomLinePlaneIntersection);
         endPoint1.scaleAdd(RandomNumbers.nextDouble(random, -10.0, 0.0), lineDirection, randomLinePlaneIntersection);
         assertFalse(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, endPoint0, endPoint1));
         assertFalse(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, endPoint1, endPoint0));

         // Annoying case 1: endPoint0 == endPoint1 => should return false whether the endPoints are on plane or not.
         endPoint0.scaleAdd(RandomNumbers.nextDouble(random, -10.0, 0.0), lineDirection, randomLinePlaneIntersection);
         endPoint1.set(endPoint0);
         assertFalse(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, endPoint0, endPoint1));
         assertFalse(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, endPoint1, endPoint0));
         endPoint0.set(randomLinePlaneIntersection);
         endPoint1.set(endPoint0);
         assertFalse(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, endPoint0, endPoint1));
         assertFalse(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, endPoint1, endPoint0));
      }

      // Annoying case 2: one of the two endPoints is on the plane, should return false.
      // Tested separately as it is sensitive to numerical errors
      Point3D pointOnPlane = new Point3D();
      Vector3D planeNormal = new Vector3D(0.0, 0.0, 1.0);

      Point3D randomLinePlaneIntersection = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
      randomLinePlaneIntersection.setZ(0.0);

      Vector3D lineDirection = RandomGeometry.nextVector3D(random, 1.0);
      // Ensure that the line direction and the plane normal are somewhat pointing the same direction.
      if (lineDirection.dot(planeNormal) < 0.0)
         lineDirection.negate();

      endPoint0.set(randomLinePlaneIntersection);
      endPoint1.scaleAdd(RandomNumbers.nextDouble(random, -10.0, 0.0), lineDirection, randomLinePlaneIntersection);
      assertFalse(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, endPoint0, endPoint1));
      assertFalse(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, endPoint1, endPoint0));
      endPoint0.set(randomLinePlaneIntersection);
      endPoint1.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection, randomLinePlaneIntersection);
      assertFalse(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, endPoint0, endPoint1));
      assertFalse(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, endPoint1, endPoint0));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDoLineSegmentsIntersect1()
   {
      boolean intersect = GeometryTools.doLineSegmentsIntersect(new Point2D(-1.0, 0.0), new Point2D(1.0, 0.0), new Point2D(0.0, -1.0), new Point2D(0.0, 1.0));
      assertTrue(intersect);

      intersect = GeometryTools.doLineSegmentsIntersect(new Point2D(-1.0, 0.0), new Point2D(1.0, 0.0), new Point2D(0.0, 1.0), new Point2D(0.0, -1.0));
      assertTrue(intersect);

      intersect = GeometryTools.doLineSegmentsIntersect(new Point2D(-1.0, 0.0), new Point2D(1.0, 0.0), new Point2D(-1.0, 1.0), new Point2D(1.0, -1.0));
      assertTrue(intersect);

      intersect = GeometryTools.doLineSegmentsIntersect(new Point2D(-1.0, 0.0), new Point2D(1.0, 0.0), new Point2D(-1.0, -1.0), new Point2D(1.0, 1.0));
      assertTrue(intersect);

      intersect = GeometryTools.doLineSegmentsIntersect(new Point2D(-1.0, 0.0), new Point2D(1.0, 0.0), new Point2D(-1.0, -1.0), new Point2D(1.0, -1.0));
      assertFalse(intersect);

      intersect = GeometryTools.doLineSegmentsIntersect(new Point2D(-1.0, 0.0), new Point2D(1.0, 0.0), new Point2D(-1.0, 1.0), new Point2D(1.0, 1.0));
      assertFalse(intersect);

      intersect = GeometryTools.doLineSegmentsIntersect(new Point2D(-1.0, 0.0), new Point2D(1.0, 0.0), new Point2D(-1.0, 0.0), new Point2D(1.0, 0.0));
      assertTrue(intersect);

      intersect = GeometryTools.doLineSegmentsIntersect(new Point2D(-1.0, 0.0), new Point2D(1.0, 0.0), new Point2D(-1.1, 1.0), new Point2D(-1.1, -1.0));
      assertFalse(intersect);

      intersect = GeometryTools.doLineSegmentsIntersect(new Point2D(-1.0, 0.0), new Point2D(1.0, 0.0), new Point2D(-1.0, 1.0), new Point2D(-1.0, -1.0));
      assertTrue(intersect);

      intersect = GeometryTools.doLineSegmentsIntersect(new Point2D(-1.0, 0.0), new Point2D(1.0, 0.0), new Point2D(-1.0, 0.0), new Point2D(1.0, 0.0));
      assertTrue(intersect);

      intersect = GeometryTools.doLineSegmentsIntersect(new Point2D(-1.0, 0.0), new Point2D(1.0, 0.0), new Point2D(-1.0, 0.0), new Point2D(1.0, 0.0));
      assertTrue(intersect);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDoLineSegmentsIntersect2()
   {
      Random random = new Random(1176L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart1 = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D lineSegmentEnd1 = RandomGeometry.nextPoint2D(random, 10.0, 10.0);

         Point2D pointOnLineSegment1 = new Point2D();
         pointOnLineSegment1.interpolate(lineSegmentStart1, lineSegmentEnd1, RandomNumbers.nextDouble(random, 0.0, 1.0));

         Vector2D lineDirection2 = RandomGeometry.nextVector2D(random, 1.0);

         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();

         // Expecting intersection
         lineSegmentStart2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection2, pointOnLineSegment1);
         lineSegmentEnd2.scaleAdd(RandomNumbers.nextDouble(random, -10.0, 0.0), lineDirection2, pointOnLineSegment1);
         assertTrue(GeometryTools.doLineSegmentsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2));
         assertTrue(GeometryTools.doLineSegmentsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2));
         assertTrue(GeometryTools.doLineSegmentsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2));
         assertTrue(GeometryTools.doLineSegmentsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2));

         // Not expecting intersection
         lineSegmentStart2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection2, pointOnLineSegment1);
         lineSegmentEnd2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection2, pointOnLineSegment1);
         assertFalse(GeometryTools.doLineSegmentsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2));
         assertFalse(GeometryTools.doLineSegmentsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2));
         assertFalse(GeometryTools.doLineSegmentsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2));
         assertFalse(GeometryTools.doLineSegmentsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2));
      }

      // Test intersection at one of the end points
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart1 = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D lineSegmentEnd1 = RandomGeometry.nextPoint2D(random, 10.0, 10.0);

         Point2D pointOnLineSegment1 = new Point2D(lineSegmentStart1);

         Vector2D lineDirection2 = RandomGeometry.nextVector2D(random, 1.0);

         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();

         // Expecting intersection
         lineSegmentStart2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection2, pointOnLineSegment1);
         lineSegmentEnd2.scaleAdd(RandomNumbers.nextDouble(random, -10.0, 0.0), lineDirection2, pointOnLineSegment1);
         assertTrue(GeometryTools.doLineSegmentsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2));
         assertTrue(GeometryTools.doLineSegmentsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2));
         assertTrue(GeometryTools.doLineSegmentsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2));
         assertTrue(GeometryTools.doLineSegmentsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2));
      }

      // Test with parallel/collinear line segments
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart1 = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D lineSegmentEnd1 = RandomGeometry.nextPoint2D(random, 10.0, 10.0);

         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();

         double alpha1 = RandomNumbers.nextDouble(random, 2.0);
         double alpha2 = RandomNumbers.nextDouble(random, 2.0);

         // Make the second line segment collinear to the first one
         lineSegmentStart2.interpolate(lineSegmentStart1, lineSegmentEnd1, alpha1);
         lineSegmentEnd2.interpolate(lineSegmentStart1, lineSegmentEnd1, alpha2);

         if ((0.0 < alpha1 && alpha1 < 1.0) || (0.0 < alpha2 && alpha2 < 1.0) || alpha1 * alpha2 < 0.0)
         {
            assertTrue(GeometryTools.doLineSegmentsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2));
            assertTrue(GeometryTools.doLineSegmentsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2));
            assertTrue(GeometryTools.doLineSegmentsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2));
            assertTrue(GeometryTools.doLineSegmentsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2));
         }
         else
         {
            assertFalse(GeometryTools.doLineSegmentsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2));
            assertFalse(GeometryTools.doLineSegmentsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2));
            assertFalse(GeometryTools.doLineSegmentsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2));
            assertFalse(GeometryTools.doLineSegmentsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2));
         }

         // Shift the second line segment such that it becomes only parallel to the first.
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(lineSegmentEnd1, lineSegmentStart1);
         orthogonal.set(-orthogonal.getY(), orthogonal.getX());
         orthogonal.normalize();

         double distance = RandomNumbers.nextDouble(random, 1.0e-10, 10.0);
         lineSegmentStart2.scaleAdd(distance, orthogonal, lineSegmentStart2);
         lineSegmentEnd2.scaleAdd(distance, orthogonal, lineSegmentEnd2);
         assertFalse(GeometryTools.doLineSegmentsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2));
         assertFalse(GeometryTools.doLineSegmentsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2));
         assertFalse(GeometryTools.doLineSegmentsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2));
         assertFalse(GeometryTools.doLineSegmentsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetIntersectionBetweenLineSegmentAndPlane1()
   {
      FramePoint pointOnPlane = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      FrameVector planeNormal = new FrameVector(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      FramePoint lineStart = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 1);
      FramePoint lineEnd = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 3);

      //    FramePoint expectedReturn = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      FramePoint actualReturn = GeometryTools.getIntersectionBetweenLineSegmentAndPlane(pointOnPlane, planeNormal, lineStart, lineEnd);
      assertNull(actualReturn);
   }

   // What happens if to lines are the same line??????
   // Parallel lines returns something.....but not the right something

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetIntersectionBetweenTwoLines1()
   {
      Point2D point1 = new Point2D(5, 1.0);
      Vector2D vector1 = new Vector2D(8, 9);
      Point2D point2 = new Point2D(5, 1.0);
      Vector2D vector2 = new Vector2D(3, 9);
      Point2D expectedReturn = new Point2D(5.0, 1.0);
      Point2D actualReturn = GeometryTools.getIntersectionBetweenTwoLines(point1, vector1, point2, vector2);
      assertEquals("return value", expectedReturn, actualReturn);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetIntersectionBetweenTwoLines2()
   {
      double epsilon = Epsilons.ONE_TRILLIONTH;
      Random random = new Random(1176L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D pointOnLine1 = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Vector2D lineDirection1 = RandomGeometry.nextVector2D(random, 10.0);

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection1, pointOnLine1);

         Vector2D lineDirection2 = RandomGeometry.nextVector2D(random, 10.0);
         Point2D pointOnLine2 = new Point2D(expectedIntersection);

         if (Math.abs(lineDirection1.dot(lineDirection2) / lineDirection1.length() / lineDirection2.length()) > 1.0 - 0.0005)
            epsilon = Epsilons.ONE_HUNDRED_BILLIONTH; // Loss of precision for small angles between the two lines.
         else
            epsilon = Epsilons.ONE_TRILLIONTH;
         Point2D actualIntersection = GeometryTools.getIntersectionBetweenTwoLines(pointOnLine1, lineDirection1, pointOnLine2, lineDirection2);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         pointOnLine2.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection2, pointOnLine2);
         actualIntersection = GeometryTools.getIntersectionBetweenTwoLines(pointOnLine1, lineDirection1, pointOnLine2, lineDirection2);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      }

      // Test when parallel but not collinear
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D pointOnLine1 = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Vector2D lineDirection1 = RandomGeometry.nextVector2D(random, 10.0);

         Vector2D lineDirection2 = new Vector2D(lineDirection1);
         if (random.nextBoolean())
            lineDirection2.negate();
         Point2D pointOnLine2 = new Point2D(pointOnLine1);

         Vector2D orthogonal = new Vector2D(-lineDirection1.getY(), lineDirection1.getX());

         pointOnLine2.scaleAdd(RandomNumbers.nextDouble(random, 10.0), orthogonal, pointOnLine2);
         pointOnLine2.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection2, pointOnLine2);
         Point2D actualIntersection = GeometryTools.getIntersectionBetweenTwoLines(pointOnLine1, lineDirection1, pointOnLine2, lineDirection2);
         assertNull(actualIntersection);
      }

      // Test when collinear
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D pointOnLine1 = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Vector2D lineDirection1 = RandomGeometry.nextVector2D(random, 10.0);

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.set(pointOnLine1);

         Vector2D lineDirection2 = new Vector2D(lineDirection1);
         Point2D pointOnLine2 = new Point2D(expectedIntersection);

         Point2D actualIntersection = GeometryTools.getIntersectionBetweenTwoLines(pointOnLine1, lineDirection1, pointOnLine2, lineDirection2);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         pointOnLine2.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection2, pointOnLine2);
         actualIntersection = GeometryTools.getIntersectionBetweenTwoLines(pointOnLine1, lineDirection1, pointOnLine2, lineDirection2);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetIntersectionBetweenTwoLineSegments1()
   {
      Point2D lineSegmentStart1, lineSegmentEnd1;
      Point2D lineSegmentStart2, lineSegmentEnd2;

      lineSegmentStart1 = new Point2D(0.0, -0.075);
      lineSegmentEnd1 = new Point2D(-1.6165337748745066E-16, -2.7150000000000007);
      lineSegmentStart2 = new Point2D(0.0, 0.075);
      lineSegmentEnd2 = new Point2D(0.0, 0.325);

      assertFalse(GeometryTools.doLineSegmentsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2));
      assertOnlyExistenceOfTwoLineSegmentsIntersectionAllCombinations(false, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetIntersectionBetweenTwoLineSegments2()
   {
      Random random = new Random(3242L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart1 = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D lineSegmentEnd1 = RandomGeometry.nextPoint2D(random, 10.0, 10.0);

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.interpolate(lineSegmentStart1, lineSegmentEnd1, RandomNumbers.nextDouble(random, 0.0, 1.0));

         Vector2D lineDirection2 = RandomGeometry.nextVector2D(random, 1.0);

         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();

         // Expecting intersection
         lineSegmentStart2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection2, expectedIntersection);
         lineSegmentEnd2.scaleAdd(RandomNumbers.nextDouble(random, -10.0, 0.0), lineDirection2, expectedIntersection);
         assertAllCombinationsOfTwoLineSegmentsIntersection(expectedIntersection, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);

         // Not expecting intersection
         lineSegmentStart2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection2, expectedIntersection);
         lineSegmentEnd2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection2, expectedIntersection);
         assertOnlyExistenceOfTwoLineSegmentsIntersectionAllCombinations(false, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      }

      // Test intersection at one of the end points
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart1 = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D lineSegmentEnd1 = RandomGeometry.nextPoint2D(random, 10.0, 10.0);

         Point2D expectedIntersection = new Point2D(lineSegmentStart1);

         Vector2D lineDirection2 = RandomGeometry.nextVector2D(random, 1.0);

         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();

         // Not expecting intersection
         lineSegmentStart2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection2, expectedIntersection);
         lineSegmentEnd2.scaleAdd(RandomNumbers.nextDouble(random, -10.0, 0.0), lineDirection2, expectedIntersection);
         assertAllCombinationsOfTwoLineSegmentsIntersection(expectedIntersection, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      }

      // Test with parallel/collinear line segments
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart1 = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D lineSegmentEnd1 = RandomGeometry.nextPoint2D(random, 10.0, 10.0);

         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();

         double alpha1 = RandomNumbers.nextDouble(random, 2.0);
         double alpha2 = RandomNumbers.nextDouble(random, 2.0);

         // Make the second line segment collinear to the first one
         lineSegmentStart2.interpolate(lineSegmentStart1, lineSegmentEnd1, alpha1);
         lineSegmentEnd2.interpolate(lineSegmentStart1, lineSegmentEnd1, alpha2);

         if ((0.0 < alpha1 && alpha1 < 1.0) || (0.0 < alpha2 && alpha2 < 1.0) || alpha1 * alpha2 < 0.0)
         {
            assertOnlyExistenceOfTwoLineSegmentsIntersectionAllCombinations(true, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
         }
         else
         {
            assertOnlyExistenceOfTwoLineSegmentsIntersectionAllCombinations(false, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
         }

         // Shift the second line segment such that it becomes only parallel to the first.
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(lineSegmentEnd1, lineSegmentStart1);
         orthogonal.set(-orthogonal.getY(), orthogonal.getX());
         orthogonal.normalize();

         double distance = RandomNumbers.nextDouble(random, 1.0e-10, 10.0);
         lineSegmentStart2.scaleAdd(distance, orthogonal, lineSegmentStart2);
         lineSegmentEnd2.scaleAdd(distance, orthogonal, lineSegmentEnd2);
         assertOnlyExistenceOfTwoLineSegmentsIntersectionAllCombinations(false, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      }
   }

   private void assertOnlyExistenceOfTwoLineSegmentsIntersectionAllCombinations(boolean intersectionExist, Point2D lineSegmentStart1, Point2D lineSegmentEnd1,
                                                                                Point2D lineSegmentStart2, Point2D lineSegmentEnd2)
   {
      boolean success;
      Point2D actualIntersection = new Point2D();

      success = GeometryTools.getIntersectionBetweenTwoLineSegments(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2, actualIntersection);
      assertTrue(success == intersectionExist);
      success = GeometryTools.getIntersectionBetweenTwoLineSegments(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2, actualIntersection);
      assertTrue(success == intersectionExist);
      success = GeometryTools.getIntersectionBetweenTwoLineSegments(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2, actualIntersection);
      assertTrue(success == intersectionExist);
      success = GeometryTools.getIntersectionBetweenTwoLineSegments(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2, actualIntersection);
      assertTrue(success == intersectionExist);

      success = GeometryTools.getIntersectionBetweenTwoLineSegments(lineSegmentStart2, lineSegmentEnd2, lineSegmentStart1, lineSegmentEnd1, actualIntersection);
      assertTrue(success == intersectionExist);
      success = GeometryTools.getIntersectionBetweenTwoLineSegments(lineSegmentStart2, lineSegmentEnd2, lineSegmentEnd1, lineSegmentStart1, actualIntersection);
      assertTrue(success == intersectionExist);
      success = GeometryTools.getIntersectionBetweenTwoLineSegments(lineSegmentEnd2, lineSegmentStart2, lineSegmentStart1, lineSegmentEnd1, actualIntersection);
      assertTrue(success == intersectionExist);
      success = GeometryTools.getIntersectionBetweenTwoLineSegments(lineSegmentEnd2, lineSegmentStart2, lineSegmentEnd1, lineSegmentStart1, actualIntersection);
      assertTrue(success == intersectionExist);
   }

   private void assertAllCombinationsOfTwoLineSegmentsIntersection(Point2D expectedIntersection, Point2D lineSegmentStart1, Point2D lineSegmentEnd1,
                                                                   Point2D lineSegmentStart2, Point2D lineSegmentEnd2)
   {
      double epsilon = Epsilons.ONE_TRILLIONTH;

      Vector2D direction1 = new Vector2D();
      direction1.sub(lineSegmentEnd1, lineSegmentStart1);
      Vector2D direction2 = new Vector2D();
      direction2.sub(lineSegmentEnd2, lineSegmentStart2);

      if (Math.abs(direction1.dot(direction2)) > 1.0 - 0.0001)
         epsilon = Epsilons.ONE_TEN_BILLIONTH;

      boolean success;
      Point2D actualIntersection = new Point2D();

      success = GeometryTools.getIntersectionBetweenTwoLineSegments(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2, actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      success = GeometryTools.getIntersectionBetweenTwoLineSegments(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2, actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      success = GeometryTools.getIntersectionBetweenTwoLineSegments(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2, actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      success = GeometryTools.getIntersectionBetweenTwoLineSegments(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2, actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

      success = GeometryTools.getIntersectionBetweenTwoLineSegments(lineSegmentStart2, lineSegmentEnd2, lineSegmentStart1, lineSegmentEnd1, actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      success = GeometryTools.getIntersectionBetweenTwoLineSegments(lineSegmentStart2, lineSegmentEnd2, lineSegmentEnd1, lineSegmentStart1, actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      success = GeometryTools.getIntersectionBetweenTwoLineSegments(lineSegmentEnd2, lineSegmentStart2, lineSegmentStart1, lineSegmentEnd1, actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      success = GeometryTools.getIntersectionBetweenTwoLineSegments(lineSegmentEnd2, lineSegmentStart2, lineSegmentEnd1, lineSegmentStart1, actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetIntersectionBetweenLineAndLineSegment()
   {
      double epsilon = Epsilons.ONE_TRILLIONTH;
      Random random = new Random(23423L);
      Point2D actualIntersection = new Point2D();
      boolean success;

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D lineSegmentEnd = RandomGeometry.nextPoint2D(random, 10.0, 10.0);

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.interpolate(lineSegmentStart, lineSegmentEnd, RandomNumbers.nextDouble(random, 0.0, 1.0));

         Point2D pointOnLine = new Point2D(expectedIntersection);
         Vector2D lineDirection = RandomGeometry.nextVector2D(random, 1.0);

         // Expecting intersection
         success = GeometryTools.getIntersectionBetweenLineAndLineSegment(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         success = GeometryTools.getIntersectionBetweenLineAndLineSegment(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         pointOnLine.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         success = GeometryTools.getIntersectionBetweenLineAndLineSegment(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         success = GeometryTools.getIntersectionBetweenLineAndLineSegment(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      }

      // Make the intersection happen outside the line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D lineSegmentEnd = RandomGeometry.nextPoint2D(random, 10.0, 10.0);

         Point2D pointOnLine = new Point2D();
         Vector2D lineDirection = RandomGeometry.nextVector2D(random, 1.0);

         Point2D lineLineIntersection = new Point2D();
         lineLineIntersection.interpolate(lineSegmentStart, lineSegmentEnd, RandomNumbers.nextDouble(random, 1.0, 2.0));
         pointOnLine.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection, lineLineIntersection);
         success = GeometryTools.getIntersectionBetweenLineAndLineSegment(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd, actualIntersection);
         assertFalse(success);

         success = GeometryTools.getIntersectionBetweenLineAndLineSegment(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart, actualIntersection);
         assertFalse(success);

         lineLineIntersection.interpolate(lineSegmentStart, lineSegmentEnd, RandomNumbers.nextDouble(random, -1.0, 0.0));
         pointOnLine.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection, lineLineIntersection);
         success = GeometryTools.getIntersectionBetweenLineAndLineSegment(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd, actualIntersection);
         assertFalse(success);

         success = GeometryTools.getIntersectionBetweenLineAndLineSegment(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart, actualIntersection);
         assertFalse(success);
      }

      // Make the intersection happen on each end point of the line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D lineSegmentEnd = RandomGeometry.nextPoint2D(random, 10.0, 10.0);

         Point2D pointOnLine = new Point2D();
         Vector2D lineDirection = RandomGeometry.nextVector2D(random, 1.0);

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.set(lineSegmentStart);
         pointOnLine.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         success = GeometryTools.getIntersectionBetweenLineAndLineSegment(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         success = GeometryTools.getIntersectionBetweenLineAndLineSegment(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         expectedIntersection.set(lineSegmentEnd);
         pointOnLine.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         success = GeometryTools.getIntersectionBetweenLineAndLineSegment(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         success = GeometryTools.getIntersectionBetweenLineAndLineSegment(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      }

      // Make the line segment and the line parallel not collinear.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D lineSegmentEnd = RandomGeometry.nextPoint2D(random, 10.0, 10.0);

         Point2D pointOnLine = new Point2D(lineSegmentStart);
         Vector2D lineDirection = new Vector2D();
         lineDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineDirection.normalize();
         if (random.nextBoolean())
            lineDirection.negate();

         Vector2D orthogonal = new Vector2D(-lineDirection.getY(), lineDirection.getY());

         pointOnLine.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), orthogonal, pointOnLine);
         pointOnLine.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection, pointOnLine);
         success = GeometryTools.getIntersectionBetweenLineAndLineSegment(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd, actualIntersection);
         assertFalse(success);
      }

      // Make the line segment and the line collinear.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D lineSegmentEnd = RandomGeometry.nextPoint2D(random, 10.0, 10.0);

         Point2D pointOnLine = new Point2D(lineSegmentStart);
         Vector2D lineDirection = new Vector2D();
         lineDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineDirection.normalize();
         if (random.nextBoolean())
            lineDirection.negate();

         pointOnLine.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection, pointOnLine);
         success = GeometryTools.getIntersectionBetweenLineAndLineSegment(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(lineSegmentStart, actualIntersection, epsilon);
         success = GeometryTools.getIntersectionBetweenLineAndLineSegment(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(lineSegmentEnd, actualIntersection, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetPerpendicularBisector1()
   {
      Point2D lineStart = new Point2D(1, 1);
      Point2D lineEnd = new Point2D(5, 5);
      Point2D bisectorStart = new Point2D(2, 1);
      Vector2D bisectorDirection = new Vector2D();
      GeometryTools.getPerpendicularBisector(lineStart, lineEnd, bisectorStart, bisectorDirection);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetPerpendicularBisector2()
   {
      Random random = new Random(1176L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D lineSegmentEnd = RandomGeometry.nextPoint2D(random, 10.0, 10.0);

         Point2D expectedBisectorStart = new Point2D();
         expectedBisectorStart.interpolate(lineSegmentStart, lineSegmentEnd, 0.5);
         Vector2D expectedBisectorDirection = new Vector2D();
         expectedBisectorDirection.sub(lineSegmentEnd, lineSegmentStart);
         GeometryTools.getPerpendicularVector(expectedBisectorDirection, expectedBisectorDirection);
         expectedBisectorDirection.normalize();

         Point2D actualBisectorStart = new Point2D();
         Vector2D actualBisectorDirection = new Vector2D();
         GeometryTools.getPerpendicularBisector(lineSegmentStart, lineSegmentEnd, actualBisectorStart, actualBisectorDirection);
         EuclidCoreTestTools.assertTuple2DEquals(expectedBisectorStart, actualBisectorStart, Epsilons.ONE_TRILLIONTH);
         EuclidCoreTestTools.assertTuple2DEquals(expectedBisectorDirection, actualBisectorDirection, Epsilons.ONE_TRILLIONTH);

         Point2D pointOnBisector = new Point2D();
         pointOnBisector.scaleAdd(1.0, actualBisectorDirection, actualBisectorStart);
         assertTrue(GeometryTools.isPointOnLeftSideOfLine(pointOnBisector, lineSegmentStart, lineSegmentEnd));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetPerpendicularVector()
   {
      Vector2D vector = new Vector2D(15.0, 10.0);
      Vector2D expectedReturn = new Vector2D(-10.0, 15.0);
      Vector2D actualReturn = GeometryTools.getPerpendicularVector(vector);
      assertEquals("return value", expectedReturn, actualReturn);
      Random random = new Random(1176L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         vector = RandomGeometry.nextVector2D(random, RandomNumbers.nextDouble(random, 0.0, 10.0));
         Vector2D perpendicularVector = GeometryTools.getPerpendicularVector(vector);
         assertEquals(vector.length(), perpendicularVector.length(), Epsilons.ONE_TRILLIONTH);
         assertEquals(vector.length() * vector.length(), vector.cross(perpendicularVector), Epsilons.ONE_TRILLIONTH);
         assertEquals(0.0, vector.dot(perpendicularVector), Epsilons.ONE_TRILLIONTH);
         assertEquals(Math.PI / 2.0, vector.angle(perpendicularVector), Epsilons.ONE_TRILLIONTH);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetPerpendicularVectorFromLineToPoint1()
   {
      FramePoint point0 = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      FramePoint lineStart0 = new FramePoint(ReferenceFrame.getWorldFrame(), -10, 10, 0);
      FramePoint lineEnd0 = new FramePoint(ReferenceFrame.getWorldFrame(), 10, 10, 0);
      FramePoint intersectionPoint0 = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 10, 0);
      FrameVector x0 = new FrameVector(point0.getReferenceFrame());
      x0.sub(point0, intersectionPoint0);
      FrameVector expectedReturn0 = x0;
      FrameVector actualReturn0 = GeometryTools.getPerpendicularVectorFromLineToPoint(point0, lineStart0, lineEnd0, intersectionPoint0);

      assertTrue("Test Failed", expectedReturn0.epsilonEquals(actualReturn0, EPSILON));

      FramePoint point = new FramePoint(ReferenceFrame.getWorldFrame(), 4, 2, 0);
      FramePoint lineStart = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      FramePoint lineEnd = new FramePoint(ReferenceFrame.getWorldFrame(), 10, 10, 0);
      FramePoint intersectionPoint = new FramePoint(ReferenceFrame.getWorldFrame(), 3, 3, 0);
      FrameVector x = new FrameVector(point.getReferenceFrame());
      x.sub(point, intersectionPoint);
      FrameVector expectedReturn = x;
      FrameVector actualReturn = GeometryTools.getPerpendicularVectorFromLineToPoint(point, lineStart, lineEnd, intersectionPoint);
      assertTrue("Test Failed", expectedReturn.epsilonEquals(actualReturn, EPSILON));

      FramePoint point1 = new FramePoint(ReferenceFrame.getWorldFrame(), -2.5, 1.5, 0);
      FramePoint lineStart1 = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      FramePoint lineEnd1 = new FramePoint(ReferenceFrame.getWorldFrame(), -4, 4, 0);
      FramePoint intersectionPoint1 = new FramePoint(ReferenceFrame.getWorldFrame(), -2, 2, 0);

      GeometryTools.getClosestPointToLineSegment(new Point2D(-2.5, 1.5), new Point2D(0, 0), new Point2D(-4, 4));
      FrameVector x1 = new FrameVector(point1.getReferenceFrame());
      x1.sub(point1, intersectionPoint1);
      FrameVector expectedReturn1 = x1;
      FrameVector actualReturn1 = GeometryTools.getPerpendicularVectorFromLineToPoint(point1, lineStart1, lineEnd1, intersectionPoint1);

      assertTrue("Test Failed", expectedReturn1.epsilonEquals(actualReturn1, EPSILON));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetPerpendicularVectorFromLineToPoint2()
   {
      Random random = new Random(1176L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D expectedPerpendicularVector = RandomGeometry.nextVector3D(random, RandomNumbers.nextDouble(random, 0.0, 10.0));
         Point3D expectedIntersection = RandomGeometry.nextPoint3D(random, -10.0, 10.0);

         Vector3D lineDirection = RandomGeometry.nextOrthogonalVector3D(random, expectedPerpendicularVector, true);
         Point3D firstPointOnLine = new Point3D();
         firstPointOnLine.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection, expectedIntersection);
         Point3D secondPointOnLine = new Point3D();
         secondPointOnLine.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection, expectedIntersection);

         Point3D point = new Point3D();
         point.add(expectedIntersection, expectedPerpendicularVector);

         Point3D actualIntersection = new Point3D();
         double epsilon = Epsilons.ONE_TRILLIONTH;

         if (firstPointOnLine.distance(secondPointOnLine) < 5.0e-4)
            epsilon = Epsilons.ONE_TEN_BILLIONTH; // Loss of precision when the given points defining the line are getting close.

         Vector3D actualPerpendicularVector = GeometryTools.getPerpendicularVectorFromLineToPoint(point, firstPointOnLine, secondPointOnLine,
                                                                                                  actualIntersection);
         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection, actualIntersection, epsilon);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPerpendicularVector, actualPerpendicularVector, epsilon);

         actualPerpendicularVector = GeometryTools.getPerpendicularVectorFromLineToPoint(point, firstPointOnLine, secondPointOnLine, null);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPerpendicularVector, actualPerpendicularVector, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetPlaneNormalGivenThreePoints()
   {
      FramePoint point1 = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 5, 0);
      FramePoint point2 = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 5, 0);
      FramePoint point3 = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 5, 0);
      FrameVector expectedReturn = null;
      FrameVector actualReturn = GeometryTools.getPlaneNormalGivenThreePoints(point1, point2, point3);
      assertEquals("test failed", expectedReturn, actualReturn);

      FramePoint point91 = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 3, 0);
      FramePoint point92 = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 5, 0);
      FramePoint point93 = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 1, 0);
      FrameVector expectedReturn9 = null;
      FrameVector actualReturn9 = GeometryTools.getPlaneNormalGivenThreePoints(point91, point92, point93);
      assertEquals("test failed", expectedReturn9, actualReturn9);

      FramePoint point81 = new FramePoint(ReferenceFrame.getWorldFrame(), 9, 0, 0);
      FramePoint point82 = new FramePoint(ReferenceFrame.getWorldFrame(), 7, 0, 0);
      FramePoint point83 = new FramePoint(ReferenceFrame.getWorldFrame(), 4, 0, 0);
      FrameVector expectedReturn8 = null;
      FrameVector actualReturn8 = GeometryTools.getPlaneNormalGivenThreePoints(point81, point82, point83);
      assertEquals("test failed", expectedReturn8, actualReturn8);

      FramePoint point71 = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 4);
      FramePoint point72 = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 6);
      FramePoint point73 = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 7);
      FrameVector expectedReturn7 = null;
      FrameVector actualReturn7 = GeometryTools.getPlaneNormalGivenThreePoints(point71, point72, point73);
      assertEquals("test failed", expectedReturn7, actualReturn7);

      FramePoint point11 = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 5, 46);
      FramePoint point12 = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 587, 3);
      FramePoint point13 = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 18, 8);
      FramePoint p1 = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 5, 5);
      FramePoint v1 = new FramePoint(ReferenceFrame.getWorldFrame(), 1, 5, 5);
      FrameVector expectedReturn1 = new FrameVector(p1.getReferenceFrame());
      expectedReturn1.sub(p1, v1);
      FrameVector actualReturn1 = GeometryTools.getPlaneNormalGivenThreePoints(point11, point12, point13);
      assertTrue("Test Failed", expectedReturn1.epsilonEquals(actualReturn1, EPSILON));

      FramePoint point21 = new FramePoint(ReferenceFrame.getWorldFrame(), 65, 0, 46);
      FramePoint point22 = new FramePoint(ReferenceFrame.getWorldFrame(), 43, 0, 3);
      FramePoint point23 = new FramePoint(ReferenceFrame.getWorldFrame(), 13, 0, 8);
      FramePoint p2 = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 1, 5);
      FramePoint v2 = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 5);
      FrameVector expectedReturn2 = new FrameVector(p2.getReferenceFrame());
      expectedReturn2.sub(p2, v2);
      FrameVector actualReturn2 = GeometryTools.getPlaneNormalGivenThreePoints(point21, point22, point23);
      assertTrue("Test Failed", expectedReturn2.epsilonEquals(actualReturn2, EPSILON));

      FramePoint point31 = new FramePoint(ReferenceFrame.getWorldFrame(), 65, 56, 0);
      FramePoint point32 = new FramePoint(ReferenceFrame.getWorldFrame(), 43, 3, 0);
      FramePoint point33 = new FramePoint(ReferenceFrame.getWorldFrame(), 13, 87, 0);
      FramePoint p3 = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 55, 0);
      FramePoint v3 = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 55, 1);
      FrameVector expectedReturn3 = new FrameVector(p3.getReferenceFrame());
      expectedReturn3.sub(p3, v3);
      FrameVector actualReturn3 = GeometryTools.getPlaneNormalGivenThreePoints(point31, point32, point33);
      assertTrue("Test Failed", expectedReturn3.epsilonEquals(actualReturn3, EPSILON));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetPlaneNormalGivenThreePoints1()
   {
      Point3D point1 = new Point3D(0, 0, 0);
      Point3D point2 = new Point3D(7, 0, 0);
      Point3D point3 = new Point3D(2, 0, 0);
      Vector3D expectedReturn = null;
      Vector3D actualReturn = GeometryTools.getPlaneNormalGivenThreePoints(point1, point2, point3);
      assertEquals("return value", expectedReturn, actualReturn);

      Point3D point01 = new Point3D(15, 0, 0);
      Point3D point02 = new Point3D(15, 0, 0);
      Point3D point03 = new Point3D(15, 0, 0);
      Vector3D expectedReturn1 = null;
      Vector3D actualReturn1 = GeometryTools.getPlaneNormalGivenThreePoints(point01, point02, point03);
      assertEquals("return value", expectedReturn1, actualReturn1);

      Point3D point11 = new Point3D(0, 4, 0);
      Point3D point12 = new Point3D(0, 2, 0);
      Point3D point13 = new Point3D(0, 67, 0);
      Vector3D expectedReturn2 = null;
      Vector3D actualReturn2 = GeometryTools.getPlaneNormalGivenThreePoints(point11, point12, point13);
      assertEquals("return value", expectedReturn2, actualReturn2);

      Point3D point21 = new Point3D(0, 0, 4);
      Point3D point22 = new Point3D(0, 0, 7);
      Point3D point23 = new Point3D(0, 0, 5);
      Vector3D expectedReturn3 = null;
      Vector3D actualReturn3 = GeometryTools.getPlaneNormalGivenThreePoints(point21, point22, point23);
      assertEquals("return value", expectedReturn3, actualReturn3);

      Point3D point31 = new Point3D(0, 67, 5);
      Point3D point32 = new Point3D(0, 3, 7);
      Point3D point33 = new Point3D(0, 90, 7.24264068712);
      Vector3D expectedReturn4 = new Vector3D(-1, 0, 0);
      Vector3D actualReturn4 = GeometryTools.getPlaneNormalGivenThreePoints(point31, point32, point33);
      assertEquals("return value", expectedReturn4, actualReturn4);

      Point3D point41 = new Point3D(45, 0, 5);
      Point3D point42 = new Point3D(35, 0, 7);
      Point3D point43 = new Point3D(132, 0, 7.24264068712);
      Vector3D expectedReturn5 = new Vector3D(0, 1, 0);
      Vector3D actualReturn5 = GeometryTools.getPlaneNormalGivenThreePoints(point41, point42, point43);
      assertTrue("Test Failed", expectedReturn5.epsilonEquals(actualReturn5, EPSILON));

      Point3D point51 = new Point3D(45, 67, 0);
      Point3D point52 = new Point3D(35, 56, 0);
      Point3D point53 = new Point3D(132, -4, 0);
      Vector3D expectedReturn6 = new Vector3D(0, 0, 1);
      Vector3D actualReturn6 = GeometryTools.getPlaneNormalGivenThreePoints(point51, point52, point53);
      assertTrue("Test Failed", expectedReturn6.epsilonEquals(actualReturn6, EPSILON));

      Point3D point61 = new Point3D(1, 5, 7);
      Point3D point62 = new Point3D(1, 5, 7);
      Point3D point63 = new Point3D(5, 12, 4325);
      Vector3D expectedReturn7 = null;
      Vector3D actualReturn7 = GeometryTools.getPlaneNormalGivenThreePoints(point61, point62, point63);
      assertEquals("return value", expectedReturn7, actualReturn7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetPlaneNormalGivenThreePoints2()
   {
      Random random = new Random(1176L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D expectedPlaneNormal = RandomGeometry.nextVector3D(random, 1.0);

         Point3D firstPointOnPlane = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Point3D secondPointOnPlane = new Point3D();
         Point3D thirdPointOnPlane = new Point3D();

         Vector3D secondOrthogonalToNormal = RandomGeometry.nextOrthogonalVector3D(random, expectedPlaneNormal, true);
         Vector3D thirdOrthogonalToNormal = RandomGeometry.nextOrthogonalVector3D(random, expectedPlaneNormal, true);

         secondPointOnPlane.scaleAdd(RandomNumbers.nextDouble(random, 1.0, 10.0), secondOrthogonalToNormal, firstPointOnPlane);
         thirdPointOnPlane.scaleAdd(RandomNumbers.nextDouble(random, 1.0, 10.0), thirdOrthogonalToNormal, firstPointOnPlane);

         Vector3D actualPlaneNormal = GeometryTools.getPlaneNormalGivenThreePoints(firstPointOnPlane, secondPointOnPlane, thirdPointOnPlane);

         if (expectedPlaneNormal.dot(actualPlaneNormal) < 0.0)
            actualPlaneNormal.negate();

         EuclidCoreTestTools.assertTuple3DEquals(expectedPlaneNormal, actualPlaneNormal, Epsilons.ONE_TRILLIONTH);
      }
   }

   /*
    * public void testGetTransform() { FramePoint point = null; FrameVector
    * normal = null; Orientation expectedReturn = null; Orientation actualReturn
    * = geometryTools.getTransform(point, normal); assertEquals("return value",
    * expectedReturn, actualReturn); } public void
    * testGetVerticalSpansOfPoints() { double xMin = 0.0; double yMin = 0.0;
    * double zMin = 0.0; double xMax = 0.0; double yMax = 0.0; double zMax =
    * 0.0; double xResolution = 0.0; double yResolution = 0.0; double
    * zResolution = 0.0; ArrayList expectedReturn = null; ArrayList actualReturn
    * = geometryTools.getVerticalSpansOfPoints(xMin, yMin, zMin, xMax, yMax,
    * zMax, xResolution, yResolution, zResolution); assertEquals("return value",
    * expectedReturn, actualReturn); }
    */

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetTopVertexOfIsoscelesTriangle1()
   {
      ReferenceFrame frame = ReferenceFrame.getWorldFrame();

      FramePoint baseVertexA = new FramePoint(frame);
      FramePoint baseVertexC = new FramePoint(frame);
      FramePoint topVertexB = new FramePoint(frame);
      FrameVector trianglePlaneNormal = new FrameVector(frame);

      FramePoint topVertexBComputed = new FramePoint(frame);

      double legLength = 1.5;
      double topVertexAngle = Math.toRadians(1.0);

      while (Math.toDegrees(topVertexAngle) < 179.0)
      {
         trianglePlaneNormal.setIncludingFrame(frame, 0.0, 0.0, 1.0);

         topVertexB.setIncludingFrame(frame, 0.0, 0.0, 0.0);
         baseVertexA.setIncludingFrame(frame, legLength * Math.cos(-0.5 * topVertexAngle), legLength * Math.sin(-0.5 * topVertexAngle), 0.0);
         baseVertexC.setIncludingFrame(frame, legLength * Math.cos(0.5 * topVertexAngle), legLength * Math.sin(0.5 * topVertexAngle), 0.0);

         assertTrue("TopVertexAngle = " + Math.toDegrees(topVertexAngle) + " degrees",
                    GeometryTools.isFormingTriangle(baseVertexA.distance(baseVertexC), baseVertexA.distance(topVertexB), topVertexB.distance(baseVertexC)));

         GeometryTools.getTopVertexOfIsoscelesTriangle(baseVertexA, baseVertexC, trianglePlaneNormal, topVertexAngle, topVertexBComputed);

         String errorMsg = "Computed vertex: " + topVertexBComputed + "\n does not match actual vertex: " + topVertexB + "\n when topVertex Angle = "
               + Math.toDegrees(topVertexAngle) + " degrees \n";
         assertEquals(errorMsg, 0.0, topVertexB.distance(topVertexBComputed), 1e-9);

         topVertexAngle += Math.toRadians(1.0);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetTopVertexOfIsoscelesTriangle2()
   {
      Random random = new Random(1176L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D expectedB = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Point3D a = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Vector3D ba = new Vector3D();
         ba.sub(a, expectedB);

         double abcAngle = RandomNumbers.nextDouble(random, 0.0, Math.PI / 2.0);

         Vector3D triangleNormal = RandomGeometry.nextOrthogonalVector3D(random, ba, true);
         triangleNormal.scale(RandomNumbers.nextDouble(random, 0.0, 10.0));
         AxisAngle abcAxisAngle = new AxisAngle(triangleNormal, abcAngle);
         RotationMatrix abcRotationMatrix = new RotationMatrix();
         abcRotationMatrix.set(abcAxisAngle);
         Vector3D bc = new Vector3D();
         abcRotationMatrix.transform(ba, bc);

         Point3D c = new Point3D();
         c.add(bc, expectedB);

         double epsilon = Epsilons.ONE_TEN_BILLIONTH;

         Point3D actualB = new Point3D();
         GeometryTools.getTopVertexOfIsoscelesTriangle(a, c, triangleNormal, abcAngle, actualB);
         EuclidCoreTestTools.assertTuple3DEquals(expectedB, actualB, epsilon);
         assertEquals(abcAngle, ba.angle(bc), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsPointOnLeftSideOfLine()
   {
      FramePoint lineStart = new FramePoint(ReferenceFrame.getWorldFrame(), 5.0, 0.0, 0.0);
      FramePoint lineEnd = new FramePoint(ReferenceFrame.getWorldFrame(), 5.0, 10.0, 0.0);

      FramePoint point = new FramePoint(ReferenceFrame.getWorldFrame(), 10, 5, 0.0);

      boolean expectedReturn = false;
      boolean actualReturn = GeometryTools.isPointOnLeftSideOfLine(point, lineStart, lineEnd);
      assertEquals("return value", expectedReturn, actualReturn);

      /** @todo fill in the test code */
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsPointOnLeftSideOfLine1()
   {
      Point2D point = new Point2D(3, 9);
      Point2D lineStart = new Point2D(-5, 8);
      Point2D lineEnd = new Point2D(10, 7);
      boolean expectedReturn = true;
      boolean actualReturn = GeometryTools.isPointOnLeftSideOfLine(point, lineStart, lineEnd);
      assertEquals("return value", expectedReturn, actualReturn);

      Point2D point2 = new Point2D(1, 5);
      Point2D lineStart2 = new Point2D(-5, 8);
      Point2D lineEnd2 = new Point2D(10, 7);
      boolean expectedReturn2 = false;
      boolean actualReturn2 = GeometryTools.isPointOnLeftSideOfLine(point2, lineStart2, lineEnd2);
      assertEquals("return value", expectedReturn2, actualReturn2);

      Point2D point3 = new Point2D(1, 1);
      Point2D lineStart3 = new Point2D(0, 0);
      Point2D lineEnd3 = new Point2D(10, 10);
      boolean expectedReturn3 = false;
      boolean actualReturn3 = GeometryTools.isPointOnLeftSideOfLine(point3, lineStart3, lineEnd3);
      assertEquals("return value", expectedReturn3, actualReturn3);

      Point2D point4 = new Point2D(3, 9);
      Point2D lineStart4 = new Point2D(10, 7);
      Point2D lineEnd4 = new Point2D(-5, 8);
      boolean expectedReturn4 = false;
      boolean actualReturn4 = GeometryTools.isPointOnLeftSideOfLine(point4, lineStart4, lineEnd4);
      assertEquals("return value", expectedReturn4, actualReturn4);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public void testAngleByLawOfCosineWithNegativeSideLengthA()
   {
      double a = 1.0;
      GeometryTools.getUnknownTriangleAngleByLawOfCosine(-a, a, a);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public void testAngleByLawOfCosineWithNegativeSideLengthB()
   {
      double a = 1.0;
      GeometryTools.getUnknownTriangleAngleByLawOfCosine(a, -a, a);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public void testAngleByLawOfCosineWithNegativeSideLengthC()
   {
      double a = 1.0;
      GeometryTools.getUnknownTriangleAngleByLawOfCosine(a, a, -a);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAngleByLawOfCosineWithEqualLengthTriangle()
   {
      double a = 1.0;
      double alpha = GeometryTools.getUnknownTriangleAngleByLawOfCosine(a, a, a);
      double expected_alpha = Math.PI / 3.0;
      assertEquals(expected_alpha, alpha, 1e-10);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public void testSideLengthByLawOfCosineNegativeSideLengthA()
   {
      double a = 1.0;
      double gamma = Math.PI / 3.0;
      GeometryTools.getUnknownTriangleSideLengthByLawOfCosine(-a, a, gamma);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public void testSideLengthByLawOfCosineNegativeSideLengthB()
   {
      double a = 1.0;
      double gamma = Math.PI / 3.0;
      GeometryTools.getUnknownTriangleSideLengthByLawOfCosine(a, -a, gamma);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public void testSideLengthByLawOfCosineGreaterThanPiAngle()
   {
      double a = 1.0;
      double gamma = Math.PI / 2.0 * 3.0;
      GeometryTools.getUnknownTriangleSideLengthByLawOfCosine(a, a, gamma);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSideLengthByLawOfCosineWithEqualLengthTriangle()
   {
      double a = 1.0;
      double gamma = Math.PI / 3.0;
      double c = GeometryTools.getUnknownTriangleSideLengthByLawOfCosine(a, a, gamma);
      assertEquals(a, c, 1e-10);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSideLengthByLawOfCosineWithEqualLengthTriangleNegativeAngle()
   {
      double a = 1.0;
      double gamma = -Math.PI / 3.0;
      double c = GeometryTools.getUnknownTriangleSideLengthByLawOfCosine(a, a, gamma);
      assertEquals(a, c, 1e-10);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLawOfCosineRadnom()
   {
      Random random = new Random(34534L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D a = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D b = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D c = RandomGeometry.nextPoint2D(random, 10.0, 10.0);

         Vector2D ab = new Vector2D();
         ab.sub(b, a);
         Vector2D ba = new Vector2D();
         ba.sub(a, b);
         Vector2D ac = new Vector2D();
         ac.sub(c, a);
         Vector2D ca = new Vector2D();
         ca.sub(a, c);
         Vector2D bc = new Vector2D();
         bc.sub(c, b);
         Vector2D cb = new Vector2D();
         cb.sub(b, c);

         // The three edge lengths
         double abLength = ab.length();
         double acLength = ac.length();
         double bcLength = bc.length();

         // The three angles
         double abc = Math.abs(ba.angle(bc));
         double bca = Math.abs(cb.angle(ca));
         double cab = Math.abs(ac.angle(ab));

         assertEquals(bcLength, GeometryTools.getUnknownTriangleSideLengthByLawOfCosine(abLength, acLength, cab), Epsilons.ONE_TRILLIONTH);
         assertEquals(abLength, GeometryTools.getUnknownTriangleSideLengthByLawOfCosine(acLength, bcLength, bca), Epsilons.ONE_TRILLIONTH);
         assertEquals(acLength, GeometryTools.getUnknownTriangleSideLengthByLawOfCosine(abLength, bcLength, abc), Epsilons.ONE_TRILLIONTH);

         assertEquals(cab, GeometryTools.getUnknownTriangleAngleByLawOfCosine(abLength, acLength, bcLength), Epsilons.ONE_TRILLIONTH);
         assertEquals(bca, GeometryTools.getUnknownTriangleAngleByLawOfCosine(acLength, bcLength, abLength), Epsilons.ONE_TRILLIONTH);
         assertEquals(abc, GeometryTools.getUnknownTriangleAngleByLawOfCosine(abLength, bcLength, acLength), Epsilons.ONE_TRILLIONTH);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void isFormingTriangleFailTest()
   {
      double a = 1.0;
      double b = 10.0;
      boolean actual = GeometryTools.isFormingTriangle(b, a, a);
      assertEquals(false, actual);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void isFormingTriangleSameSidedTest()
   {
      double a = 1.0;
      boolean actual = GeometryTools.isFormingTriangle(a, a, a);
      assertEquals(true, actual);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public void illegalPythagorasGetCathetus()
   {
      GeometryTools.pythagorasGetCathetus(1.0, 2.0);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testClipToBoundingBox()
   {
      Tuple3DBasics tuple3d = new Point3D(1.0, -1.0, 0.0);
      GeometryTools.clipToBoundingBox(tuple3d, -0.5, 0.5, 0.5, -0.5, 0.0, 0.0);
      EuclidCoreTestTools.assertTuple3DEquals("not equal", new Point3D(0.5, -0.5, 0.0), tuple3d, 0.0);
      tuple3d.set(1.0, -1.0, 0.0);
      GeometryTools.clipToBoundingBox(tuple3d, 0.5, -0.5, -0.5, 0.5, -0.1, 0.1);
      EuclidCoreTestTools.assertTuple3DEquals("not equal", new Point3D(0.5, -0.5, 0.0), tuple3d, 0.0);
      tuple3d.set(1.0, -1.0, 2.0);
      GeometryTools.clipToBoundingBox(tuple3d, 0.5, -0.5, -0.5, 0.5, -0.1, 1.0);
      EuclidCoreTestTools.assertTuple3DEquals("not equal", new Point3D(0.5, -0.5, 1.0), tuple3d, 0.0);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCombine()
   {
      Random random = new Random(1176L);
      ArrayList<Point2D> firstList = new ArrayList<Point2D>();
      for (int i = 0; i < 100; i++)
      {
         firstList.add(new Point2D(random.nextDouble(), random.nextDouble()));
      }

      ConvexPolygon2d firstPolygon = new ConvexPolygon2d(firstList);

      ArrayList<Point2D> secondList = new ArrayList<Point2D>();
      for (int i = 0; i < 200; i++)
      {
         secondList.add(new Point2D(random.nextDouble(), random.nextDouble()));
      }

      ConvexPolygon2d secondPolygon = new ConvexPolygon2d(secondList);

      ConvexPolygon2d result = new ConvexPolygon2d(firstPolygon, secondPolygon);

      // convexity of the result is already checked in another test
      for (Point2D point : firstList)
      {
         if (!result.isPointInside(point))
         {
            double distance = result.distance(point);

            if (distance > 1e-7)
               throw new RuntimeException("Not each point is inside the result. distance = " + distance);
         }

         //       assertTrue("Not each point isinside the result. distance = " , result.isPointInside(point));
      }

      for (Point2D point : secondList)
      {
         if (!result.isPointInside(point))
         {
            double distance = result.distance(point);

            if (distance > 1e-7)
               throw new RuntimeException("Not each point is inside the result. distance = " + distance);
         }

         //       assertTrue("Not each point is inside the result", result.isPointInside(point));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetAngleFromFirstToSecondVector() throws Exception
   {
      Random random = new Random(51651L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double firstVectorLength = RandomNumbers.nextDouble(random, 0.0, 10.0);
         double secondVectorLength = RandomNumbers.nextDouble(random, 0.0, 10.0);
         Vector2D firstVector = RandomGeometry.nextVector2D(random, firstVectorLength);
         Vector2D secondVector = new Vector2D();

         for (double yaw = -Math.PI; yaw <= Math.PI; yaw += Math.PI / 100.0)
         {
            double c = Math.cos(yaw);
            double s = Math.sin(yaw);
            secondVector.setX(firstVector.getX() * c - firstVector.getY() * s);
            secondVector.setY(firstVector.getX() * s + firstVector.getY() * c);
            secondVector.scale(secondVectorLength / firstVectorLength);
            double computedYaw = GeometryTools.getAngleFromFirstToSecondVector(firstVector, secondVector);
            double yawDifference = AngleTools.computeAngleDifferenceMinusPiToPi(yaw, computedYaw);
            assertEquals(0.0, yawDifference, Epsilons.ONE_TRILLIONTH);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetClosestPointsForTwoLines() throws Exception
   {
      Point3D expectedPointOnLine1ToPack = new Point3D();
      Point3D expectedPointOnLine2ToPack = new Point3D();

      Point3D actualPointOnLine1ToPack = new Point3D();
      Point3D actualPointOnLine2ToPack = new Point3D();
      Random random = new Random(1176L);

      // Most usual case: the lines are not parallel, not intersecting.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineStart1 = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Vector3D lineDirection1 = RandomGeometry.nextVector3D(random, RandomNumbers.nextDouble(random, 0.5, 10.0));

         // Make line2 == line1
         Point3D lineStart2 = new Point3D(lineStart1);
         Vector3D lineDirection2 = new Vector3D(lineDirection1);

         // Shift orthogonally line2 away from line1.
         Vector3D orthogonalToLine1 = RandomGeometry.nextOrthogonalVector3D(random, lineDirection1, true);
         double expectedMinimumDistance = RandomNumbers.nextDouble(random, 0.0, 10.0);
         lineStart2.scaleAdd(expectedMinimumDistance, orthogonalToLine1, lineStart1);

         // Rotate line2 around the vector we shifted it along, so it preserves the minimum distance.
         AxisAngle axisAngleAroundShiftVector = new AxisAngle(orthogonalToLine1, RandomNumbers.nextDouble(random, Math.PI));
         RotationMatrix rotationMatrixAroundShiftVector = new RotationMatrix();
         rotationMatrixAroundShiftVector.set(axisAngleAroundShiftVector);
         rotationMatrixAroundShiftVector.transform(lineDirection2);

         // At this point, lineStart1 and lineStart2 are expected to be the closest points.
         expectedPointOnLine1ToPack.set(lineStart1);
         expectedPointOnLine2ToPack.set(lineStart2);

         double actualMinimumDistance = GeometryTools.getClosestPointsForTwoLines(lineStart1, lineDirection1, lineStart2, lineDirection2, actualPointOnLine1ToPack, actualPointOnLine2ToPack);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLine1ToPack, actualPointOnLine1ToPack, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLine2ToPack, actualPointOnLine2ToPack, EPSILON);

         // Let's shift lineStart1 and lineStart2 along their respective line direction so they're not the closest points.
         lineStart1.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection1, lineStart1);
         lineStart2.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection2, lineStart2);

         actualMinimumDistance = GeometryTools.getClosestPointsForTwoLines(lineStart1, lineDirection1, lineStart2, lineDirection2, actualPointOnLine1ToPack, actualPointOnLine2ToPack);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLine1ToPack, actualPointOnLine1ToPack, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLine2ToPack, actualPointOnLine2ToPack, EPSILON);
      }

      // Test the parallel case. There's an infinite number of solutions but only one minimum distance between the two lines.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineStart1 = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Vector3D lineDirection1 = RandomGeometry.nextVector3D(random, 1.0);

         // Make line2 == line1
         Point3D lineStart2 = new Point3D(lineStart1);
         Vector3D lineDirection2 = new Vector3D(lineDirection1);

         // Shift orthogonally line2 away from line1.
         Vector3D orthogonalToLine1 = RandomGeometry.nextOrthogonalVector3D(random, lineDirection1, true);
         double expectedMinimumDistance = RandomNumbers.nextDouble(random, 0.0, 10.0);
         lineStart2.scaleAdd(expectedMinimumDistance, orthogonalToLine1, lineStart1);

         double actualMinimumDistance = GeometryTools.getClosestPointsForTwoLines(lineStart1, lineDirection1, lineStart2, lineDirection2, actualPointOnLine1ToPack, actualPointOnLine2ToPack);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);

         // Let's shift lineStart1 and lineStart2 along their respective line direction (the minimum distance should remain the same).
         lineStart1.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection1, lineStart1);
         lineStart2.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection2, lineStart2);

         actualMinimumDistance = GeometryTools.getClosestPointsForTwoLines(lineStart1, lineDirection1, lineStart2, lineDirection2, actualPointOnLine1ToPack, actualPointOnLine2ToPack);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
      }

      // Intersection case: the lines are not parallel, but intersecting.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineStart1 = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Vector3D lineDirection1 = RandomGeometry.nextVector3D(random, RandomNumbers.nextDouble(random, 0.5, 10.0));

         // Set the intersection point randomly on line1
         Point3D intersection = new Point3D();
         intersection.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection1, lineStart1);

         // Set both closest points to the intersection
         expectedPointOnLine1ToPack.set(intersection);
         expectedPointOnLine2ToPack.set(intersection);

         // Create line2 such that it intersects line1 at intersection
         Point3D lineStart2 = new Point3D(intersection);
         Vector3D lineDirection2 = RandomGeometry.nextVector3D(random, RandomNumbers.nextDouble(random, 10.0));
         lineStart2.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection2, lineStart2);

         double actualMinimumDistance = GeometryTools.getClosestPointsForTwoLines(lineStart1, lineDirection1, lineStart2, lineDirection2, actualPointOnLine1ToPack, actualPointOnLine2ToPack);
         assertEquals(0.0, actualMinimumDistance, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLine1ToPack, actualPointOnLine1ToPack, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLine2ToPack, actualPointOnLine2ToPack, EPSILON);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetClosestPointsForTwoLineSegments() throws Exception
   {
      Point3D expectedPointOnLineSegment1 = new Point3D();
      Point3D expectedPointOnLineSegment2 = new Point3D();

      Point3D actualPointOnLineSegment1 = new Point3D();
      Point3D actualPointOnLineSegment2 = new Point3D();
      
      Vector3D lineSegmentDirection1 = new Vector3D();
      Vector3D lineSegmentDirection2 = new Vector3D();

      Random random = new Random(1176L);

      // Easy case, the closest points on inside each line segment bounds.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart1 = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Point3D lineSegmentEnd1 = RandomGeometry.nextPoint3D(random, -10.0, 10.0);

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // Put the first closest within bounds of line segment 1
         expectedPointOnLineSegment1.interpolate(lineSegmentStart1, lineSegmentEnd1, RandomNumbers.nextDouble(random, 0.0, 1.0));

         // Create the closest point of line segment 2
         Vector3D orthogonalToLineSegment1 = RandomGeometry.nextOrthogonalVector3D(random, lineSegmentDirection1, true);
         expectedPointOnLineSegment2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), orthogonalToLineSegment1, expectedPointOnLineSegment1);

         // Set the line direction 2 to be the rotation of 1 around the shift direction used to create the expectedPointOnLineSegment2
         double rotationAngle = RandomNumbers.nextDouble(random, 2.0 * Math.PI);
         AxisAngle rotationAroundShiftVector = new AxisAngle(orthogonalToLineSegment1, rotationAngle);
         rotationAroundShiftVector.transform(lineSegmentDirection1, lineSegmentDirection2);

         // Set the end points of the line segment 2 around the expected closest point.
         Point3D lineSegmentStart2 = new Point3D();
         Point3D lineSegmentEnd2 = new Point3D();
         lineSegmentStart2.scaleAdd(RandomNumbers.nextDouble(random, -10.0, 0.0), lineSegmentDirection2, expectedPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineSegmentDirection2, expectedPointOnLineSegment2);

         GeometryTools.getClosestPointsForTwoLineSegments(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2, actualPointOnLineSegment1, actualPointOnLineSegment2);
         
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, EPSILON);
      }

      // Parallel case, expecting expectedPointOnLineSegment1 = lineSegmentStart1
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart1 = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Point3D lineSegmentEnd1 = RandomGeometry.nextPoint3D(random, -10.0, 10.0);

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // expectedPointOnLineSegment1 = lineSegmentStart1
         expectedPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2
         Vector3D orthogonalToLineSegment1 = RandomGeometry.nextOrthogonalVector3D(random, lineSegmentDirection1, true);
         expectedPointOnLineSegment2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), orthogonalToLineSegment1, expectedPointOnLineSegment1);

         // Set the lineSegmentDirection2 = lineSegmentDirection1
         lineSegmentDirection2.set(lineSegmentDirection1);

         // Set the end points of the line segment 2 around the expected closest point.
         Point3D lineSegmentStart2 = new Point3D();
         Point3D lineSegmentEnd2 = new Point3D();
         lineSegmentStart2.scaleAdd(RandomNumbers.nextDouble(random, -10.0, 0.0), lineSegmentDirection2, expectedPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineSegmentDirection2, expectedPointOnLineSegment2);

         GeometryTools.getClosestPointsForTwoLineSegments(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2, actualPointOnLineSegment1, actualPointOnLineSegment2);
         
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, EPSILON);

         // Set the end points of the line segment 2 before the expected closest point, so we have expectedClosestPointOnLineSegment2 = lineSegmentEnd2
         double shiftStartFromExpected = RandomNumbers.nextDouble(random, -20.0, -10.0);
         double shiftEndFromExpected = RandomNumbers.nextDouble(random, -10.0, 0.0);
         lineSegmentStart2.scaleAdd(shiftStartFromExpected, lineSegmentDirection2, expectedPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(shiftEndFromExpected, lineSegmentDirection2, expectedPointOnLineSegment2);
         expectedPointOnLineSegment2.set(lineSegmentEnd2);

         GeometryTools.getClosestPointsForTwoLineSegments(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2, actualPointOnLineSegment1, actualPointOnLineSegment2);
         
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, EPSILON);

         GeometryTools.getClosestPointsForTwoLineSegments(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2, actualPointOnLineSegment1, actualPointOnLineSegment2);
         
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, EPSILON);
      }

      // Case: on closest point on lineSegment1 outside end points.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart1 = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Point3D lineSegmentEnd1 = RandomGeometry.nextPoint3D(random, -10.0, 10.0);

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // Put the first closest to the start of line segment 1
         expectedPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2 such that it reaches out of line segment 1
         Vector3D oppositeOflineSegmentDirection1 = new Vector3D();
         oppositeOflineSegmentDirection1.setAndNegate(lineSegmentDirection1);
         Vector3D orthogonalToLineSegment1 = RandomGeometry.nextOrthogonalVector3D(random, lineSegmentDirection1, true);
         Vector3D shiftVector = new Vector3D();
         shiftVector.interpolate(orthogonalToLineSegment1, oppositeOflineSegmentDirection1, RandomNumbers.nextDouble(random, 0.0, 1.0));
         expectedPointOnLineSegment2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), shiftVector, expectedPointOnLineSegment1);

         // Set the line direction 2 to orthogonal to the shift vector
         lineSegmentDirection2 = RandomGeometry.nextOrthogonalVector3D(random, shiftVector, true);

         // Set the end points of the line segment 2 around the expected closest point.
         Point3D lineSegmentStart2 = new Point3D();
         Point3D lineSegmentEnd2 = new Point3D();
         lineSegmentStart2.scaleAdd(RandomNumbers.nextDouble(random, -10.0, 0.0), lineSegmentDirection2, expectedPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineSegmentDirection2, expectedPointOnLineSegment2);

         GeometryTools.getClosestPointsForTwoLineSegments(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2, actualPointOnLineSegment1, actualPointOnLineSegment2);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, EPSILON);

         GeometryTools.getClosestPointsForTwoLineSegments(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2, actualPointOnLineSegment1, actualPointOnLineSegment2);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, EPSILON);

         GeometryTools.getClosestPointsForTwoLineSegments(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2, actualPointOnLineSegment1, actualPointOnLineSegment2);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, EPSILON);

         GeometryTools.getClosestPointsForTwoLineSegments(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2, actualPointOnLineSegment1, actualPointOnLineSegment2);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, EPSILON);
      }
      
      // Edge case: both closest points are outside bounds of each line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart1 = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Point3D lineSegmentEnd1 = RandomGeometry.nextPoint3D(random, -10.0, 10.0);

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // Put the first closest to the start of line segment 1
         expectedPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2 such that it reaches out of line segment 1
         Vector3D oppositeOflineSegmentDirection1 = new Vector3D();
         oppositeOflineSegmentDirection1.setAndNegate(lineSegmentDirection1);
         Vector3D orthogonalToLineSegment1 = RandomGeometry.nextOrthogonalVector3D(random, lineSegmentDirection1, true);
         Vector3D shiftVector = new Vector3D();
         shiftVector.interpolate(orthogonalToLineSegment1, oppositeOflineSegmentDirection1, RandomNumbers.nextDouble(random, 0.0, 1.0));
         expectedPointOnLineSegment2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), shiftVector, expectedPointOnLineSegment1);

         // set the start of the second line segment to the expected closest point
         Point3D lineSegmentStart2 = new Point3D(expectedPointOnLineSegment2);
         

         // Set the line direction 2 to point somewhat in the same direction as the shift vector
         Vector3D orthogonalToShiftVector = RandomGeometry.nextOrthogonalVector3D(random, shiftVector, true);
         lineSegmentDirection2.interpolate(shiftVector, orthogonalToShiftVector, RandomNumbers.nextDouble(random, 0.0, 1.0));

         // Set the end points of the line segment 2 around the expected closest point.
         Point3D lineSegmentEnd2 = new Point3D();
         lineSegmentEnd2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineSegmentDirection2, expectedPointOnLineSegment2);

         GeometryTools.getClosestPointsForTwoLineSegments(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2, actualPointOnLineSegment1, actualPointOnLineSegment2);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, EPSILON);

         GeometryTools.getClosestPointsForTwoLineSegments(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2, actualPointOnLineSegment1, actualPointOnLineSegment2);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, EPSILON);

         GeometryTools.getClosestPointsForTwoLineSegments(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2, actualPointOnLineSegment1, actualPointOnLineSegment2);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, EPSILON);

         GeometryTools.getClosestPointsForTwoLineSegments(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2, actualPointOnLineSegment1, actualPointOnLineSegment2);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, EPSILON);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testIsPointInsideTriangleABC() throws Exception
   {
      Point2D inside = new Point2D();
      Point2D outside = new Point2D();
      Random random = new Random(1176L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D a = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D b = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D c = RandomGeometry.nextPoint2D(random, 10.0, 10.0);

         assertTrue(GeometryTools.isPointInsideTriangleABC(a, a, b, c));
         assertTrue(GeometryTools.isPointInsideTriangleABC(a, c, b, a));
         assertTrue(GeometryTools.isPointInsideTriangleABC(b, a, b, c));
         assertTrue(GeometryTools.isPointInsideTriangleABC(b, c, b, a));
         assertTrue(GeometryTools.isPointInsideTriangleABC(c, a, b, c));
         assertTrue(GeometryTools.isPointInsideTriangleABC(c, c, b, a));

         inside.interpolate(a, b, RandomNumbers.nextDouble(random, 0.0, 1.0));
         inside.interpolate(inside, c, RandomNumbers.nextDouble(random, 0.0, 1.0));
         assertTrue(GeometryTools.isPointInsideTriangleABC(inside, a, b, c));
         assertTrue(GeometryTools.isPointInsideTriangleABC(inside, c, b, a));

         outside.interpolate(a, b, RandomNumbers.nextDouble(random, 1.0, 10.0));
         outside.interpolate(outside, c, RandomNumbers.nextDouble(random, 0.0, 1.0));
         assertFalse(GeometryTools.isPointInsideTriangleABC(outside, a, b, c));
         assertFalse(GeometryTools.isPointInsideTriangleABC(outside, c, b, a));

         outside.interpolate(a, b, RandomNumbers.nextDouble(random, -10.0, 0.0));
         outside.interpolate(outside, c, RandomNumbers.nextDouble(random, 0.0, 1.0));
         assertFalse(GeometryTools.isPointInsideTriangleABC(outside, a, b, c));
         assertFalse(GeometryTools.isPointInsideTriangleABC(outside, c, b, a));

         outside.interpolate(a, b, RandomNumbers.nextDouble(random, 0.0, 1.0));
         outside.interpolate(outside, c, RandomNumbers.nextDouble(random, 1.0, 10.0));
         assertFalse(GeometryTools.isPointInsideTriangleABC(outside, a, b, c));
         assertFalse(GeometryTools.isPointInsideTriangleABC(outside, c, b, a));

         outside.interpolate(a, b, RandomNumbers.nextDouble(random, 0.0, 1.0));
         outside.interpolate(outside, c, RandomNumbers.nextDouble(random, -10.0, 0.0));
         assertFalse(GeometryTools.isPointInsideTriangleABC(outside, a, b, c));
         assertFalse(GeometryTools.isPointInsideTriangleABC(outside, c, b, a));
      }

      Point2D a = new Point2D(1.0, 0.0);
      Point2D b = new Point2D(1.0, 1.0);
      Point2D c = new Point2D(0.0, 1.0);

      // These tests tend to be flaky inside the loop
      inside.interpolate(a, b, 0.5);
      assertTrue(GeometryTools.isPointInsideTriangleABC(inside, a, b, c));
      assertTrue(GeometryTools.isPointInsideTriangleABC(inside, c, b, a));
      inside.interpolate(a, c, 0.5);
      assertTrue(GeometryTools.isPointInsideTriangleABC(inside, a, b, c));
      assertTrue(GeometryTools.isPointInsideTriangleABC(inside, c, b, a));
      inside.interpolate(b, c, 0.5);
      assertTrue(GeometryTools.isPointInsideTriangleABC(inside, a, b, c));
      assertTrue(GeometryTools.isPointInsideTriangleABC(inside, c, b, a));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testComputeTriangleArea() throws Exception
   {
      Random random = new Random(1176L);
      // Test for right rectangle, should be half the area of the corresponding rectangle
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D a = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D b = new Point2D();
         Point2D c = new Point2D();
         Point2D d = new Point2D();

         Vector2D rectangleLength = RandomGeometry.nextVector2D(random, 1.0);
         Vector2D rectangleWidth = new Vector2D(-rectangleLength.getY(), rectangleLength.getX());
         double length = RandomNumbers.nextDouble(random, 0.0, 10.0);
         double width = RandomNumbers.nextDouble(random, 0.0, 10.0);
         rectangleLength.scale(length);
         rectangleWidth.scale(width);

         b.add(a, rectangleLength);
         c.add(b, rectangleWidth);
         d.add(a, rectangleWidth);

         double expectedArea = 0.5 * length * width;
         double actualArea = GeometryTools.computeTriangleArea(a, b, c);
         assertEquals(expectedArea, actualArea, EPSILON);
         actualArea = GeometryTools.computeTriangleArea(a, c, d);
         assertEquals(expectedArea, actualArea, EPSILON);
         actualArea = GeometryTools.computeTriangleArea(b, c, d);
         assertEquals(expectedArea, actualArea, EPSILON);
         actualArea = GeometryTools.computeTriangleArea(a, b, d);
         assertEquals(expectedArea, actualArea, EPSILON);

         // Just an annoying case
         assertEquals(0.0, GeometryTools.computeTriangleArea(a, a, c), EPSILON);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testNormalizeSafeZUp() throws Exception
   {
      Vector3D actualVector;
      Vector3D expectedVector = new Vector3D();
      Random random = new Random(1176L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         actualVector = RandomGeometry.nextVector3D(random, RandomNumbers.nextDouble(random, Epsilons.ONE_TRILLIONTH, 10.0));

         expectedVector.setAndNormalize(actualVector);
         GeometryTools.normalizeSafelyZUp(actualVector);
         EuclidCoreTestTools.assertTuple3DEquals(expectedVector, actualVector, Epsilons.ONE_TRILLIONTH);

         actualVector = RandomGeometry.nextVector3D(random, 0.999 * Epsilons.ONE_TRILLIONTH);
         expectedVector.set(0.0, 0.0, 1.0);
         GeometryTools.normalizeSafelyZUp(actualVector);
         EuclidCoreTestTools.assertTuple3DEquals(expectedVector, actualVector, Epsilons.ONE_TRILLIONTH);

         actualVector = new Vector3D();
         expectedVector.set(0.0, 0.0, 1.0);
         GeometryTools.normalizeSafelyZUp(actualVector);
         EuclidCoreTestTools.assertTuple3DEquals(expectedVector, actualVector, Epsilons.ONE_TRILLIONTH);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetIntersectionBetweenLineAndPlane() throws Exception
   {
      Random random = new Random(1176L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointOnPlane = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Vector3D planeNormal = RandomGeometry.nextVector3D(random, RandomNumbers.nextDouble(random, 10.0));
         Vector3D parallelToPlane = RandomGeometry.nextOrthogonalVector3D(random, planeNormal, true);

         Point3D expectedIntersection = new Point3D();
         expectedIntersection.scaleAdd(RandomNumbers.nextDouble(random, 10.0), parallelToPlane, pointOnPlane);

         Vector3D lineDirection = RandomGeometry.nextVector3D(random, RandomNumbers.nextDouble(random, 10.0));
         Point3D pointOnLine = new Point3D();
         pointOnLine.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection, expectedIntersection);

         Point3D actualIntersection = GeometryTools.getIntersectionBetweenLineAndPlane(pointOnPlane, planeNormal, pointOnLine, lineDirection);

         double epsilon = Epsilons.ONE_TRILLIONTH;
         if (Math.abs(lineDirection.angle(planeNormal)) > Math.PI / 2.0 - 0.001)
            epsilon = Epsilons.ONE_HUNDRED_BILLIONTH; // Loss of precision when the line direction and the plane normal are almost orthogonal.

         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection, actualIntersection, epsilon);
      }

      // Try parallel lines to plane
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointOnPlane = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Vector3D planeNormal = RandomGeometry.nextVector3D(random, RandomNumbers.nextDouble(random, 10.0));

         Vector3D lineDirection = RandomGeometry.nextOrthogonalVector3D(random, planeNormal, false);
         Point3D pointOnLine = new Point3D();
         pointOnLine.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection, pointOnPlane);

         Point3D actualIntersection = GeometryTools.getIntersectionBetweenLineAndPlane(pointOnPlane, planeNormal, pointOnLine, lineDirection);
         assertNull(actualIntersection);

         pointOnLine.scaleAdd(RandomNumbers.nextDouble(random, 1.0), planeNormal, pointOnLine);
         actualIntersection = GeometryTools.getIntersectionBetweenLineAndPlane(pointOnPlane, planeNormal, pointOnLine, lineDirection);
         assertNull(actualIntersection);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetIntersectionBetweenLineSegmentAndPlane2() throws Exception
   {
      Point3D endPoint0 = new Point3D();
      Point3D endPoint1 = new Point3D();

      Random random = new Random(1176L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointOnPlane = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Vector3D planeNormal = RandomGeometry.nextVector3D(random, RandomNumbers.nextDouble(random, 10.0));
         Vector3D parallelToPlane = RandomGeometry.nextOrthogonalVector3D(random, planeNormal, true);

         Point3D expectedIntersection = new Point3D();
         expectedIntersection.scaleAdd(RandomNumbers.nextDouble(random, 10.0), parallelToPlane, pointOnPlane);

         Vector3D lineDirection = RandomGeometry.nextVector3D(random, RandomNumbers.nextDouble(random, 10.0));

         // Expecting an actual intersection
         endPoint0.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         endPoint1.scaleAdd(RandomNumbers.nextDouble(random, -10.0, 0.0), lineDirection, expectedIntersection);
         Point3D actualIntersection = GeometryTools.getIntersectionBetweenLineSegmentAndPlane(pointOnPlane, planeNormal, endPoint0, endPoint1);
         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection, actualIntersection, Epsilons.ONE_TRILLIONTH);
         actualIntersection = GeometryTools.getIntersectionBetweenLineSegmentAndPlane(pointOnPlane, planeNormal, endPoint1, endPoint0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection, actualIntersection, Epsilons.ONE_TRILLIONTH);

         // Expecting no intersection
         endPoint0.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         endPoint1.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         actualIntersection = GeometryTools.getIntersectionBetweenLineSegmentAndPlane(pointOnPlane, planeNormal, endPoint0, endPoint1);
         assertNull(actualIntersection);
         actualIntersection = GeometryTools.getIntersectionBetweenLineSegmentAndPlane(pointOnPlane, planeNormal, endPoint1, endPoint0);
         assertNull(actualIntersection);
      }

      // Try parallel lines to plane
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointOnPlane = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Vector3D planeNormal = RandomGeometry.nextVector3D(random, RandomNumbers.nextDouble(random, 10.0));

         Vector3D lineDirection = RandomGeometry.nextOrthogonalVector3D(random, planeNormal, false);
         endPoint0.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection, pointOnPlane);
         endPoint1.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection, pointOnPlane);

         Point3D actualIntersection = GeometryTools.getIntersectionBetweenLineSegmentAndPlane(pointOnPlane, planeNormal, endPoint0, endPoint1);
         assertNull(actualIntersection);

         double distanceAwayFromPlane = RandomNumbers.nextDouble(random, 1.0);
         endPoint0.scaleAdd(distanceAwayFromPlane, planeNormal, endPoint0);
         endPoint1.scaleAdd(distanceAwayFromPlane, planeNormal, endPoint0);
         actualIntersection = GeometryTools.getIntersectionBetweenLineSegmentAndPlane(pointOnPlane, planeNormal, endPoint0, endPoint1);
         assertNull(actualIntersection);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetRadiusOfArc() throws Exception
   {
      Random random = new Random(1176L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         double expectedArcRadius = RandomNumbers.nextDouble(random, 0.1, 100.0);
         double chordAngle = RandomNumbers.nextDouble(random, -3.0 * Math.PI, 3.0 * Math.PI);
         double chordLength = 2.0 * expectedArcRadius * Math.sin(0.5 * chordAngle);
         double actualArcRadius = GeometryTools.getRadiusOfArc(chordLength, chordAngle);
         assertEquals(expectedArcRadius, actualArcRadius, Epsilons.ONE_TRILLIONTH);
      }

      assertTrue(Double.isNaN(GeometryTools.getRadiusOfArc(1.0, 0.0)));
      assertTrue(Double.isNaN(GeometryTools.getRadiusOfArc(1.0, Math.PI)));
      assertTrue(Double.isNaN(GeometryTools.getRadiusOfArc(1.0, -Math.PI)));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetAxisAngleFromFirstToSecondVector1() throws Exception
   {
      Random random = new Random(1176L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D firstVector = RandomGeometry.nextVector3D(random, RandomNumbers.nextDouble(random, 0.0, 10.0));
         double expectedAngle = RandomNumbers.nextDouble(random, 0.0, Math.PI);
         Vector3D expectedAxis = RandomGeometry.nextOrthogonalVector3D(random, firstVector, true);
         AxisAngle expectedAxisAngle = new AxisAngle(expectedAxis, expectedAngle);
         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(expectedAxisAngle);

         Vector3D secondVector = new Vector3D();
         rotationMatrix.transform(firstVector, secondVector);
         secondVector.scale(RandomNumbers.nextDouble(random, 0.0, 10.0));

         AxisAngle actualAxisAngle = new AxisAngle();
         GeometryTools.getAxisAngleFromFirstToSecondVector(firstVector, secondVector, actualAxisAngle);

         Vector3D actualAxis = new Vector3D(actualAxisAngle.getX(), actualAxisAngle.getY(), actualAxisAngle.getZ());

         assertEquals(1.0, actualAxis.length(), Epsilons.ONE_TRILLIONTH);
         assertEquals(0.0, actualAxis.dot(firstVector), Epsilons.ONE_TRILLIONTH);
         assertEquals(0.0, actualAxis.dot(secondVector), Epsilons.ONE_TRILLIONTH);

         assertEquals(0.0, expectedAxis.dot(firstVector), Epsilons.ONE_TRILLIONTH);
         assertEquals(0.0, expectedAxis.dot(secondVector), Epsilons.ONE_TRILLIONTH);

         if (actualAxisAngle.getAngle() * expectedAxisAngle.getAngle() < 0.0)
         {
            expectedAxis.negate();
            expectedAngle = -expectedAngle;
            expectedAxisAngle.set(expectedAxis, expectedAngle);
         }

         try
         {
            assertTrue(expectedAxisAngle.epsilonEquals(actualAxisAngle, Epsilons.ONE_TRILLIONTH));
         }
         catch (AssertionError e)
         {
            throw new AssertionError("expected:\n<" + expectedAxisAngle + ">\n but was:\n<" + actualAxisAngle + ">");
         }
      }

      // Test close to 0.0
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D firstVector = RandomGeometry.nextVector3D(random, RandomNumbers.nextDouble(random, 0.0, 10.0));
         double expectedAngle = RandomNumbers.nextDouble(random, 0.0001, 0.001);
         if (random.nextBoolean())
            expectedAngle = -expectedAngle;
         Vector3D expectedAxis = RandomGeometry.nextOrthogonalVector3D(random, firstVector, true);
         AxisAngle expectedAxisAngle = new AxisAngle(expectedAxis, expectedAngle);
         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(expectedAxisAngle);

         Vector3D secondVector = new Vector3D();
         rotationMatrix.transform(firstVector, secondVector);
         secondVector.scale(RandomNumbers.nextDouble(random, 0.0, 10.0));

         AxisAngle actualAxisAngle = new AxisAngle();
         GeometryTools.getAxisAngleFromFirstToSecondVector(firstVector, secondVector, actualAxisAngle);

         Vector3D actualAxis = new Vector3D(actualAxisAngle.getX(), actualAxisAngle.getY(), actualAxisAngle.getZ());

         assertEquals(1.0, actualAxis.length(), Epsilons.ONE_TRILLIONTH);
         // Can not be as accurate as we get closer to 0.0
         assertEquals(0.0, actualAxis.dot(firstVector), Epsilons.ONE_TEN_BILLIONTH);
         assertEquals(0.0, actualAxis.dot(secondVector), Epsilons.ONE_TEN_BILLIONTH);

         assertEquals(0.0, expectedAxis.dot(firstVector), Epsilons.ONE_TRILLIONTH);
         assertEquals(0.0, expectedAxis.dot(secondVector), Epsilons.ONE_TRILLIONTH);

         if (actualAxisAngle.getAngle() * expectedAxisAngle.getAngle() < 0.0)
         {
            expectedAxis.negate();
            expectedAngle = -expectedAngle;
            expectedAxisAngle.set(expectedAxis, expectedAngle);
         }

         try
         {
            // Can not be as accurate as we get closer to 0.0
            assertTrue(expectedAxisAngle.epsilonEquals(actualAxisAngle, Epsilons.ONE_TEN_BILLIONTH));
         }
         catch (AssertionError e)
         {
            throw new AssertionError("expected:\n<" + expectedAxisAngle + ">\n but was:\n<" + actualAxisAngle + ">");
         }
      }

      // Test close to Math.PI
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D referenceNormal = RandomGeometry.nextVector3D(random, RandomNumbers.nextDouble(random, 0.0, 10.0));
         double expectedAngle = RandomNumbers.nextDouble(random, 0.00001, 0.001);
         if (random.nextBoolean())
            expectedAngle = -expectedAngle;
         expectedAngle += Math.PI;
         Vector3D expectedAxis = RandomGeometry.nextOrthogonalVector3D(random, referenceNormal, true);
         AxisAngle expectedAxisAngle = new AxisAngle(expectedAxis, expectedAngle);
         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(expectedAxisAngle);

         Vector3D rotatedNormal = new Vector3D();
         rotationMatrix.transform(referenceNormal, rotatedNormal);
         rotatedNormal.scale(RandomNumbers.nextDouble(random, 0.0, 10.0));

         AxisAngle actualAxisAngle = new AxisAngle();
         GeometryTools.getAxisAngleFromFirstToSecondVector(referenceNormal, rotatedNormal, actualAxisAngle);

         Vector3D actualAxis = new Vector3D(actualAxisAngle.getX(), actualAxisAngle.getY(), actualAxisAngle.getZ());

         assertEquals(1.0, actualAxis.length(), Epsilons.ONE_TRILLIONTH);
         // Can not be as accurate as we get closer to Math.PI
         assertEquals(0.0, actualAxis.dot(referenceNormal), Epsilons.ONE_TEN_BILLIONTH);
         assertEquals(0.0, actualAxis.dot(rotatedNormal), Epsilons.ONE_TEN_BILLIONTH);

         assertEquals(0.0, expectedAxis.dot(referenceNormal), Epsilons.ONE_TRILLIONTH);
         assertEquals(0.0, expectedAxis.dot(rotatedNormal), Epsilons.ONE_TRILLIONTH);
         if (actualAxisAngle.getAngle() * expectedAxisAngle.getAngle() < 0.0)
         {
            expectedAxis.negate();
            expectedAngle = -expectedAngle;
            expectedAxisAngle.set(expectedAxis, expectedAngle);
         }

         if (Math.abs(expectedAxisAngle.getAngle() + actualAxisAngle.getAngle()) > 2.0 * Math.PI - 0.1)
         {
            // Here the sign of the axis does not matter.
            if (expectedAxis.dot(actualAxis) < 0.0)
               expectedAxis.negate();
            // Can not be as accurate as we get closer to Math.PI
            EuclidCoreTestTools.assertTuple3DEquals(expectedAxis, actualAxis, Epsilons.ONE_TEN_BILLIONTH);
         }
         else
         {
            try
            {
               assertTrue(expectedAxisAngle.epsilonEquals(actualAxisAngle, Epsilons.ONE_TRILLIONTH));
            }
            catch (AssertionError e)
            {
               throw new AssertionError("expected:\n<" + expectedAxisAngle + ">\n but was:\n<" + actualAxisAngle + ">");
            }
         }
      }

      // Test exactly at 0.0
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D referenceNormal = RandomGeometry.nextVector3D(random, RandomNumbers.nextDouble(random, 0.0, 10.0));
         Vector3D rotatedNormal = new Vector3D(referenceNormal);
         rotatedNormal.scale(RandomNumbers.nextDouble(random, 0.0, 10.0));
         double expectedAngle = 0.0;
         Vector3D expectedAxis = new Vector3D(1.0, 0.0, 0.0);
         AxisAngle expectedAxisAngle = new AxisAngle(expectedAxis, expectedAngle);

         AxisAngle actualAxisAngle = new AxisAngle();
         GeometryTools.getAxisAngleFromFirstToSecondVector(referenceNormal, rotatedNormal, actualAxisAngle);

         Vector3D actualAxis = new Vector3D(actualAxisAngle.getX(), actualAxisAngle.getY(), actualAxisAngle.getZ());

         assertEquals(1.0, actualAxis.length(), Epsilons.ONE_TRILLIONTH);

         if (actualAxisAngle.getAngle() * expectedAxisAngle.getAngle() < 0.0)
         {
            expectedAxis.negate();
            expectedAngle = -expectedAngle;
            expectedAxisAngle.set(expectedAxis, expectedAngle);
         }

         try
         {
            assertTrue(expectedAxisAngle.epsilonEquals(actualAxisAngle, Epsilons.ONE_TRILLIONTH));
         }
         catch (AssertionError e)
         {
            throw new AssertionError("expected:\n<" + expectedAxisAngle + ">\n but was:\n<" + actualAxisAngle + ">");
         }
      }

      // Test exactly at Math.PI
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D referenceNormal = RandomGeometry.nextVector3D(random, RandomNumbers.nextDouble(random, 0.0, 10.0));
         Vector3D rotatedNormal = new Vector3D();
         rotatedNormal.setAndNegate(referenceNormal);
         rotatedNormal.scale(RandomNumbers.nextDouble(random, 0.0, 10.0));
         double expectedAngle = Math.PI;
         Vector3D expectedAxis = new Vector3D(1.0, 0.0, 0.0);
         AxisAngle expectedAxisAngle = new AxisAngle(expectedAxis, expectedAngle);

         AxisAngle actualAxisAngle = new AxisAngle();
         GeometryTools.getAxisAngleFromFirstToSecondVector(referenceNormal, rotatedNormal, actualAxisAngle);

         Vector3D actualAxis = new Vector3D(actualAxisAngle.getX(), actualAxisAngle.getY(), actualAxisAngle.getZ());

         assertEquals(1.0, actualAxis.length(), Epsilons.ONE_TRILLIONTH);

         if (actualAxisAngle.getAngle() * expectedAxisAngle.getAngle() < 0.0)
         {
            expectedAxis.negate();
            expectedAngle = -expectedAngle;
            expectedAxisAngle.set(expectedAxis, expectedAngle);
         }

         try
         {
            assertTrue(expectedAxisAngle.epsilonEquals(actualAxisAngle, Epsilons.ONE_TRILLIONTH));
         }
         catch (AssertionError e)
         {
            throw new AssertionError("expected:\n<" + expectedAxisAngle + ">\n but was:\n<" + actualAxisAngle + ">");
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetAxisAngleFromFirstToSecondVector2() throws Exception
   {
      Random random = new Random(1176L);
      // Test getRotationBasedOnNormal(AxisAngle4d rotationToPack, Vector3d normalVector3d)
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D referenceNormal = new Vector3D(0.0, 0.0, 1.0);
         double expectedAngle = RandomNumbers.nextDouble(random, 0.0, Math.PI);
         Vector3D expectedAxis = RandomGeometry.nextOrthogonalVector3D(random, referenceNormal, true);
         AxisAngle expectedAxisAngle = new AxisAngle(expectedAxis, expectedAngle);
         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(expectedAxisAngle);

         Vector3D rotatedNormal = new Vector3D();
         rotationMatrix.transform(referenceNormal, rotatedNormal);
         rotatedNormal.scale(RandomNumbers.nextDouble(random, 0.0, 10.0));

         AxisAngle actualAxisAngle = new AxisAngle();
         GeometryTools.getAxisAngleFromFirstToSecondVector(referenceNormal, rotatedNormal, actualAxisAngle);

         Vector3D actualAxis = new Vector3D(actualAxisAngle.getX(), actualAxisAngle.getY(), actualAxisAngle.getZ());

         assertEquals(1.0, actualAxis.length(), Epsilons.ONE_TRILLIONTH);
         assertEquals(0.0, actualAxis.dot(referenceNormal), Epsilons.ONE_TRILLIONTH);
         assertEquals(0.0, actualAxis.dot(rotatedNormal), Epsilons.ONE_TRILLIONTH);

         assertEquals(0.0, expectedAxis.dot(referenceNormal), Epsilons.ONE_TRILLIONTH);
         assertEquals(0.0, expectedAxis.dot(rotatedNormal), Epsilons.ONE_TRILLIONTH);

         if (actualAxisAngle.getAngle() * expectedAxisAngle.getAngle() < 0.0)
         {
            expectedAxis.negate();
            expectedAngle = -expectedAngle;
            expectedAxisAngle.set(expectedAxis, expectedAngle);
         }

         try
         {
            assertTrue(expectedAxisAngle.epsilonEquals(actualAxisAngle, Epsilons.ONE_TRILLIONTH));
         }
         catch (AssertionError e)
         {
            throw new AssertionError("expected:\n<" + expectedAxisAngle + ">\n but was:\n<" + actualAxisAngle + ">");
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetPerpendicularBisectorSegment1() throws Exception
   {
      Point2D firstLinePoint = new Point2D(1.0, 1.0);
      Point2D secondLinePoint = new Point2D(0.0, 1.0);
      double lengthOffset = 2.0;
      List<Point2D> normalPointsFromLine = GeometryTools.getPerpendicularBisectorSegment(firstLinePoint, secondLinePoint, lengthOffset);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.5, -1.0), normalPointsFromLine.get(0), Epsilons.ONE_TRILLIONTH);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.5, 3.0), normalPointsFromLine.get(1), Epsilons.ONE_TRILLIONTH);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetPerpendicularBisectorSegment2() throws Exception
   {
      Random random = new Random(1176L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D secondPointOnLine = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Vector2D lineDirection = new Vector2D();
         lineDirection.sub(secondPointOnLine, firstPointOnLine);
         double lengthOffset = RandomNumbers.nextDouble(random, 0.0, 10.0);
         List<Point2D> normalPointsFromLine = GeometryTools.getPerpendicularBisectorSegment(firstPointOnLine, secondPointOnLine, lengthOffset);

         Point2D normalPoint0 = normalPointsFromLine.get(0);
         Point2D normalPoint1 = normalPointsFromLine.get(1);
         Vector2D normalDirection = new Vector2D();
         normalDirection.sub(normalPoint1, normalPoint0);

         assertEquals(2.0 * lengthOffset, normalDirection.length(), Epsilons.ONE_TRILLIONTH);
         assertEquals(0.0, lineDirection.dot(normalDirection), Epsilons.ONE_TRILLIONTH);
         assertEquals(lengthOffset, GeometryTools.distanceFromPointToLine(normalPoint0, firstPointOnLine, secondPointOnLine), Epsilons.ONE_TRILLIONTH);
         assertEquals(lengthOffset, GeometryTools.distanceFromPointToLine(normalPoint1, firstPointOnLine, secondPointOnLine), Epsilons.ONE_TRILLIONTH);
         assertTrue(GeometryTools.isPointOnLeftSideOfLine(normalPoint0, firstPointOnLine, secondPointOnLine));
         assertTrue(GeometryTools.isPointOnRightSideOfLine(normalPoint1, firstPointOnLine, secondPointOnLine));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetTriangleBisector() throws Exception
   {
      Random random = new Random(1176L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D a = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D b = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D c = RandomGeometry.nextPoint2D(random, 10.0, 10.0);

         Vector2D ba = new Vector2D();
         ba.sub(a, b);
         Vector2D bc = new Vector2D();
         bc.sub(c, b);

         double abcAngle = ba.angle(bc);

         Point2D x = new Point2D();
         GeometryTools.getTriangleBisector(a, b, c, x);

         Vector2D bx = new Vector2D();
         bx.sub(x, b);

         double abxAngle = ba.angle(bx);

         assertEquals(0.5 * abcAngle, abxAngle, Epsilons.ONE_TRILLIONTH);
         assertEquals(0.0, GeometryTools.distanceFromPointToLine(x, a, c), Epsilons.ONE_TRILLIONTH);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetOrthogonalProjectionOnLine2D() throws Exception
   {
      Random random = new Random(1176L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D secondPointOnLine = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D expectionProjection = new Point2D();
         expectionProjection.interpolate(firstPointOnLine, secondPointOnLine, RandomNumbers.nextDouble(random, 10.0));
         Vector2D perpendicularToLineDirection = new Vector2D();
         perpendicularToLineDirection.sub(secondPointOnLine, firstPointOnLine);
         perpendicularToLineDirection.normalize();
         GeometryTools.getPerpendicularVector(perpendicularToLineDirection, perpendicularToLineDirection);

         Point2D testPoint = new Point2D();
         testPoint.scaleAdd(RandomNumbers.nextDouble(random, 10.0), perpendicularToLineDirection, expectionProjection);

         Point2D actualProjection = GeometryTools.getOrthogonalProjectionOnLine(testPoint, firstPointOnLine, secondPointOnLine);
         EuclidCoreTestTools.assertTuple2DEquals(expectionProjection, actualProjection, Epsilons.ONE_TRILLIONTH);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetOrthogonalProjectionOnLine3D() throws Exception
   {
      Random random = new Random(1176L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointOnLine = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Vector3D lineDirection = RandomGeometry.nextVector3D(random, RandomNumbers.nextDouble(random, 0.0, 10.0));
         Point3D expectedProjection = new Point3D();
         expectedProjection.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection, pointOnLine);
         Vector3D perpendicularToLineDirection = RandomGeometry.nextOrthogonalVector3D(random, lineDirection, true);

         Point3D testPoint = new Point3D();
         testPoint.scaleAdd(RandomNumbers.nextDouble(random, 10.0), perpendicularToLineDirection, expectedProjection);

         Point3D actualProjection = GeometryTools.getOrthogonalProjectionOnLine(testPoint, pointOnLine, lineDirection);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, Epsilons.ONE_TRILLIONTH);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetPercentageAlongLineSegment2d() throws Exception
   {
      Random random = new Random(23424L);

      // Test on line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D lineSegmentEnd = RandomGeometry.nextPoint2D(random, 10.0, 10.0);

         Point2D pointOnLineSegment = new Point2D();

         // Test between end points
         double expectedPercentage = RandomNumbers.nextDouble(random, 0.0, 1.0);
         pointOnLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         double actualPercentage = GeometryTools.getPercentageAlongLineSegment(pointOnLineSegment, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedPercentage, actualPercentage, Epsilons.ONE_TRILLIONTH);

         // Test before end points
         expectedPercentage = RandomNumbers.nextDouble(random, -10.0, 0.0);
         pointOnLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         actualPercentage = GeometryTools.getPercentageAlongLineSegment(pointOnLineSegment, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedPercentage, actualPercentage, Epsilons.ONE_TRILLIONTH);

         // Test after end points
         expectedPercentage = RandomNumbers.nextDouble(random, 1.0, 10.0);
         pointOnLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         actualPercentage = GeometryTools.getPercentageAlongLineSegment(pointOnLineSegment, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedPercentage, actualPercentage, Epsilons.ONE_TRILLIONTH);
      }

      // Test off line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D lineSegmentEnd = RandomGeometry.nextPoint2D(random, 10.0, 10.0);

         Point2D pointOffLineSegment = new Point2D();
         Vector2D lineSegmentDirection = new Vector2D();
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);
         Vector2D orthogonal = GeometryTools.getPerpendicularVector(lineSegmentDirection);
         orthogonal.normalize();

         // Test between end points
         double expectedPercentage = RandomNumbers.nextDouble(random, 0.0, 1.0);
         pointOffLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         pointOffLineSegment.scaleAdd(RandomNumbers.nextDouble(random, 10.0), orthogonal, pointOffLineSegment);
         double actualPercentage = GeometryTools.getPercentageAlongLineSegment(pointOffLineSegment, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedPercentage, actualPercentage, Epsilons.ONE_TRILLIONTH);

         // Test before end points
         expectedPercentage = RandomNumbers.nextDouble(random, -10.0, 0.0);
         pointOffLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         pointOffLineSegment.scaleAdd(RandomNumbers.nextDouble(random, 10.0), orthogonal, pointOffLineSegment);
         actualPercentage = GeometryTools.getPercentageAlongLineSegment(pointOffLineSegment, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedPercentage, actualPercentage, Epsilons.ONE_TRILLIONTH);

         // Test after end points
         expectedPercentage = RandomNumbers.nextDouble(random, 1.0, 10.0);
         pointOffLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         pointOffLineSegment.scaleAdd(RandomNumbers.nextDouble(random, 10.0), orthogonal, pointOffLineSegment);
         actualPercentage = GeometryTools.getPercentageAlongLineSegment(pointOffLineSegment, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedPercentage, actualPercentage, Epsilons.ONE_TRILLIONTH);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetPercentageAlongLineSegment3d() throws Exception
   {
      Random random = new Random(23424L);

      // Test on line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Point3D lineSegmentEnd = RandomGeometry.nextPoint3D(random, -10.0, 10.0);

         Point3D pointOnLineSegment = new Point3D();

         // Test between end points
         double expectedPercentage = RandomNumbers.nextDouble(random, 0.0, 1.0);
         pointOnLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         double actualPercentage = GeometryTools.getPercentageAlongLineSegment(pointOnLineSegment, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedPercentage, actualPercentage, Epsilons.ONE_TRILLIONTH);

         // Test before end points
         expectedPercentage = RandomNumbers.nextDouble(random, -10.0, 0.0);
         pointOnLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         actualPercentage = GeometryTools.getPercentageAlongLineSegment(pointOnLineSegment, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedPercentage, actualPercentage, Epsilons.ONE_TRILLIONTH);

         // Test after end points
         expectedPercentage = RandomNumbers.nextDouble(random, 1.0, 10.0);
         pointOnLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         actualPercentage = GeometryTools.getPercentageAlongLineSegment(pointOnLineSegment, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedPercentage, actualPercentage, Epsilons.ONE_TRILLIONTH);
      }

      // Test off line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Point3D lineSegmentEnd = RandomGeometry.nextPoint3D(random, -10.0, 10.0);

         Point3D pointOffLineSegment = new Point3D();
         Vector3D lineSegmentDirection = new Vector3D();
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineSegmentDirection.normalize();
         Vector3D orthogonal = RandomGeometry.nextOrthogonalVector3D(random, lineSegmentDirection, true);

         // Test between end points
         double expectedPercentage = RandomNumbers.nextDouble(random, 0.0, 1.0);
         pointOffLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         pointOffLineSegment.scaleAdd(RandomNumbers.nextDouble(random, 10.0), orthogonal, pointOffLineSegment);
         double actualPercentage = GeometryTools.getPercentageAlongLineSegment(pointOffLineSegment, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedPercentage, actualPercentage, Epsilons.ONE_TRILLIONTH);

         // Test before end points
         expectedPercentage = RandomNumbers.nextDouble(random, -10.0, 0.0);
         pointOffLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         pointOffLineSegment.scaleAdd(RandomNumbers.nextDouble(random, 10.0), orthogonal, pointOffLineSegment);
         actualPercentage = GeometryTools.getPercentageAlongLineSegment(pointOffLineSegment, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedPercentage, actualPercentage, Epsilons.ONE_TRILLIONTH);

         // Test after end points
         expectedPercentage = RandomNumbers.nextDouble(random, 1.0, 10.0);
         pointOffLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         pointOffLineSegment.scaleAdd(RandomNumbers.nextDouble(random, 10.0), orthogonal, pointOffLineSegment);
         actualPercentage = GeometryTools.getPercentageAlongLineSegment(pointOffLineSegment, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedPercentage, actualPercentage, Epsilons.ONE_TRILLIONTH);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetOrthogonalProjectionOnLineSegment2D() throws Exception
   {
      Random random = new Random(232L);

      for (int i = 0; i < 1000; i++)
      {
         Point2D lineSegmentStart = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D lineSegmentEnd = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(lineSegmentEnd, lineSegmentStart);
         GeometryTools.getPerpendicularVector(orthogonal, orthogonal);
         orthogonal.normalize();
         Point2D expectedProjection = new Point2D();
         Point2D testPoint = new Point2D();

         // Between end points
         expectedProjection.interpolate(lineSegmentStart, lineSegmentEnd, RandomNumbers.nextDouble(random, 0.0, 1.0));
         testPoint.scaleAdd(RandomNumbers.nextDouble(random, 10.0), orthogonal, expectedProjection);
         Point2D actualProjection = GeometryTools.getOrthogonalProjectionOnLineSegment(testPoint, lineSegmentStart, lineSegmentEnd);
         EuclidCoreTestTools.assertTuple2DEquals(expectedProjection, actualProjection, Epsilons.ONE_TRILLIONTH);

         // Before end points
         expectedProjection.interpolate(lineSegmentStart, lineSegmentEnd, RandomNumbers.nextDouble(random, -10.0, 0.0));
         testPoint.scaleAdd(RandomNumbers.nextDouble(random, 10.0), orthogonal, expectedProjection);
         expectedProjection.set(lineSegmentStart);
         actualProjection = GeometryTools.getOrthogonalProjectionOnLineSegment(testPoint, lineSegmentStart, lineSegmentEnd);
         EuclidCoreTestTools.assertTuple2DEquals(expectedProjection, actualProjection, Epsilons.ONE_TRILLIONTH);

         // After end points
         expectedProjection.interpolate(lineSegmentStart, lineSegmentEnd, RandomNumbers.nextDouble(random, 1.0, 10.0));
         testPoint.scaleAdd(RandomNumbers.nextDouble(random, 10.0), orthogonal, expectedProjection);
         expectedProjection.set(lineSegmentEnd);
         actualProjection = GeometryTools.getOrthogonalProjectionOnLineSegment(testPoint, lineSegmentStart, lineSegmentEnd);
         EuclidCoreTestTools.assertTuple2DEquals(expectedProjection, actualProjection, Epsilons.ONE_TRILLIONTH);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetOrthogonalProjectionOnLineSegment3D() throws Exception
   {
      Random random = new Random(232L);

      for (int i = 0; i < 1000; i++)
      {
         Point3D lineSegmentStart = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Point3D lineSegmentEnd = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Vector3D lineSegmentDirection = new Vector3D();
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);
         Vector3D orthogonal = RandomGeometry.nextOrthogonalVector3D(random, lineSegmentDirection, true);
         Point3D expectedProjection = new Point3D();
         Point3D testPoint = new Point3D();

         // Between end points
         expectedProjection.interpolate(lineSegmentStart, lineSegmentEnd, RandomNumbers.nextDouble(random, 0.0, 1.0));
         testPoint.scaleAdd(RandomNumbers.nextDouble(random, 10.0), orthogonal, expectedProjection);
         Point3D actualProjection = GeometryTools.getOrthogonalProjectionOnLineSegment(testPoint, lineSegmentStart, lineSegmentEnd);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, Epsilons.ONE_TRILLIONTH);

         // Before end points
         expectedProjection.interpolate(lineSegmentStart, lineSegmentEnd, RandomNumbers.nextDouble(random, -10.0, 0.0));
         testPoint.scaleAdd(RandomNumbers.nextDouble(random, 10.0), orthogonal, expectedProjection);
         expectedProjection.set(lineSegmentStart);
         actualProjection = GeometryTools.getOrthogonalProjectionOnLineSegment(testPoint, lineSegmentStart, lineSegmentEnd);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, Epsilons.ONE_TRILLIONTH);

         // After end points
         expectedProjection.interpolate(lineSegmentStart, lineSegmentEnd, RandomNumbers.nextDouble(random, 1.0, 10.0));
         testPoint.scaleAdd(RandomNumbers.nextDouble(random, 10.0), orthogonal, expectedProjection);
         expectedProjection.set(lineSegmentEnd);
         actualProjection = GeometryTools.getOrthogonalProjectionOnLineSegment(testPoint, lineSegmentStart, lineSegmentEnd);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, Epsilons.ONE_TRILLIONTH);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetXYDistance() throws Exception
   {
      Random random = new Random(232L);

      for (int i = 0; i < 1000; i++)
      {
         Point3D firstPoint3d = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
         Point3D secondPoint3d = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
         Point2D firstPoint2d = new Point2D(firstPoint3d.getX(), firstPoint3d.getY());
         Point2D secondPoint2d = new Point2D(secondPoint3d.getX(), secondPoint3d.getY());
         double expectedDistance = firstPoint2d.distance(secondPoint2d);
         double actualDistance = GeometryTools.getXYDistance(firstPoint3d, secondPoint3d);
         assertEquals(expectedDistance, actualDistance, Epsilons.ONE_TRILLIONTH);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetAngleFromFirstToSecondVector3D() throws Exception
   {
      Random random = new Random(1176L);
      // Test getRotationBasedOnNormal(AxisAngle4d rotationToPack, Vector3d normalVector3d)
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D firstVector = new Vector3D(0.0, 0.0, 1.0);
         double expectedAngle = RandomNumbers.nextDouble(random, 0.0, Math.PI);
         Vector3D expectedAxis = RandomGeometry.nextOrthogonalVector3D(random, firstVector, true);
         AxisAngle expectedAxisAngle = new AxisAngle(expectedAxis, expectedAngle);
         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(expectedAxisAngle);

         Vector3D secondVector = new Vector3D();
         rotationMatrix.transform(firstVector, secondVector);
         secondVector.scale(RandomNumbers.nextDouble(random, 0.0, 10.0));

         double actualAngle = GeometryTools.getAngleFromFirstToSecondVector(firstVector, secondVector);

         assertEquals(expectedAngle, actualAngle, Epsilons.ONE_TRILLIONTH);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testAreVectorsCollinear3D() throws Exception
   {
      Random random = new Random(232L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D firstVector = RandomGeometry.nextVector3D(random, RandomNumbers.nextDouble(random, 0.0, 10.0));

         Vector3D rotationAxis = RandomGeometry.nextOrthogonalVector3D(random, firstVector, true);
         double angleEpsilon = RandomNumbers.nextDouble(random, 0.0, Math.PI / 2.0);
         double rotationAngle = RandomNumbers.nextDouble(random, 0.0, Math.PI / 2.0);

         AxisAngle rotationAxisAngle = new AxisAngle(rotationAxis, rotationAngle);
         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(rotationAxisAngle);

         Vector3D secondVector = new Vector3D();
         rotationMatrix.transform(firstVector, secondVector);
         secondVector.normalize();
         secondVector.scale(RandomNumbers.nextDouble(random, 0.0, 10.0));

         assertEquals(rotationAngle < angleEpsilon, GeometryTools.areVectorsCollinear(firstVector, secondVector, angleEpsilon));
      }

      // Try again with small values
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D firstVector = RandomGeometry.nextVector3D(random, 1.0);

         Vector3D rotationAxis = RandomGeometry.nextOrthogonalVector3D(random, firstVector, true);
         double angleEpsilon = RandomNumbers.nextDouble(random, 0.0, Epsilons.ONE_MILLIONTH * Math.PI / 2.0);
         double rotationAngle = RandomNumbers.nextDouble(random, 0.0, Epsilons.ONE_MILLIONTH * Math.PI / 2.0);
         if (Math.abs(rotationAngle - angleEpsilon) < 1.0e-7)
            continue; // This is the limit of accuracy.

         AxisAngle rotationAxisAngle = new AxisAngle(rotationAxis, rotationAngle);
         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(rotationAxisAngle);

         Vector3D secondVector = new Vector3D();
         rotationMatrix.transform(firstVector, secondVector);

         assertEquals(rotationAngle < angleEpsilon, GeometryTools.areVectorsCollinear(firstVector, secondVector, angleEpsilon));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testAreVectorsCollinear2D() throws Exception
   {
      Random random = new Random(232L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector2D firstVector = RandomGeometry.nextVector2D(random, RandomNumbers.nextDouble(random, 0.0, 10.0));

         double angleEpsilon = RandomNumbers.nextDouble(random, 0.0, Math.PI / 2.0);
         double rotationAngle = RandomNumbers.nextDouble(random, 0.0, Math.PI / 2.0);

         Vector2D secondVector = new Vector2D();
         GeometryTools.rotateTuple2d(rotationAngle, firstVector, secondVector);
         secondVector.normalize();
         secondVector.scale(RandomNumbers.nextDouble(random, 0.0, 10.0));

         assertEquals(rotationAngle < angleEpsilon, GeometryTools.areVectorsCollinear(firstVector, secondVector, angleEpsilon));
      }

      // Try again with small values
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector2D firstVector = RandomGeometry.nextVector2D(random, RandomNumbers.nextDouble(random, 0.0, 10.0));

         double angleEpsilon = RandomNumbers.nextDouble(random, 0.0, Epsilons.ONE_MILLIONTH * Math.PI / 2.0);
         double rotationAngle = RandomNumbers.nextDouble(random, 0.0, Epsilons.ONE_MILLIONTH * Math.PI / 2.0);
         if (Math.abs(rotationAngle - angleEpsilon) < 1.0e-7)
            continue; // This is the limit of accuracy.

         Vector2D secondVector = new Vector2D();
         GeometryTools.rotateTuple2d(rotationAngle, firstVector, secondVector);

         assertEquals(rotationAngle < angleEpsilon, GeometryTools.areVectorsCollinear(firstVector, secondVector, angleEpsilon));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testAreLinesCollinear2D() throws Exception
   {
      Random random = new Random(232L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector2D lineDirection1 = RandomGeometry.nextVector2D(random, RandomNumbers.nextDouble(random, 0.0, 10.0));

         double angleEpsilon = RandomNumbers.nextDouble(random, 0.0, Math.PI / 2.0);
         double rotationAngle = RandomNumbers.nextDouble(random, 0.0, Math.PI / 2.0);

         Vector2D lineDirection2 = new Vector2D();
         GeometryTools.rotateTuple2d(rotationAngle, lineDirection1, lineDirection2);
         lineDirection2.normalize();
         lineDirection2.scale(RandomNumbers.nextDouble(random, 0.0, 10.0));

         Point2D firstPointOnLine1 = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D secondPointOnLine1 = new Point2D();
         secondPointOnLine1.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection1, firstPointOnLine1);

         Vector2D orthogonal = GeometryTools.getPerpendicularVector(lineDirection1);
         orthogonal.normalize();
         double distance = RandomNumbers.nextDouble(random, 0.0, 10.0);
         double distanceEspilon = RandomNumbers.nextDouble(random, 0.0, 10.0);

         Point2D firstPointOnLine2 = new Point2D();
         firstPointOnLine2.scaleAdd(distance, orthogonal, firstPointOnLine1);
         Point2D secondPointOnLine2 = new Point2D();
         secondPointOnLine2.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection2, firstPointOnLine2);

         boolean expectedCollinear = rotationAngle < angleEpsilon && distance < distanceEspilon;
         boolean actualCollinear = GeometryTools.areLinesCollinear(firstPointOnLine1, secondPointOnLine1, firstPointOnLine2, secondPointOnLine2, angleEpsilon, distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
      }

      // Test only the distance with parallel line segments.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector2D lineDirection = RandomGeometry.nextVector2D(random, RandomNumbers.nextDouble(random, 0.0, 10.0));
         Vector2D orthogonal = GeometryTools.getPerpendicularVector(lineDirection);
         orthogonal.normalize();

         double angleEpsilon = Epsilons.ONE_MILLIONTH;

         Point2D firstPointOnLine1 = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D secondPointOnLine1 = new Point2D();
         secondPointOnLine1.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection, firstPointOnLine1);

         double distance = RandomNumbers.nextDouble(random, 0.0, 10.0);
         double distanceEspilon = RandomNumbers.nextDouble(random, 0.0, 10.0);

         Point2D firstPointOnLine2 = new Point2D();
         firstPointOnLine2.scaleAdd(distance, orthogonal, firstPointOnLine1);
         Point2D secondPointOnLine2 = new Point2D();
         secondPointOnLine2.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection, firstPointOnLine2);
         firstPointOnLine2.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection, firstPointOnLine2);

         boolean expectedCollinear = distance < distanceEspilon;
         boolean actualCollinear;
         actualCollinear = GeometryTools.areLinesCollinear(firstPointOnLine1, secondPointOnLine1, firstPointOnLine2, secondPointOnLine2, angleEpsilon, distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
         actualCollinear = GeometryTools.areLinesCollinear(firstPointOnLine1, secondPointOnLine1, secondPointOnLine2, firstPointOnLine2, angleEpsilon, distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
         actualCollinear = GeometryTools.areLinesCollinear(secondPointOnLine1, firstPointOnLine1, secondPointOnLine2, firstPointOnLine2, angleEpsilon, distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
         actualCollinear = GeometryTools.areLinesCollinear(secondPointOnLine1, firstPointOnLine1, firstPointOnLine2, secondPointOnLine2, angleEpsilon, distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);

         actualCollinear = GeometryTools.areLinesCollinear(firstPointOnLine2, secondPointOnLine2, firstPointOnLine1, secondPointOnLine1, angleEpsilon, distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);                                                                                
         actualCollinear = GeometryTools.areLinesCollinear(secondPointOnLine2, firstPointOnLine2, firstPointOnLine1, secondPointOnLine1, angleEpsilon, distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);                                                                                
         actualCollinear = GeometryTools.areLinesCollinear(secondPointOnLine2, firstPointOnLine2, secondPointOnLine1, firstPointOnLine1, angleEpsilon, distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);                                                                                
         actualCollinear = GeometryTools.areLinesCollinear(firstPointOnLine2, secondPointOnLine2, secondPointOnLine1, firstPointOnLine1, angleEpsilon, distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testAreLinesCollinear3D() throws Exception
   {
      Random random = new Random(2312L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D lineDirection1 = RandomGeometry.nextVector3D(random, RandomNumbers.nextDouble(random, 0.0, 10.0));

         double angleEpsilon = RandomNumbers.nextDouble(random, 0.0, Math.PI / 2.0);
         double rotationAngle = RandomNumbers.nextDouble(random, 0.0, Math.PI / 2.0);
         Vector3D orthogonal = RandomGeometry.nextOrthogonalVector3D(random, lineDirection1, true);
         AxisAngle axisAngle = new AxisAngle(orthogonal, rotationAngle);

         Vector3D lineDirection2 = new Vector3D();
         axisAngle.transform(lineDirection1, lineDirection2);
         lineDirection2.normalize();
         lineDirection2.scale(RandomNumbers.nextDouble(random, 0.0, 10.0));

         Point3D firstPointOnLine1 = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Point3D secondPointOnLine1 = new Point3D();
         secondPointOnLine1.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection1, firstPointOnLine1);

         double distance = RandomNumbers.nextDouble(random, 0.0, 10.0);
         double distanceEspilon = RandomNumbers.nextDouble(random, 0.0, 10.0);

         Point3D firstPointOnLine2 = new Point3D();
         firstPointOnLine2.scaleAdd(distance, orthogonal, firstPointOnLine1);
         Point3D secondPointOnLine2 = new Point3D();
         secondPointOnLine2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection2, firstPointOnLine2);

         boolean expectedCollinear = rotationAngle < angleEpsilon && distance < distanceEspilon;
         boolean actualCollinear = GeometryTools.areLinesCollinear(firstPointOnLine1, secondPointOnLine1, firstPointOnLine2, secondPointOnLine2, angleEpsilon, distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
         actualCollinear = GeometryTools.areLinesCollinear(firstPointOnLine1, lineDirection1, firstPointOnLine2, lineDirection2, angleEpsilon, distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
      }

      // Test only the distance with parallel line segments.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D lineDirection = RandomGeometry.nextVector3D(random, RandomNumbers.nextDouble(random, 0.0, 10.0));
         Vector3D orthogonal = RandomGeometry.nextOrthogonalVector3D(random, lineDirection, true);

         double angleEpsilon = Epsilons.ONE_MILLIONTH;

         Point3D firstPointOnLine1 = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         Point3D secondPointOnLine1 = new Point3D();
         secondPointOnLine1.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection, firstPointOnLine1);

         double distance = RandomNumbers.nextDouble(random, 0.0, 10.0);
         double distanceEspilon = RandomNumbers.nextDouble(random, 0.0, 10.0);

         Point3D firstPointOnLine2 = new Point3D();
         firstPointOnLine2.scaleAdd(distance, orthogonal, firstPointOnLine1);
         Point3D secondPointOnLine2 = new Point3D();
         secondPointOnLine2.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection, firstPointOnLine2);
         firstPointOnLine2.scaleAdd(RandomNumbers.nextDouble(random, 10.0), lineDirection, firstPointOnLine2);

         boolean expectedCollinear = distance < distanceEspilon;
         boolean actualCollinear;
         actualCollinear = GeometryTools.areLinesCollinear(firstPointOnLine1, secondPointOnLine1, firstPointOnLine2, secondPointOnLine2, angleEpsilon, distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
         actualCollinear = GeometryTools.areLinesCollinear(firstPointOnLine1, secondPointOnLine1, secondPointOnLine2, firstPointOnLine2, angleEpsilon, distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
         actualCollinear = GeometryTools.areLinesCollinear(secondPointOnLine1, firstPointOnLine1, secondPointOnLine2, firstPointOnLine2, angleEpsilon, distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
         actualCollinear = GeometryTools.areLinesCollinear(secondPointOnLine1, firstPointOnLine1, firstPointOnLine2, secondPointOnLine2, angleEpsilon, distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);

         actualCollinear = GeometryTools.areLinesCollinear(firstPointOnLine2, secondPointOnLine2, firstPointOnLine1, secondPointOnLine1, angleEpsilon, distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);                                                                                
         actualCollinear = GeometryTools.areLinesCollinear(secondPointOnLine2, firstPointOnLine2, firstPointOnLine1, secondPointOnLine1, angleEpsilon, distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);                                                                                
         actualCollinear = GeometryTools.areLinesCollinear(secondPointOnLine2, firstPointOnLine2, secondPointOnLine1, firstPointOnLine1, angleEpsilon, distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);                                                                                
         actualCollinear = GeometryTools.areLinesCollinear(firstPointOnLine2, secondPointOnLine2, secondPointOnLine1, firstPointOnLine1, angleEpsilon, distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testRotateTuple2d() throws Exception
   {
      Random random = new Random(232L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector2D original2d = RandomGeometry.nextVector2D(random, RandomNumbers.nextDouble(random, 10.0));
         Vector3D original3d = new Vector3D(original2d.getX(), original2d.getY(), 0.0);

         double yaw = RandomNumbers.nextDouble(random, 3.0 * Math.PI);

         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.setToYawMatrix(yaw);

         Vector2D expectedTransformed2d = new Vector2D();
         Vector3D expectedTransformed3d = new Vector3D();
         rotationMatrix.transform(original3d, expectedTransformed3d);
         expectedTransformed2d.set(expectedTransformed3d.getX(), expectedTransformed3d.getY());

         Vector2D actualTransformed2d = new Vector2D();
         GeometryTools.rotateTuple2d(yaw, original2d, actualTransformed2d);

         EuclidCoreTestTools.assertTuple2DEquals(expectedTransformed2d, actualTransformed2d, Epsilons.ONE_TRILLIONTH);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testArePlanesCoincident() throws Exception
   {
      Random random = new Random();//232L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointOnPlane1 = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
         Vector3D planeNormal1 = RandomGeometry.nextVector3D(random, 1.0);

         Point3D pointOnPlane2 = new Point3D();
         Vector3D planeNormal2 = new Vector3D();

         double distanceEpsilon = RandomNumbers.nextDouble(random, 1.0);
         double distanceBetweenPlanes = RandomNumbers.nextDouble(random, 1.0);

         pointOnPlane2.scaleAdd(distanceBetweenPlanes, planeNormal1, pointOnPlane1);

         Vector3D rotationAxis = RandomGeometry.nextOrthogonalVector3D(random, planeNormal1, true);
         double angleEpsilon = RandomNumbers.nextDouble(random, 0.0, Math.PI / 2.0);
         double rotationAngle = RandomNumbers.nextDouble(random, 0.0, Math.PI / 2.0);

         AxisAngle rotationAxisAngle = new AxisAngle(rotationAxis, rotationAngle);
         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(rotationAxisAngle);

         rotationMatrix.transform(planeNormal1, planeNormal2);

         boolean expectedCoincidentResult = Math.abs(distanceBetweenPlanes) < distanceEpsilon && rotationAngle < angleEpsilon;
         boolean actualCoincidentResult = GeometryTools.arePlanesCoincident(pointOnPlane1, planeNormal1, pointOnPlane2, planeNormal2, angleEpsilon,
                                                                        distanceEpsilon);
         assertEquals(expectedCoincidentResult, actualCoincidentResult);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetOrthogonalProjectionOnPlane() throws Exception
   {
      Random random = new Random(23423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointOnPlane = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
         Vector3D planeNormal = RandomGeometry.nextVector3D(random, RandomNumbers.nextDouble(random, 0.0, 10.0));

         Vector3D parallelToPlane = RandomGeometry.nextOrthogonalVector3D(random, planeNormal, true);
         Point3D expectedProjection = new Point3D();
         expectedProjection.scaleAdd(RandomNumbers.nextDouble(random, 10.0), parallelToPlane, pointOnPlane);

         Point3D pointToProject = new Point3D();
         double distanceOffPlane = RandomNumbers.nextDouble(random, 10.0);
         pointToProject.scaleAdd(distanceOffPlane, planeNormal, expectedProjection);

         Point3D actualProjection = GeometryTools.getOrthogonalProjectionOnPlane(pointToProject, pointOnPlane, planeNormal);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, Epsilons.ONE_TRILLIONTH);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testIsZero() throws Exception
   {
      Random random = new Random(23423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double x = RandomNumbers.nextDouble(random, 0.0, 10.0);
         double y = RandomNumbers.nextDouble(random, 0.0, 10.0);
         double z = RandomNumbers.nextDouble(random, 0.0, 10.0);
         double epsilon = RandomNumbers.nextDouble(random, 0.0, 10.0);

         boolean isTuple2dZero = x < epsilon && y < epsilon;
         boolean isTuple3dZero = x < epsilon && y < epsilon && z < epsilon;

         assertEquals(isTuple2dZero, GeometryTools.isZero(new Point2D(x, y), epsilon));
         assertEquals(isTuple2dZero, GeometryTools.isZero(new Point2D(-x, y), epsilon));
         assertEquals(isTuple2dZero, GeometryTools.isZero(new Point2D(-x, -y), epsilon));
         assertEquals(isTuple2dZero, GeometryTools.isZero(new Point2D(x, -y), epsilon));
         assertEquals(isTuple2dZero, GeometryTools.isZero(new Point2D(x, y), -epsilon));
         assertEquals(isTuple2dZero, GeometryTools.isZero(new Point2D(-x, y), -epsilon));
         assertEquals(isTuple2dZero, GeometryTools.isZero(new Point2D(-x, -y), -epsilon));
         assertEquals(isTuple2dZero, GeometryTools.isZero(new Point2D(x, -y), -epsilon));

         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(x, y, z), epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(x, y, -z), epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(x, -y, z), epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(x, -y, -z), epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(-x, y, z), epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(-x, y, -z), epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(-x, -y, z), epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(-x, -y, -z), epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(x, y, z), -epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(x, y, -z), -epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(x, -y, z), -epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(x, -y, -z), -epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(-x, y, z), -epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(-x, y, -z), -epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(-x, -y, z), -epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(-x, -y, -z), -epsilon));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetIntersectionBetweenTwoPlanes() throws Exception
   {
      Random random = new Random(23423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointOnPlane1 = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
         Vector3D planeNormal1 = RandomGeometry.nextVector3D(random, RandomNumbers.nextDouble(random, 0.0, 10.0));

         Vector3D firstParallelToPlane1 = RandomGeometry.nextOrthogonalVector3D(random, planeNormal1, true);
         Vector3D secondParallelToPlane1 = RandomGeometry.nextOrthogonalVector3D(random, planeNormal1, true);

         Point3D firstPointOnIntersection = new Point3D();
         Point3D secondPointOnIntersection = new Point3D();
         firstPointOnIntersection.scaleAdd(RandomNumbers.nextDouble(random, 10.0), firstParallelToPlane1, pointOnPlane1);
         secondPointOnIntersection.scaleAdd(RandomNumbers.nextDouble(random, 10.0), secondParallelToPlane1, firstPointOnIntersection);

         Vector3D expectedIntersectionDirection = new Vector3D();
         expectedIntersectionDirection.sub(secondPointOnIntersection, firstPointOnIntersection);
         expectedIntersectionDirection.normalize();

         double rotationAngle = RandomNumbers.nextDouble(random, Math.PI);
         AxisAngle rotationAxisAngle = new AxisAngle(expectedIntersectionDirection, rotationAngle);
         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(rotationAxisAngle);

         Vector3D planeNormal2 = new Vector3D();
         rotationMatrix.transform(planeNormal1, planeNormal2);
         planeNormal2.scale(RandomNumbers.nextDouble(random, 0.0, 10.0));
         Point3D pointOnPlane2 = new Point3D();

         Vector3D parallelToPlane2 = RandomGeometry.nextOrthogonalVector3D(random, planeNormal2, true);
         pointOnPlane2.scaleAdd(RandomNumbers.nextDouble(random, 10.0), parallelToPlane2, firstPointOnIntersection);

         Point3D actualPointOnIntersection = new Point3D();
         Vector3D actualIntersectionDirection = new Vector3D();

         boolean success = GeometryTools.getIntersectionBetweenTwoPlanes(pointOnPlane1, planeNormal1, pointOnPlane2, planeNormal2, actualPointOnIntersection, actualIntersectionDirection);
         boolean areParallel = GeometryTools.areVectorsCollinear(planeNormal1, planeNormal2, Epsilons.ONE_MILLIONTH);
         assertNotEquals(areParallel, success);
         if (areParallel)
            continue;

         if (expectedIntersectionDirection.dot(actualIntersectionDirection) < 0.0)
            expectedIntersectionDirection.negate();

         String message = "Angle between vectors " + expectedIntersectionDirection.angle(actualIntersectionDirection);
         assertTrue(message, GeometryTools.areVectorsCollinear(expectedIntersectionDirection, actualIntersectionDirection, Epsilons.ONE_TEN_MILLIONTH));
         assertEquals(1.0, actualIntersectionDirection.length(), Epsilons.ONE_TRILLIONTH);

         if (planeNormal1.dot(planeNormal2) < 0.0)
            planeNormal1.negate();

         double epsilon = Epsilons.ONE_BILLIONTH;

         if (planeNormal1.angle(planeNormal2) < 0.15)
            epsilon = Epsilons.ONE_HUNDRED_MILLIONTH;
         if (planeNormal1.angle(planeNormal2) < 0.05)
            epsilon = Epsilons.ONE_TEN_MILLIONTH;
         if (planeNormal1.angle(planeNormal2) < 0.03)
            epsilon = Epsilons.ONE_MILLIONTH;
         if (planeNormal1.angle(planeNormal2) < 0.001)
            epsilon = Epsilons.ONE_HUNDRED_THOUSANDTH;
         if (planeNormal1.angle(planeNormal2) < 0.0001)
            epsilon = Epsilons.ONE_TEN_THOUSANDTH;
         if (planeNormal1.angle(planeNormal2) < 0.00005)
            epsilon = Epsilons.ONE_THOUSANDTH;

//         System.out.println("angle: " + planeNormal1.angle(planeNormal2) + ", distance: " + pointOnPlane1.distance(pointOnPlane2));
         assertEquals(0.0, GeometryTools.distanceFromPointToLine(actualPointOnIntersection, firstPointOnIntersection, expectedIntersectionDirection), epsilon);
      }
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(GeometryTools.class, GeometryToolsTest.class);
   }
}
