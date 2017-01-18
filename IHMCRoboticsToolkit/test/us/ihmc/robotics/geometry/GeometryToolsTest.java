package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.robotics.math.Epsilons;
import us.ihmc.robotics.math.exceptions.UndefinedOperationException;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.MutationTestingTools;
import us.ihmc.tools.thread.RunnableThatThrows;

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
      Point3d a = new Point3d(5.8, 9.9, 4.5);
      Point3d b = new Point3d(5.6, 8.1, 5.5);
      double expectedReturn1 = 5.7;
      double expectedReturn2 = 9.0;
      double expectedReturn3 = 5;
      Point3d actualReturn = GeometryTools.averagePoints(a, b);
      double actualReturn1 = actualReturn.getX();
      double actualReturn2 = actualReturn.getY();
      double actualReturn3 = actualReturn.getZ();
      assertEquals("return value", expectedReturn1, actualReturn1, EPSILON);
      assertEquals("return value", expectedReturn2, actualReturn2, EPSILON);
      assertEquals("return value", expectedReturn3, actualReturn3, EPSILON);

      Point3d a1 = new Point3d(-5, -5, -5);
      Point3d b1 = new Point3d(-5, -5, -5);
      double expectedReturn11 = -5;
      double expectedReturn12 = -5;
      double expectedReturn13 = -5;
      Point3d actualReturn01 = GeometryTools.averagePoints(a1, b1);
      double actualReturn11 = actualReturn01.getX();
      double actualReturn12 = actualReturn01.getY();
      double actualReturn13 = actualReturn01.getZ();
      assertEquals("return value", expectedReturn11, actualReturn11, EPSILON);
      assertEquals("return value", expectedReturn12, actualReturn12, EPSILON);
      assertEquals("return value", expectedReturn13, actualReturn13, EPSILON);

      Point3d a2 = new Point3d(0, 0, 0);
      Point3d b2 = new Point3d(0, 0, 0);
      double expectedReturn21 = 0;
      double expectedReturn22 = 0;
      double expectedReturn23 = 0;
      Point3d actualReturn02 = GeometryTools.averagePoints(a2, b2);
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
      ArrayList<Point2d> points = new ArrayList<Point2d>();
      Point2d a = new Point2d(1.0, 4.6);
      Point2d b = new Point2d(5.2, 6.0);
      Point2d c = new Point2d(3.7, 2.0);
      points.add(a);
      points.add(b);
      points.add(c);
      double expectedReturn1 = 3.3;
      double expectedReturn2 = 4.2;
      Point2d actualReturn = GeometryTools.averagePoint2ds(points);
      double actualReturn1 = actualReturn.getX();
      double actualReturn2 = actualReturn.getY();
      assertEquals("return value", expectedReturn1, actualReturn1, EPSILON);
      assertEquals("return value", expectedReturn2, actualReturn2, EPSILON);

      ArrayList<Point2d> points1 = new ArrayList<Point2d>();
      Point2d a1 = new Point2d(0.0, 0.0);
      Point2d b1 = new Point2d(0.0, 0.0);
      Point2d c1 = new Point2d(0.0, 0.0);
      points1.add(a1);
      points1.add(b1);
      points1.add(c1);
      double expectedReturn11 = 0.0;
      double expectedReturn12 = 0.0;
      Point2d actualReturn01 = GeometryTools.averagePoint2ds(points1);
      double actualReturn11 = actualReturn01.getX();
      double actualReturn12 = actualReturn01.getY();
      assertEquals("return value", expectedReturn11, actualReturn11, EPSILON);
      assertEquals("return value", expectedReturn12, actualReturn12, EPSILON);

      ArrayList<Point2d> points2 = new ArrayList<Point2d>();
      Point2d a2 = new Point2d(-1.0, -4.6);
      Point2d b2 = new Point2d(-5.2, -6.0);
      Point2d c2 = new Point2d(-3.7, -2.0);
      points2.add(a2);
      points2.add(b2);
      points2.add(c2);
      double expectedReturn21 = -3.3;
      double expectedReturn22 = -4.2;
      Point2d actualReturn02 = GeometryTools.averagePoint2ds(points2);
      double actualReturn21 = actualReturn02.getX();
      double actualReturn22 = actualReturn02.getY();
      assertEquals("return value", expectedReturn21, actualReturn21, EPSILON);
      assertEquals("return value", expectedReturn22, actualReturn22, EPSILON);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAveragePoints2()
   {
      ArrayList<Point3d> points = new ArrayList<Point3d>();
      Point3d a = new Point3d(4.3, 5.6, 3.6);
      Point3d b = new Point3d(8.1, 8.4, 0.0);
      Point3d c = new Point3d(5.6, 1.0, 4.5);
      points.add(a);
      points.add(b);
      points.add(c);
      double expectedReturn1 = 6.0;
      double expectedReturn2 = 5.0;
      double expectedReturn3 = 2.7;
      Point3d actualReturn = GeometryTools.averagePoint3ds(points);
      double actualReturn1 = actualReturn.getX();
      double actualReturn2 = actualReturn.getY();
      double actualReturn3 = actualReturn.getZ();
      assertEquals("return value", expectedReturn1, actualReturn1, EPSILON);
      assertEquals("return value", expectedReturn2, actualReturn2, EPSILON);
      assertEquals("return value", expectedReturn3, actualReturn3, EPSILON);

      ArrayList<Point3d> points1 = new ArrayList<Point3d>();
      Point3d a1 = new Point3d(0.0, 0.0, 0.0);
      Point3d b1 = new Point3d(0.0, 0.0, 0.0);
      Point3d c1 = new Point3d(0.0, 0.0, 0.0);
      points1.add(a1);
      points1.add(b1);
      points1.add(c1);
      double expectedReturn11 = 0.0;
      double expectedReturn12 = 0.0;
      double expectedReturn13 = 0.0;
      Point3d actualReturn01 = GeometryTools.averagePoint3ds(points1);
      double actualReturn11 = actualReturn01.getX();
      double actualReturn12 = actualReturn01.getY();
      double actualReturn13 = actualReturn01.getZ();
      assertEquals("return value", expectedReturn11, actualReturn11, EPSILON);
      assertEquals("return value", expectedReturn12, actualReturn12, EPSILON);
      assertEquals("return value", expectedReturn13, actualReturn13, EPSILON);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPoints()
   {
      double[] a = {2.0, 2.0};
      double[] b = {6.0, -1.0};
      double expectedReturn = 5.0;
      double actualReturn = GeometryTools.distanceBetweenPoints(a, b);
      assertEquals("return value", expectedReturn, actualReturn, Double.MIN_VALUE);

      double[] a1 = {2.5, 5.1};
      double[] b1 = {9.3, 10.7};
      double expectedReturn1 = 8.80908621822;
      double actualReturn1 = GeometryTools.distanceBetweenPoints(a1, b1);
      assertEquals("return value", expectedReturn1, actualReturn1, EPSILON);

      double[] a2 = {5.0, 2.0};
      double[] b2 = {5.0, 2.0};
      double expectedReturn2 = 0.0;
      double actualReturn2 = GeometryTools.distanceBetweenPoints(a2, b2);
      assertEquals("return value", expectedReturn2, actualReturn2, Double.MIN_VALUE);

      Point2d a3 = new Point2d(2.0, 2.0);
      Point2d b3 = new Point2d(6.0, -1.0);
      double expectedReturn3 = 5.0;
      double actualReturn3 = GeometryTools.distanceBetweenPoints(a3, b3);
      assertEquals("return value", expectedReturn3, actualReturn3, Double.MIN_VALUE);

      Point2d a4 = new Point2d(2.5, 5.1);
      Point2d b4 = new Point2d(9.3, 10.7);
      double expectedReturn4 = 8.80908621822;
      double actualReturn4 = GeometryTools.distanceBetweenPoints(a4, b4);
      assertEquals("return value", expectedReturn4, actualReturn4, EPSILON);

      Point2d a5 = new Point2d(5.0, 2.0);
      Point2d b5 = new Point2d(5.0, 2.0);
      double expectedReturn5 = 0.0;
      double actualReturn5 = GeometryTools.distanceBetweenPoints(a5, b5);
      assertEquals("return value", expectedReturn5, actualReturn5, Double.MIN_VALUE);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceFromPointToLine()
   {
      Point2d point = new Point2d(10, 2);
      Point2d lineStart = new Point2d(4, 2);
      Point2d lineEnd = new Point2d(10, 10);
      double expectedReturn = 4.8;
      double actualReturn = GeometryTools.distanceFromPointToLine(point, lineStart, lineEnd);
      assertEquals("return value", expectedReturn, actualReturn, Double.MIN_VALUE);

      Point2d point1 = new Point2d(10, 2);
      Point2d lineStart1 = new Point2d(10, 1);
      Point2d lineEnd1 = new Point2d(10, 10);
      double expectedReturn1 = 0.0;
      double actualReturn1 = GeometryTools.distanceFromPointToLine(point1, lineStart1, lineEnd1);
      assertEquals("return value", expectedReturn1, actualReturn1, Double.MIN_VALUE);

      Point2d point2 = new Point2d(1, 2);
      Point2d lineStart2 = new Point2d(4, 2);
      Point2d lineEnd2 = new Point2d(10, 10);
      double expectedReturn2 = 2.4;
      double actualReturn2 = GeometryTools.distanceFromPointToLine(point2, lineStart2, lineEnd2);
      assertEquals("return value", expectedReturn2, actualReturn2, EPSILON);

      Point2d point3 = new Point2d(10, 10);
      Point2d lineStart3 = new Point2d(4, 2);
      Point2d lineEnd3 = new Point2d(4, 2);
      double expectedReturn3 = 10;
      double actualReturn3 = GeometryTools.distanceFromPointToLine(point3, lineStart3, lineEnd3);
      assertEquals("return value", expectedReturn3, actualReturn3, Double.MIN_VALUE);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceFromPointToLine3D()
   {
      Point3d point = new Point3d(10, 2, 0);
      Point3d lineStart = new Point3d(4, 2, 0);
      Point3d lineEnd = new Point3d(10, 10, 0);
      double expectedReturn = 4.8;
      double actualReturn = GeometryTools.distanceFromPointToLine(point, lineStart, lineEnd);
      assertEquals("return value", expectedReturn, actualReturn, Double.MIN_VALUE);

      Point3d point2 = new Point3d(3, 3, 0);
      Point3d lineStart2 = new Point3d(0, 0, 0);
      Point3d lineEnd2 = new Point3d(3, 3, 3);
      double expectedReturn2 = 2.44948974278;
      double actualReturn2 = GeometryTools.distanceFromPointToLine(point2, lineStart2, lineEnd2);
      assertEquals("return value", expectedReturn2, actualReturn2, EPSILON);

      Point3d point1 = new Point3d(10, 10, 0);
      Point3d lineStart1 = new Point3d(4, 2, 0);
      Point3d lineEnd1 = new Point3d(4, 2, 0);
      double expectedReturn1 = 10.0;
      double actualReturn1 = GeometryTools.distanceFromPointToLine(point1, lineStart1, lineEnd1);
      assertEquals("return value", expectedReturn1, actualReturn1, Double.MIN_VALUE);

      for (int i = 0; i < 100; i++)
      {
         // Generate a random line
         Point3d start = RandomTools.generateRandomPoint3d(random, -10.0, 10.0);
         Point3d end = RandomTools.generateRandomPoint3d(random, -10.0, 10.0);
         Vector3d lineDirection = new Vector3d();
         lineDirection.sub(end, start);
         // Generate a random vector orthogonal to the line
         Vector3d orthogonalVector = RandomTools.generateRandomOrthogonalVector3d(random, lineDirection, true);
         double expectedDistance = RandomTools.generateRandomDouble(random, 0.0, 10.0);
         // Generate a random point located at an expected distance from the line
         Point3d randomPoint = new Point3d();
         // Randomize on the line
         randomPoint.interpolate(start, end, RandomTools.generateRandomDouble(random, 10.0));
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
   public void testDistanceFromPointToLineSegment()
   {
      Point2d point = new Point2d(10, 2);
      Point2d lineStart = new Point2d(4, 2);
      Point2d lineEnd = new Point2d(10, 10);
      double expectedReturn = 4.8;
      double actualReturn = GeometryTools.distanceFromPointToLineSegment(point, lineStart, lineEnd);
      assertEquals("return value", expectedReturn, actualReturn, Double.MIN_VALUE);

      Point2d point1 = new Point2d(10, 10);
      Point2d lineStart1 = new Point2d(4, 2);
      Point2d lineEnd1 = new Point2d(4, 2);
      double expectedReturn1 = 10.0;
      double actualReturn1 = GeometryTools.distanceFromPointToLineSegment(point1, lineStart1, lineEnd1);
      assertEquals("return value", expectedReturn1, actualReturn1, Double.MIN_VALUE);

      Point2d point2 = new Point2d(1, 1);
      Point2d lineStart2 = new Point2d(4, 2);
      Point2d lineEnd2 = new Point2d(5, 5);
      double expectedReturn2 = 3.16227766017;
      double actualReturn2 = GeometryTools.distanceFromPointToLineSegment(point2, lineStart2, lineEnd2);
      assertEquals("return value", expectedReturn2, actualReturn2, EPSILON);

      /** @todo fill in the test code */
   }

/*
   public void testGetClosestPointsForTwoLines()
   {
      Point3d p1 = new Point3d(5, 5, 0);
      FramePoint point1 = new FramePoint(ReferenceFrame.getWorldFrame(), p1.x, p1.y, p1.z);

      FrameVector vector1 = null;
      FramePoint point2 = null;
      FrameVector vector2 = null;
      FramePoint pointOnLine1 = null;
      FramePoint pointOnLine2 = null;
      geometryTools.getClosestPointsForTwoLines(point1, vector1, point2, vector2, pointOnLine1, pointOnLine2);
   }
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
      for (int i = 0; i < 100; i++)
      {
         Point3d pointOnPlane = RandomTools.generateRandomPoint3d(random, -10.0, 10.0);
         Vector3d planeNormal = RandomTools.generateRandomVector(random, RandomTools.generateRandomDouble(random, 0.0, 10.0));

         Vector3d parallelToPlane = RandomTools.generateRandomOrthogonalVector3d(random, planeNormal, true);
         Point3d secondPointOnPlane = new Point3d();
         secondPointOnPlane.scaleAdd(RandomTools.generateRandomDouble(random, 10.0), parallelToPlane, pointOnPlane);

         double expectedDistance = RandomTools.generateRandomDouble(random, 0.0, 10.0);
         Point3d point = new Point3d();
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
      Point3d endPoint0 = new Point3d();
      Point3d endPoint1 = new Point3d();

      for (int i = 0; i < 100; i++)
      {
         Point3d pointOnPlane = RandomTools.generateRandomPoint3d(random, -10.0, 10.0);
         Vector3d planeNormal = RandomTools.generateRandomVector(random, RandomTools.generateRandomDouble(random, 0.0, 10.0));

         Vector3d parallelToPlane = RandomTools.generateRandomOrthogonalVector3d(random, planeNormal, true);
         Point3d randomLinePlaneIntersection = new Point3d();
         randomLinePlaneIntersection.scaleAdd(RandomTools.generateRandomDouble(random, 10.0), parallelToPlane, pointOnPlane);

         Vector3d lineDirection = RandomTools.generateRandomVector(random, 1.0);

         // Create the two endPoints on each side of the plane:
         endPoint0.scaleAdd(RandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection, randomLinePlaneIntersection);
         endPoint1.scaleAdd(RandomTools.generateRandomDouble(random, -10.0, 0.0), lineDirection, randomLinePlaneIntersection);
         assertTrue(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, endPoint0, endPoint1));
         assertTrue(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, endPoint1, endPoint0));

         // Create the two endPoints on one side of the plane:
         endPoint0.scaleAdd(RandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection, randomLinePlaneIntersection);
         endPoint1.scaleAdd(RandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection, randomLinePlaneIntersection);
         assertFalse(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, endPoint0, endPoint1));
         assertFalse(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, endPoint1, endPoint0));
         
         // Create the two endPoints on the other side of the plane:
         endPoint0.scaleAdd(RandomTools.generateRandomDouble(random, -10.0, 0.0), lineDirection, randomLinePlaneIntersection);
         endPoint1.scaleAdd(RandomTools.generateRandomDouble(random, -10.0, 0.0), lineDirection, randomLinePlaneIntersection);
         assertFalse(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, endPoint0, endPoint1));
         assertFalse(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, endPoint1, endPoint0));

         // Annoying case 1: endPoint0 == endPoint1 => should return false whether the endPoints are on plane or not.
         endPoint0.scaleAdd(RandomTools.generateRandomDouble(random, -10.0, 0.0), lineDirection, randomLinePlaneIntersection);
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
      Point3d pointOnPlane = new Point3d();
      Vector3d planeNormal = new Vector3d(0.0, 0.0, 1.0);

      Point3d randomLinePlaneIntersection = RandomTools.generateRandomPoint3d(random, -10.0, 10.0);
      randomLinePlaneIntersection.setZ(0.0);

      Vector3d lineDirection = RandomTools.generateRandomVector(random, 1.0);
      // Ensure that the line direction and the plane normal are somewhat pointing the same direction.
      if (lineDirection.dot(planeNormal) < 0.0)
         lineDirection.negate();

      endPoint0.set(randomLinePlaneIntersection);
      endPoint1.scaleAdd(RandomTools.generateRandomDouble(random, -10.0, 0.0), lineDirection, randomLinePlaneIntersection);
      assertFalse(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, endPoint0, endPoint1));
      assertFalse(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, endPoint1, endPoint0));
      endPoint0.set(randomLinePlaneIntersection);
      endPoint1.scaleAdd(RandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection, randomLinePlaneIntersection);
      assertFalse(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, endPoint0, endPoint1));
      assertFalse(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, endPoint1, endPoint0));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDoLineSegmentsIntersect1()
   {
      boolean intersect = GeometryTools.doLineSegmentsIntersect(new Point2d(-1.0, 0.0), new Point2d(1.0, 0.0), new Point2d(0.0, -1.0), new Point2d(0.0, 1.0));
      assertTrue(intersect);

      intersect = GeometryTools.doLineSegmentsIntersect(new Point2d(-1.0, 0.0), new Point2d(1.0, 0.0), new Point2d(0.0, 1.0), new Point2d(0.0, -1.0));
      assertTrue(intersect);

      intersect = GeometryTools.doLineSegmentsIntersect(new Point2d(-1.0, 0.0), new Point2d(1.0, 0.0), new Point2d(-1.0, 1.0), new Point2d(1.0, -1.0));
      assertTrue(intersect);

      intersect = GeometryTools.doLineSegmentsIntersect(new Point2d(-1.0, 0.0), new Point2d(1.0, 0.0), new Point2d(-1.0, -1.0), new Point2d(1.0, 1.0));
      assertTrue(intersect);

      intersect = GeometryTools.doLineSegmentsIntersect(new Point2d(-1.0, 0.0), new Point2d(1.0, 0.0), new Point2d(-1.0, -1.0), new Point2d(1.0, -1.0));
      assertFalse(intersect);

      intersect = GeometryTools.doLineSegmentsIntersect(new Point2d(-1.0, 0.0), new Point2d(1.0, 0.0), new Point2d(-1.0, 1.0), new Point2d(1.0, 1.0));
      assertFalse(intersect);

      intersect = GeometryTools.doLineSegmentsIntersect(new Point2d(-1.0, 0.0), new Point2d(1.0, 0.0), new Point2d(-1.0, 0.0), new Point2d(1.0, 0.0));
      assertTrue(intersect);

      intersect = GeometryTools.doLineSegmentsIntersect(new Point2d(-1.0, 0.0), new Point2d(1.0, 0.0), new Point2d(-1.1, 1.0), new Point2d(-1.1, -1.0));
      assertFalse(intersect);

      intersect = GeometryTools.doLineSegmentsIntersect(new Point2d(-1.0, 0.0), new Point2d(1.0, 0.0), new Point2d(-1.0, 1.0), new Point2d(-1.0, -1.0));
      assertTrue(intersect);

      intersect = GeometryTools.doLineSegmentsIntersect(new Point2d(-1.0, 0.0), new Point2d(1.0, 0.0), new Point2d(-1.0, 0.0), new Point2d(1.0, 0.0));
      assertTrue(intersect);

      intersect = GeometryTools.doLineSegmentsIntersect(new Point2d(-1.0, 0.0), new Point2d(1.0, 0.0), new Point2d(-1.0, 0.0), new Point2d(1.0, 0.0));
      assertTrue(intersect);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDoLineSegmentsIntersect2()
   {
      for (int i = 0; i < 100; i++)
      {
         Point2d lineSegmentStart1 = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         Point2d lineSegmentEnd1 = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);

         Point2d pointOnLineSegment1 = new Point2d();
         pointOnLineSegment1.interpolate(lineSegmentStart1, lineSegmentEnd1, RandomTools.generateRandomDouble(random, 0.0, 1.0));

         Vector2d lineDirection2 = RandomTools.generateRandomVector2d(random, 1.0);

         Point2d lineSegmentStart2 = new Point2d();
         Point2d lineSegmentEnd2 = new Point2d();

         // Expecting intersection
         lineSegmentStart2.scaleAdd(RandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection2, pointOnLineSegment1);
         lineSegmentEnd2.scaleAdd(RandomTools.generateRandomDouble(random, -10.0, 0.0), lineDirection2, pointOnLineSegment1);
         assertTrue(GeometryTools.doLineSegmentsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2));
         assertTrue(GeometryTools.doLineSegmentsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2));
         assertTrue(GeometryTools.doLineSegmentsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2));
         assertTrue(GeometryTools.doLineSegmentsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2));

         // Not expecting intersection
         lineSegmentStart2.scaleAdd(RandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection2, pointOnLineSegment1);
         lineSegmentEnd2.scaleAdd(RandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection2, pointOnLineSegment1);
         assertFalse(GeometryTools.doLineSegmentsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2));
         assertFalse(GeometryTools.doLineSegmentsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2));
         assertFalse(GeometryTools.doLineSegmentsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2));
         assertFalse(GeometryTools.doLineSegmentsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2));
      }

      // Test intersection at one of the end points
      for (int i = 0; i < 100; i++)
      {
         Point2d lineSegmentStart1 = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         Point2d lineSegmentEnd1 = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);

         Point2d pointOnLineSegment1 = new Point2d(lineSegmentStart1);

         Vector2d lineDirection2 = RandomTools.generateRandomVector2d(random, 1.0);

         Point2d lineSegmentStart2 = new Point2d();
         Point2d lineSegmentEnd2 = new Point2d();

         // Expecting intersection
         lineSegmentStart2.scaleAdd(RandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection2, pointOnLineSegment1);
         lineSegmentEnd2.scaleAdd(RandomTools.generateRandomDouble(random, -10.0, 0.0), lineDirection2, pointOnLineSegment1);
         assertTrue(GeometryTools.doLineSegmentsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2));
         assertTrue(GeometryTools.doLineSegmentsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2));
         assertTrue(GeometryTools.doLineSegmentsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2));
         assertTrue(GeometryTools.doLineSegmentsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2));
      }

      // Test with parallel/collinear line segments
      for (int i = 0; i < 100; i++)
      {
         Point2d lineSegmentStart1 = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         Point2d lineSegmentEnd1 = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);

         Point2d lineSegmentStart2 = new Point2d();
         Point2d lineSegmentEnd2 = new Point2d();

         double alpha1 = RandomTools.generateRandomDouble(random, 2.0);
         double alpha2 = RandomTools.generateRandomDouble(random, 2.0);

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
         Vector2d orthogonal = new Vector2d();
         orthogonal.sub(lineSegmentEnd1, lineSegmentStart1);
         orthogonal.set(-orthogonal.getY(), orthogonal.getX());
         orthogonal.normalize();

         double distance = RandomTools.generateRandomDouble(random, 1.0e-10, 10.0);
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
   public void testGetLineSegmentPercentagesIfIntersecting()
   {
      double[] percentages;
      JUnitTools.assertExceptionThrown(UndefinedOperationException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            double[] percentages = GeometryTools.getLineSegmentPercentagesIfIntersecting(new Point2d(-1.0, 0.0), new Point2d(1.0, 0.0), new Point2d(-1.0, 1.0),
                  new Point2d(1.0, 1.0));
         }
      });

      percentages = GeometryTools.getLineSegmentPercentagesIfIntersecting(new Point2d(-1.0, 0.0), new Point2d(1.0, 0.0), new Point2d(0.0, 1.0),
              new Point2d(0.0, -1.0));
      assertEquals(0.5, percentages[0], 1e-7);
      assertEquals(0.5, percentages[1], 1e-7);

      percentages = GeometryTools.getLineSegmentPercentagesIfIntersecting(new Point2d(-1.0, 0.0), new Point2d(1.0, 0.0), new Point2d(0.0, -1.0),
              new Point2d(0.0, 1.0));
      System.out.println(percentages[0] + "   " + percentages[1]);
      assertEquals(0.5, percentages[0], 1e-7);
      assertEquals(0.5, percentages[1], 1e-7);

      JUnitTools.assertExceptionThrown(UndefinedOperationException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            double[] percentages = GeometryTools.getLineSegmentPercentagesIfIntersecting(new Point2d(-1.0, 0.0), new Point2d(1.0, 0.0), new Point2d(-1.0, 0.0), new Point2d(1.0, 0.0));
         }
      });
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
      Point2d point1 = new Point2d(5, 1.0);
      Vector2d vector1 = new Vector2d(8, 9);
      Point2d point2 = new Point2d(5, 1.0);
      Vector2d vector2 = new Vector2d(3, 9);
      Point2d expectedReturn = new Point2d(5.0, 1.0);
      Point2d actualReturn = GeometryTools.getIntersectionBetweenTwoLines(point1, vector1, point2, vector2);
      assertEquals("return value", expectedReturn, actualReturn);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetIntersectionBetweenTwoLines2()
   {
      double epsilon = Epsilons.ONE_TRILLIONTH;

      for (int i = 0; i < 100; i++)
      {
         Point2d pointOnLine1 = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         Vector2d lineDirection1 = RandomTools.generateRandomVector2d(random, 10.0);

         Point2d expectedIntersection = new Point2d();
         expectedIntersection.scaleAdd(RandomTools.generateRandomDouble(random, 10.0), lineDirection1, pointOnLine1);

         Vector2d lineDirection2 = RandomTools.generateRandomVector2d(random, 10.0);
         Point2d pointOnLine2 = new Point2d(expectedIntersection);

         Point2d actualIntersection = GeometryTools.getIntersectionBetweenTwoLines(pointOnLine1, lineDirection1, pointOnLine2, lineDirection2);
         JUnitTools.assertTuple2dEquals(expectedIntersection, actualIntersection, epsilon);

         pointOnLine2.scaleAdd(RandomTools.generateRandomDouble(random, 10.0), lineDirection2, pointOnLine2);
         actualIntersection = GeometryTools.getIntersectionBetweenTwoLines(pointOnLine1, lineDirection1, pointOnLine2, lineDirection2);
         JUnitTools.assertTuple2dEquals(expectedIntersection, actualIntersection, epsilon);
      }

      // Test when parallel but not collinear
      for (int i = 0; i < 100; i++)
      {
         Point2d pointOnLine1 = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         Vector2d lineDirection1 = RandomTools.generateRandomVector2d(random, 10.0);

         Vector2d lineDirection2 = new Vector2d(lineDirection1);
         if (random.nextBoolean())
            lineDirection2.negate();
         Point2d pointOnLine2 = new Point2d(pointOnLine1);

         Vector2d orthogonal = new Vector2d(- lineDirection1.getY(), lineDirection1.getX());

         pointOnLine2.scaleAdd(RandomTools.generateRandomDouble(random, 10.0), orthogonal, pointOnLine2);
         pointOnLine2.scaleAdd(RandomTools.generateRandomDouble(random, 10.0), lineDirection2, pointOnLine2);
         Point2d actualIntersection = GeometryTools.getIntersectionBetweenTwoLines(pointOnLine1, lineDirection1, pointOnLine2, lineDirection2);
         assertNull(actualIntersection);
      }

      // Test when collinear
      for (int i = 0; i < 100; i++)
      {
         Point2d pointOnLine1 = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         Vector2d lineDirection1 = RandomTools.generateRandomVector2d(random, 10.0);

         Point2d expectedIntersection = new Point2d();
         expectedIntersection.set(pointOnLine1);

         Vector2d lineDirection2 = new Vector2d(lineDirection1);
         Point2d pointOnLine2 = new Point2d(expectedIntersection);

         Point2d actualIntersection = GeometryTools.getIntersectionBetweenTwoLines(pointOnLine1, lineDirection1, pointOnLine2, lineDirection2);
         JUnitTools.assertTuple2dEquals(expectedIntersection, actualIntersection, epsilon);

         pointOnLine2.scaleAdd(RandomTools.generateRandomDouble(random, 10.0), lineDirection2, pointOnLine2);
         actualIntersection = GeometryTools.getIntersectionBetweenTwoLines(pointOnLine1, lineDirection1, pointOnLine2, lineDirection2);
         JUnitTools.assertTuple2dEquals(expectedIntersection, actualIntersection, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetIntersectionBetweenTwoLineSegments()
   {
      Random random = new Random(3242L);

      for (int i = 0; i < 100; i++)
      {
         Point2d lineSegmentStart1 = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         Point2d lineSegmentEnd1 = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);

         Point2d expectedIntersection = new Point2d();
         expectedIntersection.interpolate(lineSegmentStart1, lineSegmentEnd1, RandomTools.generateRandomDouble(random, 0.0, 1.0));

         Vector2d lineDirection2 = RandomTools.generateRandomVector2d(random, 1.0);

         Point2d lineSegmentStart2 = new Point2d();
         Point2d lineSegmentEnd2 = new Point2d();

         // Expecting intersection
         lineSegmentStart2.scaleAdd(RandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection2, expectedIntersection);
         lineSegmentEnd2.scaleAdd(RandomTools.generateRandomDouble(random, -10.0, 0.0), lineDirection2, expectedIntersection);
         assertAllCombinationsOfTwoLineSegmentsIntersection(expectedIntersection, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);

         // Not expecting intersection
         lineSegmentStart2.scaleAdd(RandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection2, expectedIntersection);
         lineSegmentEnd2.scaleAdd(RandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection2, expectedIntersection);
         assertOnlyExistenceOfTwoLineSegmentsIntersectionAllCombinations(false, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      }

      // Test intersection at one of the end points
      for (int i = 0; i < 100; i++)
      {
         Point2d lineSegmentStart1 = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         Point2d lineSegmentEnd1 = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);

         Point2d expectedIntersection = new Point2d(lineSegmentStart1);

         Vector2d lineDirection2 = RandomTools.generateRandomVector2d(random, 1.0);

         Point2d lineSegmentStart2 = new Point2d();
         Point2d lineSegmentEnd2 = new Point2d();

         // Not expecting intersection
         lineSegmentStart2.scaleAdd(RandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection2, expectedIntersection);
         lineSegmentEnd2.scaleAdd(RandomTools.generateRandomDouble(random, -10.0, 0.0), lineDirection2, expectedIntersection);
         assertAllCombinationsOfTwoLineSegmentsIntersection(expectedIntersection, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      }

      // Test with parallel/collinear line segments
      for (int i = 0; i < 100; i++)
      {
         Point2d lineSegmentStart1 = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         Point2d lineSegmentEnd1 = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);

         Point2d lineSegmentStart2 = new Point2d();
         Point2d lineSegmentEnd2 = new Point2d();

         double alpha1 = RandomTools.generateRandomDouble(random, 2.0);
         double alpha2 = RandomTools.generateRandomDouble(random, 2.0);

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
         Vector2d orthogonal = new Vector2d();
         orthogonal.sub(lineSegmentEnd1, lineSegmentStart1);
         orthogonal.set(-orthogonal.getY(), orthogonal.getX());
         orthogonal.normalize();

         double distance = RandomTools.generateRandomDouble(random, 1.0e-10, 10.0);
         lineSegmentStart2.scaleAdd(distance, orthogonal, lineSegmentStart2);
         lineSegmentEnd2.scaleAdd(distance, orthogonal, lineSegmentEnd2);
         assertOnlyExistenceOfTwoLineSegmentsIntersectionAllCombinations(false, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      }
   }

   private void assertOnlyExistenceOfTwoLineSegmentsIntersectionAllCombinations(boolean intersectionExist, Point2d lineSegmentStart1, Point2d lineSegmentEnd1, Point2d lineSegmentStart2, Point2d lineSegmentEnd2)
   {
      boolean success;
      Point2d actualIntersection = new Point2d();

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

   private void assertAllCombinationsOfTwoLineSegmentsIntersection(Point2d expectedIntersection, Point2d lineSegmentStart1, Point2d lineSegmentEnd1, Point2d lineSegmentStart2, Point2d lineSegmentEnd2)
   {
      double epsilon = Epsilons.ONE_TRILLIONTH;
      boolean success;
      Point2d actualIntersection = new Point2d();

      success = GeometryTools.getIntersectionBetweenTwoLineSegments(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2, actualIntersection);
      assertTrue(success);
      JUnitTools.assertTuple2dEquals(expectedIntersection, actualIntersection, epsilon);
      success = GeometryTools.getIntersectionBetweenTwoLineSegments(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2, actualIntersection);
      assertTrue(success);
      JUnitTools.assertTuple2dEquals(expectedIntersection, actualIntersection, epsilon);
      success = GeometryTools.getIntersectionBetweenTwoLineSegments(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2, actualIntersection);
      assertTrue(success);
      JUnitTools.assertTuple2dEquals(expectedIntersection, actualIntersection, epsilon);
      success = GeometryTools.getIntersectionBetweenTwoLineSegments(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2, actualIntersection);
      assertTrue(success);
      JUnitTools.assertTuple2dEquals(expectedIntersection, actualIntersection, epsilon);

      success = GeometryTools.getIntersectionBetweenTwoLineSegments(lineSegmentStart2, lineSegmentEnd2, lineSegmentStart1, lineSegmentEnd1, actualIntersection);
      assertTrue(success);
      JUnitTools.assertTuple2dEquals(expectedIntersection, actualIntersection, epsilon);
      success = GeometryTools.getIntersectionBetweenTwoLineSegments(lineSegmentStart2, lineSegmentEnd2, lineSegmentEnd1, lineSegmentStart1, actualIntersection);
      assertTrue(success);
      JUnitTools.assertTuple2dEquals(expectedIntersection, actualIntersection, epsilon);
      success = GeometryTools.getIntersectionBetweenTwoLineSegments(lineSegmentEnd2, lineSegmentStart2, lineSegmentStart1, lineSegmentEnd1, actualIntersection);
      assertTrue(success);
      JUnitTools.assertTuple2dEquals(expectedIntersection, actualIntersection, epsilon);
      success = GeometryTools.getIntersectionBetweenTwoLineSegments(lineSegmentEnd2, lineSegmentStart2, lineSegmentEnd1, lineSegmentStart1, actualIntersection);
      assertTrue(success);
      JUnitTools.assertTuple2dEquals(expectedIntersection, actualIntersection, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetNormalToPlane()
   {
      Point3d a = new Point3d(0, 0, 0);
      Point3d b = new Point3d(1, 0, 0);
      Point3d c = new Point3d(0, 1, 0);
      Vector3d expectedReturn = new Vector3d(0, 0, 1);
      Vector3d actualReturn = GeometryTools.getNormalToPlane(a, b, c);
      assertEquals("return value", expectedReturn, actualReturn);

      Point3d a1 = new Point3d(5, 0, 6);
      Point3d b1 = new Point3d(1, 0, 4);
      Point3d c1 = new Point3d(2, 0, 65);
      Vector3d expectedReturn1 = new Vector3d(0, 1, 0);
      Vector3d actualReturn1 = GeometryTools.getNormalToPlane(a1, b1, c1);
      assertEquals("return value", expectedReturn1, actualReturn1);

      Point3d a2 = new Point3d(0, 6, 4);
      Point3d b2 = new Point3d(0, 32, 6);
      Point3d c2 = new Point3d(0, 1, 4);
      Vector3d expectedReturn2 = new Vector3d(1, 0, 0);
      Vector3d actualReturn2 = GeometryTools.getNormalToPlane(a2, b2, c2);
      assertEquals("return value", expectedReturn2, actualReturn2);

      /*
       *    Point3d a3 = new Point3d(7, 5, 9);
       *    Point3d b3 = new Point3d(0, 1, 0);
       *    Point3d c3 = new Point3d(0, 1, 0);
       *    Vector3d expectedReturn3 = new Vector3d(0, 0, 1);
       *    Vector3d actualReturn3 = geometryTools.getNormalToPlane(a3, b3, c3);
       *    assertEquals("return value", expectedReturn3, actualReturn3);
       */
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetPerpendicularBisector()
   {
      Point2d lineStart = new Point2d(1, 1);
      Point2d lineEnd = new Point2d(5, 5);
      Point2d bisectorStart = new Point2d(2, 1);
      Vector2d bisectorDirection = new Vector2d();
      GeometryTools.getPerpendicularBisector(lineStart, lineEnd, bisectorStart, bisectorDirection);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetPerpendicularVector()
   {
      Vector2d vector = new Vector2d(15.0, 10.0);
      Vector2d expectedReturn = new Vector2d(-10.0, 15.0);
      Vector2d actualReturn = GeometryTools.getPerpendicularVector(vector);
      assertEquals("return value", expectedReturn, actualReturn);

      /** @todo fill in the test code */
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetPerpendicularVectorFromLineToPoint()
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

      Point2d returnPoint = GeometryTools.getClosestPointToLineSegment(new Point2d(-2.5, 1.5), new Point2d(0, 0), new Point2d(-4, 4));
      FrameVector x1 = new FrameVector(point1.getReferenceFrame());
      x1.sub(point1, intersectionPoint1);
      FrameVector expectedReturn1 = x1;
      FrameVector actualReturn1 = GeometryTools.getPerpendicularVectorFromLineToPoint(point1, lineStart1, lineEnd1, intersectionPoint1);

      assertTrue("Test Failed", expectedReturn1.epsilonEquals(actualReturn1, EPSILON));


      /*
       *  FramePoint point1 = new FramePoint(ReferenceFrame.getWorldFrame(), 4, 2, 0);
       *  FramePoint lineStart1 = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 0);
       *  FramePoint lineEnd1 = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 0);
       *  FramePoint intersectionPoint1 = new FramePoint(ReferenceFrame.getWorldFrame(), 3, 3, 0);
       *  FrameVector x1 = new FrameVector(point1.getReferenceFrame());
       *  x1.sub(point1, intersectionPoint1);
       *  FrameVector expectedReturn1 = x1;
       *  FrameVector actualReturn1 = geometryTools.getPerpendicularVectorFromLineToPoint(point1, lineStart1, lineEnd1, intersectionPoint1);
       *  assertTrue("Test Failed", expectedReturn1.epsilonEquals(actualReturn1, EPSILON));
       *
       * /returns zeros if point is on line
       *  FramePoint point2 = new FramePoint(ReferenceFrame.getWorldFrame(), 5, 0, 0);
       *  FramePoint lineStart2 = new FramePoint(ReferenceFrame.getWorldFrame(), 0, 0, 0);
       *  FramePoint lineEnd2 = new FramePoint(ReferenceFrame.getWorldFrame(), 10, 0, 0);
       *  FramePoint intersectionPoint2 = new FramePoint(ReferenceFrame.getWorldFrame(), 5, 0, 0);
       *  FrameVector x2 = new FrameVector(point2.getReferenceFrame());
       *  x2.sub(point2, intersectionPoint2);
       *  FrameVector expectedReturn2 = x2;
       *  FrameVector actualReturn2 = geometryTools.getPerpendicularVectorFromLineToPoint(point2, lineStart2, lineEnd2, intersectionPoint2);
       *  assertEquals("return value", expectedReturn2, actualReturn2);
       */
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
      Point3d point1 = new Point3d(0, 0, 0);
      Point3d point2 = new Point3d(7, 0, 0);
      Point3d point3 = new Point3d(2, 0, 0);
      Vector3d expectedReturn = null;
      Vector3d actualReturn = GeometryTools.getPlaneNormalGivenThreePoints(point1, point2, point3);
      assertEquals("return value", expectedReturn, actualReturn);

      Point3d point01 = new Point3d(15, 0, 0);
      Point3d point02 = new Point3d(15, 0, 0);
      Point3d point03 = new Point3d(15, 0, 0);
      Vector3d expectedReturn1 = null;
      Vector3d actualReturn1 = GeometryTools.getPlaneNormalGivenThreePoints(point01, point02, point03);
      assertEquals("return value", expectedReturn1, actualReturn1);

      Point3d point11 = new Point3d(0, 4, 0);
      Point3d point12 = new Point3d(0, 2, 0);
      Point3d point13 = new Point3d(0, 67, 0);
      Vector3d expectedReturn2 = null;
      Vector3d actualReturn2 = GeometryTools.getPlaneNormalGivenThreePoints(point11, point12, point13);
      assertEquals("return value", expectedReturn2, actualReturn2);

      Point3d point21 = new Point3d(0, 0, 4);
      Point3d point22 = new Point3d(0, 0, 7);
      Point3d point23 = new Point3d(0, 0, 5);
      Vector3d expectedReturn3 = null;
      Vector3d actualReturn3 = GeometryTools.getPlaneNormalGivenThreePoints(point21, point22, point23);
      assertEquals("return value", expectedReturn3, actualReturn3);

      Point3d point31 = new Point3d(0, 67, 5);
      Point3d point32 = new Point3d(0, 3, 7);
      Point3d point33 = new Point3d(0, 90, 7.24264068712);
      Vector3d expectedReturn4 = new Vector3d(-1, 0, 0);
      Vector3d actualReturn4 = GeometryTools.getPlaneNormalGivenThreePoints(point31, point32, point33);
      assertEquals("return value", expectedReturn4, actualReturn4);

      Point3d point41 = new Point3d(45, 0, 5);
      Point3d point42 = new Point3d(35, 0, 7);
      Point3d point43 = new Point3d(132, 0, 7.24264068712);
      Vector3d expectedReturn5 = new Vector3d(0, 1, 0);
      Vector3d actualReturn5 = GeometryTools.getPlaneNormalGivenThreePoints(point41, point42, point43);
      assertTrue("Test Failed", expectedReturn5.epsilonEquals(actualReturn5, EPSILON));

      Point3d point51 = new Point3d(45, 67, 0);
      Point3d point52 = new Point3d(35, 56, 0);
      Point3d point53 = new Point3d(132, -4, 0);
      Vector3d expectedReturn6 = new Vector3d(0, 0, 1);
      Vector3d actualReturn6 = GeometryTools.getPlaneNormalGivenThreePoints(point51, point52, point53);
      assertTrue("Test Failed", expectedReturn6.epsilonEquals(actualReturn6, EPSILON));

      Point3d point61 = new Point3d(1, 5, 7);
      Point3d point62 = new Point3d(1, 5, 7);
      Point3d point63 = new Point3d(5, 12, 4325);
      Vector3d expectedReturn7 = null;
      Vector3d actualReturn7 = GeometryTools.getPlaneNormalGivenThreePoints(point61, point62, point63);
      assertEquals("return value", expectedReturn7, actualReturn7);


   }

/*
   public void testGetTransform()
   {
      FramePoint point = null;
      FrameVector normal = null;
      Orientation expectedReturn = null;
      Orientation actualReturn = geometryTools.getTransform(point, normal);
      assertEquals("return value", expectedReturn, actualReturn);
   }

   public void testGetVerticalSpansOfPoints()
   {
      double xMin = 0.0;
      double yMin = 0.0;
      double zMin = 0.0;
      double xMax = 0.0;
      double yMax = 0.0;
      double zMax = 0.0;
      double xResolution = 0.0;
      double yResolution = 0.0;
      double zResolution = 0.0;
      ArrayList expectedReturn = null;
      ArrayList actualReturn = geometryTools.getVerticalSpansOfPoints(xMin, yMin, zMin, xMax, yMax, zMax, xResolution, yResolution, zResolution);
      assertEquals("return value", expectedReturn, actualReturn);
   }
*/

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetTopVertexOfIsoscelesTriangle()
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
   public void testGetPerpendicularToLine()
   {
      ReferenceFrame frame = ReferenceFrame.getWorldFrame();

      FramePoint lineStart = new FramePoint(frame);
      FramePoint lineEnd = new FramePoint(frame);

      FrameVector lineStartToEnd = new FrameVector(frame);
      FrameVector planeNormal = new FrameVector(frame);
      FramePoint bisectorEnd = new FramePoint(frame);

      FrameVector bisectorComputed = new FrameVector(frame);

      double bisectorLengthDesired = 1.5;

      planeNormal.setIncludingFrame(frame, 0.0, 0.0, 1.0);
      lineStart.setIncludingFrame(frame, 0.0, -1.0, 0.0);
      lineEnd.setIncludingFrame(frame, 0.0, 1.0, 0.0);
      lineStartToEnd.sub(lineEnd, lineStart);
      bisectorEnd.setIncludingFrame(frame, -bisectorLengthDesired, 0.0, 0.0);

      GeometryTools.getPerpendicularToLine(lineStartToEnd, planeNormal, bisectorLengthDesired, bisectorComputed);
      FramePoint bisectorEndComputed = FramePoint.getMidPoint(lineStart, lineEnd);
      bisectorEndComputed.add(bisectorComputed);

      String errorMsg = "Computed bisector endpoint: " + bisectorEndComputed + "\n does not match actual bisector endpoint: " + bisectorEnd;
      assertEquals(errorMsg, 0.0, bisectorEnd.distance(bisectorEndComputed), 1e-9);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetZPlanePerpendicularBisector()
   {
      FramePoint lineStart = new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 11.5);
      FramePoint lineEnd = new FramePoint(ReferenceFrame.getWorldFrame(), -3.0, 3.0, -89.6);

      FramePoint mid = new FramePoint(lineEnd.getReferenceFrame());
      FrameVector direction = new FrameVector(lineEnd.getReferenceFrame());

      GeometryTools.getZPlanePerpendicularBisector(lineStart, lineEnd, mid, direction);

      // assertEquals("return value", expectedReturn, actualReturn);

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
      Point2d point = new Point2d(3, 9);
      Point2d lineStart = new Point2d(-5, 8);
      Point2d lineEnd = new Point2d(10, 7);
      boolean expectedReturn = true;
      boolean actualReturn = GeometryTools.isPointOnLeftSideOfLine(point, lineStart, lineEnd);
      assertEquals("return value", expectedReturn, actualReturn);

      Point2d point2 = new Point2d(1, 5);
      Point2d lineStart2 = new Point2d(-5, 8);
      Point2d lineEnd2 = new Point2d(10, 7);
      boolean expectedReturn2 = false;
      boolean actualReturn2 = GeometryTools.isPointOnLeftSideOfLine(point2, lineStart2, lineEnd2);
      assertEquals("return value", expectedReturn2, actualReturn2);

      Point2d point3 = new Point2d(1, 1);
      Point2d lineStart3 = new Point2d(0, 0);
      Point2d lineEnd3 = new Point2d(10, 10);
      boolean expectedReturn3 = false;
      boolean actualReturn3 = GeometryTools.isPointOnLeftSideOfLine(point3, lineStart3, lineEnd3);
      assertEquals("return value", expectedReturn3, actualReturn3);

      Point2d point4 = new Point2d(3, 9);
      Point2d lineStart4 = new Point2d(10, 7);
      Point2d lineEnd4 = new Point2d(-5, 8);
      boolean expectedReturn4 = false;
      boolean actualReturn4 = GeometryTools.isPointOnLeftSideOfLine(point4, lineStart4, lineEnd4);
      assertEquals("return value", expectedReturn4, actualReturn4);


   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(
      timeout = 30000,
      expected = RuntimeException.class
   )
   public void testAngleByLawOfCosineWithNegativeSideLengthA()
   {
      double a = 1.0;
      GeometryTools.getUnknownTriangleAngleByLawOfCosine(-a, a, a);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(
      timeout = 30000,
      expected = RuntimeException.class
   )
   public void testAngleByLawOfCosineWithNegativeSideLengthB()
   {
      double a = 1.0;
      GeometryTools.getUnknownTriangleAngleByLawOfCosine(a, -a, a);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(
      timeout = 30000,
      expected = RuntimeException.class
   )
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
   @Test(
      timeout = 30000,
      expected = RuntimeException.class
   )
   public void testSideLengthByLawOfCosineNegativeSideLengthA()
   {
      double a = 1.0;
      double gamma = Math.PI / 3.0;
      GeometryTools.getUnknownTriangleSideLengthByLawOfCosine(-a, a, gamma);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(
      timeout = 30000,
      expected = RuntimeException.class
   )
   public void testSideLengthByLawOfCosineNegativeSideLengthB()
   {
      double a = 1.0;
      double gamma = Math.PI / 3.0;
      GeometryTools.getUnknownTriangleSideLengthByLawOfCosine(a, -a, gamma);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(
      timeout = 30000,
      expected = RuntimeException.class
   )
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
   @Test(
      timeout = 30000,
      expected = RuntimeException.class
   )
   public void illegalPythagorasGetCathetus()
   {
      GeometryTools.pythagorasGetCathetus(1.0, 2.0);
   }

   private static final boolean VERBOSE = false;

   private Random random = new Random(1176L);

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceLineSegment2dLineSegment2d()
   {
      // not yet implemented
//    for (int i = 0; i < 1000; i++)
//    {
//       Point2d seg1p1 = new Point2d(random.nextDouble() * 1000 - 500, random.nextDouble() * 1000 - 500);
//       Point2d seg1p2 = new Point2d(random.nextDouble() * 1000 - 500, random.nextDouble() * 1000 - 500);
//       Point2d seg2p1 = new Point2d(random.nextDouble() * 1000 - 500, random.nextDouble() * 1000 - 500);
//       Point2d seg2p2 = new Point2d(random.nextDouble() * 1000 - 500, random.nextDouble() * 1000 - 500);
//
//       LineSegment2d seg1 = new LineSegment2d(seg1p1, seg1p2);
//       LineSegment2d seg2 = new LineSegment2d(seg2p1, seg2p2);
//
//       double returnedDistance = Geometry2dCalculator.distance(seg1, seg2);
//       if (seg1.intersectionWith(seg2) != null)
//          assertEquals("segmentes: " + seg1 + " " + seg2 + " intersect but distance is greater than 0.0: " + returnedDistance, 0.0, returnedDistance,
//                       0.000001);
//       else
//       {
//          double shortestDistance = Double.MAX_VALUE;
//          double distance = seg1.distance(seg2p1);
//          if (distance < shortestDistance)
//             shortestDistance = distance;
//
//
//
//       }
//    }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testClipToBoundingBox()
   {
      Tuple3d tuple3d = new Point3d(1.0, -1.0, 0.0);
      GeometryTools.clipToBoundingBox(tuple3d, -0.5, 0.5, 0.5, -0.5, 0.0, 0.0);
      JUnitTools.assertTuple3dEquals("not equal", new Point3d(0.5, -0.5, 0.0), tuple3d, 0.0);
      tuple3d.set(1.0, -1.0, 0.0);
      GeometryTools.clipToBoundingBox(tuple3d, 0.5, -0.5, -0.5, 0.5, -0.1, 0.1);
      JUnitTools.assertTuple3dEquals("not equal", new Point3d(0.5, -0.5, 0.0), tuple3d, 0.0);
      tuple3d.set(1.0, -1.0, 2.0);
      GeometryTools.clipToBoundingBox(tuple3d, 0.5, -0.5, -0.5, 0.5, -0.1, 1.0);
      JUnitTools.assertTuple3dEquals("not equal", new Point3d(0.5, -0.5, 1.0), tuple3d, 0.0);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCombine()
   {
      ArrayList<Point2d> firstList = new ArrayList<Point2d>();
      for (int i = 0; i < 100; i++)
      {
         firstList.add(new Point2d(random.nextDouble(), random.nextDouble()));
      }

      ConvexPolygon2d firstPolygon = new ConvexPolygon2d(firstList);

      ArrayList<Point2d> secondList = new ArrayList<Point2d>();
      for (int i = 0; i < 200; i++)
      {
         secondList.add(new Point2d(random.nextDouble(), random.nextDouble()));
      }

      ConvexPolygon2d secondPolygon = new ConvexPolygon2d(secondList);

      ConvexPolygon2d result = new ConvexPolygon2d(firstPolygon, secondPolygon);

      // convexity of the result is already checked in another test
      for (Point2d point : firstList)
      {
         if (!result.isPointInside(point))
         {
            double distance = result.distance(point);

            if (distance > 1e-7)
               throw new RuntimeException("Not each point is inside the result. distance = " + distance);
         }

//       assertTrue("Not each point isinside the result. distance = " , result.isPointInside(point));
      }

      for (Point2d point : secondList)
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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMovePointInsidePolygonAlongVector()
   {
      int nTests = 1000;
      int testNumber = 0;
      ReferenceFrame frame = ReferenceFrame.getWorldFrame();

      int minPoints = 3;
      int maxPoints = 10;
      while (testNumber < nTests)
      {
         ArrayList<FramePoint2d> points = new ArrayList<FramePoint2d>();
         int nPoints = minPoints + random.nextInt(maxPoints - minPoints);
         for (int i = 0; i < nPoints; i++)
         {
            points.add(new FramePoint2d(frame, random.nextDouble() - 0.5, random.nextDouble() - 0.5));
         }

         FrameConvexPolygon2d polygon = new FrameConvexPolygon2d(points);

         FramePoint2d pointToMove = new FramePoint2d(frame, 2.0 * (random.nextDouble() - 0.5), 2.0 * (random.nextDouble() - 0.5));

         FramePoint2d pointInside = new FramePoint2d(frame, random.nextDouble() - 0.5, random.nextDouble() - 0.5);
         if (polygon.isPointInside(pointInside))
         {
            // do test
            FrameVector2d vector = new FrameVector2d(pointToMove, pointInside);
            FrameLine2d line = new FrameLine2d(pointInside, vector);
            FramePoint2d[] intersections = polygon.intersectionWith(line);
            double distanceBetweenIntersections = intersections[0].distance(intersections[1]);

            double scaling = 0.6;    // larger than 0.5 means that distanceToBeInside could be infeasible
            double distanceToBeInside = distanceBetweenIntersections * random.nextDouble() * scaling;
            boolean feasible = distanceToBeInside < distanceBetweenIntersections / 2.0;

            GeometryTools.movePointInsidePolygonAlongVector(pointToMove, vector, polygon, distanceToBeInside);

            if (feasible)
            {
               for (int i = 0; i < intersections.length; i++)
               {
                  double distanceToIntersection = pointToMove.distance(intersections[i]);
                  assertTrue(distanceToIntersection >= distanceToBeInside - 1e-12);
               }

               assertTrue(polygon.isPointInside(pointToMove, 1e-12));
               assertTrue(line.containsEpsilon(pointToMove, 1e-12));
            }
            else
            {
               assertTrue(polygon.isPointInside(pointToMove, 1e-12));
               assertTrue(line.containsEpsilon(pointToMove, 1e-12));
               FrameLineSegment2d intersectionsLineSegment = new FrameLineSegment2d(intersections[0], intersections[1]);
               assertEquals(0.5, intersectionsLineSegment.percentageAlongLineSegment(pointToMove), 1e-12);
            }


            testNumber++;
         }

      }

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPolygonsNegativeAngle()
   {
      assertPolygons(new double[]
      {
         0, 5, 2, -2, 2, 0
      }, new double[]
      {
         2.5, 1, 2.8, 1, 3, .9, 4, 0, 3, -1
      }, new double[] {2, 0, 46.0 / 17, 6.0 / 34}, .001);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPolygonsThirdQuadrant()
   {
      assertPolygons(new double[]
      {
         -2, -1, -1, -1, -1, -2
      }, new double[]
      {
         -2, -2, -2, -3, -4, -4, -4, -2
      }, new double[] {-1.5, -1.5, -2, -2}, .001);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPolygonsNegativeAngleAndTwoVisibleVerticesOnPolygon1()
   {
      assertPolygons(new double[]
      {
         0, 0, 1, 2, 1, 0
      }, new double[]
      {
         2, 2, 0, 3, -1, 4
      }, new double[] {1, 2, 1.2, 2.4}, .001);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPolygonsParalellEdges()
   {
      assertPolygons(new double[]
      {
         0, 0, 0, 1, 1, 0, 2, 1, 1, 2
      }, new double[]
      {
         0, 3, 2, 3, -1, 4, 3, 4
      }, new double[] {1, 2, 1, 3}, .001);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPolygonsMultiplePossibleAnswers()
   {
      assertPolygons(new double[]
      {
         0, 0, 0, 1, 1, 0, 2, 1, 1, 2
      }, new double[]
      {
         3, 2, 2, 3, 2, 4, 4, 2
      }, new double[] {1, 2, 2, 3}, .001);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPolygonsTwoVisiblePoints()
   {
      assertPolygons(new double[]
      {
         0, 0, 0, 1, 1, 0, 2, 1, 1, 2
      }, new double[]
      {
         4, 1, 1, 4, 2, 4, 4, 2
      }, new double[] {2, 1, 3, 2}, .001);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPolygonsTwoVisiblePoints2()
   {
      assertPolygons(new double[]
      {
         0, 0, 0, 1, 1, 0, 2, 1, 1, 2
      }, new double[]
      {
         4, 1, 1.5, 4, 2, 4, 4, 2
      }, new double[] {2, 1, 194.0 / 61, 121.0 / 61}, .001);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPolygonsOneOfTheAnglesIsZero()
   {
      assertPolygons(new double[]
      {
         0, 0, 0, 1, 1, 0, 2, 1, 1, 2
      }, new double[]
      {
         0, 2, 0, 3, 1, 3, .8, 2
      }, new double[] {.9, 1.9, .8, 2}, .001);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPolygonsTriangles()
   {
      assertPolygons(new double[]
      {
         0, 1, 1, 0, 2, 0
      }, new double[]
      {
         0, 3, 4, 3, 1, 2
      }, new double[] {.4, .8, 1, 2}, .001);
   }

// @Test(timeout=300000)
   public void testDistanceBetweenPolygonsSharedPoint()
   {
      assertPolygons(new double[]
      {
         0, 0, 0, 1, 1, 0, 2, 1, 1, 2
      }, new double[]
      {
         0, 2, 0, 3, 1, 3, 1, 2
      }, new double[] {1, 2, 1, 2}, .001);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPolygonsPointOnEdge()
   {
      assertPolygons(new double[]
      {
         0, 0, 0, 1, 1, 0, 2, 1, 1, 2
      }, new double[]
      {
         0, 2, 0, 3, 1, 3, .5, 1.5
      }, new double[] {.5, 1.5, .5, 1.5}, .001);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPolygonsNegativeAngle2()
   {
      assertPolygons(new double[]
      {
         0, 0, 0, 1, 1, 0, 2, 1, 1, 2
      }, new double[]
      {
         0, 2, 0, 3, 1, 3, .4, 1.5
      }, new double[] {.45, 1.45, .4, 1.5}, .001);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPolygonsSolutionIsTwoVertices()
   {
      assertPolygons(new double[]
      {
         0, 0, 2, 0, 2, 2
      }, new double[]
      {
         4, 3, 6, 3, 6, 7
      }, new double[] {2, 2, 4, 3}, 0);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPolygonsIntersectingPolygons()
   {
      ConvexPolygon2d polygon1 = getPolygon(new double[]
      {
         0, 0, 0, 1, 1, 0, 2, 1, 1, 2
      });
      ConvexPolygon2d polygon2 = getPolygon(new double[]
      {
         1, 1, 0, 3, 2, 2, 3, 0
      });

      try
      {
         GeometryTools.computeMinimumDistancePoints(polygon1, polygon2);
         fail();
      }

      catch (RuntimeException re)
      {
         assertEquals(re.getMessage(), "Cannot compute minimum distance between intersecting polygons.");
      }

      try
      {
         GeometryTools.computeMinimumDistancePoints(polygon2, polygon1);
         fail();
      }

      catch (RuntimeException re)
      {
         assertEquals(re.getMessage(), "Cannot compute minimum distance between intersecting polygons.");
      }
   }

   private void assertPolygons(double[] p1, double[] p2, double[] expectedSolution, double epsilon)
   {
      if (expectedSolution.length != 4)
      {
         throw new RuntimeException("Invalid input.");
      }

      ConvexPolygon2d polygon1 = getPolygon(p1);
      ConvexPolygon2d polygon2 = getPolygon(p2);
      Point2d[] closestPoints = GeometryTools.computeMinimumDistancePoints(polygon1, polygon2);
      Point2d[] closestPointsReversed = GeometryTools.computeMinimumDistancePoints(polygon2, polygon1);
      assertEquals(closestPoints[0].distance(closestPoints[1]), closestPointsReversed[0].distance(closestPointsReversed[1]), epsilon);
      assertEquals(expectedSolution[0], closestPoints[0].getX(), epsilon);
      assertEquals(expectedSolution[1], closestPoints[0].getY(), epsilon);
      assertEquals(expectedSolution[2], closestPoints[1].getX(), epsilon);
      assertEquals(expectedSolution[3], closestPoints[1].getY(), epsilon);
   }

   private ConvexPolygon2d getPolygon(double[] polygon)
   {
      if (polygon.length % 2 != 0)
      {
         throw new RuntimeException("Invalid input.");
      }

      List<Point2d> list = new ArrayList<Point2d>();
      for (int i = 0; i < polygon.length; i += 2)
      {
         list.add(new Point2d(polygon[i], polygon[i + 1]));
      }

      return new ConvexPolygon2d(list);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetAngleFromFirstToSecondVector() throws Exception
   {
      Random random = new Random(51651L);

      for (int i = 0; i<1000; i++)
      {
         double firstVectorLength = RandomTools.generateRandomDouble(random, 0.0, 10.0);
         double secondVectorLength = RandomTools.generateRandomDouble(random, 0.0, 10.0);
         Vector2d firstVector = RandomTools.generateRandomVector2d(random, firstVectorLength);
         Vector2d secondVector = new Vector2d();

         for (double yaw = -Math.PI; yaw <= Math.PI; yaw += Math.PI / 100.0)
         {
            double c = Math.cos(yaw);
            double s = Math.sin(yaw);
            secondVector.setX(firstVector.getX() * c - firstVector.getY() * s);
            secondVector.setY(firstVector.getX() * s + firstVector.getY() * c);
            secondVector.scale(secondVectorLength / firstVectorLength);
            double computedYaw = GeometryTools.getAngleFromFirstToSecondVector(firstVector, secondVector);
            double yawDifference = AngleTools.computeAngleDifferenceMinusPiToPi(yaw, computedYaw);
            assertEquals(0.0, yawDifference, EPSILON);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetClosestPointsForTwoLines() throws Exception
   {
      Point3d expectedPointOnLine1ToPack = new Point3d();
      Point3d expectedPointOnLine2ToPack = new Point3d();

      Point3d actualPointOnLine1ToPack = new Point3d();
      Point3d actualPointOnLine2ToPack = new Point3d();

      for (int i = 0; i < 100; i++)
      {
         Point3d lineStart1 = RandomTools.generateRandomPoint3d(random, -10.0, 10.0);
         Vector3d lineDirection1 = RandomTools.generateRandomVector(random, RandomTools.generateRandomDouble(random, 0.0, 10.0));

         // Make line2 == line1
         Point3d lineStart2 = new Point3d(lineStart1);
         Vector3d lineDirection2 = new Vector3d(lineDirection1);

         // Shift orthogonally line2 away from line1.
         Vector3d orthogonalToLine1 = RandomTools.generateRandomOrthogonalVector3d(random, lineDirection1, true);
         double expectedMinimumDistance = RandomTools.generateRandomDouble(random, 0.0, 10.0);
         lineStart2.scaleAdd(expectedMinimumDistance, orthogonalToLine1, lineStart1);

         // Rotate line2 around the vector we shifted it along, so it preserves the minimum distance.
         AxisAngle4d axisAngleAroundShiftVector = new AxisAngle4d(orthogonalToLine1, RandomTools.generateRandomDouble(random, Math.PI));
         Matrix3d rotationMatrixAroundShiftVector = new Matrix3d();
         rotationMatrixAroundShiftVector.set(axisAngleAroundShiftVector);
         rotationMatrixAroundShiftVector.transform(lineDirection2);

         // At this point, lineStart1 and lineStart2 are expected to be the closest points.
         expectedPointOnLine1ToPack.set(lineStart1);
         expectedPointOnLine2ToPack.set(lineStart2);

         GeometryTools.getClosestPointsForTwoLines(lineStart1, lineDirection1, lineStart2, lineDirection2, actualPointOnLine1ToPack, actualPointOnLine2ToPack);
         JUnitTools.assertTuple3dEquals(expectedPointOnLine1ToPack, actualPointOnLine1ToPack, EPSILON);
         JUnitTools.assertTuple3dEquals(expectedPointOnLine2ToPack, actualPointOnLine2ToPack, EPSILON);

         // Let's shift lineStart1 and lineStart2 along their respective line direction so they're not the closest points.
         lineStart1.scaleAdd(RandomTools.generateRandomDouble(random, 10.0), lineDirection1, lineStart1);
         lineStart2.scaleAdd(RandomTools.generateRandomDouble(random, 10.0), lineDirection2, lineStart2);

         GeometryTools.getClosestPointsForTwoLines(lineStart1, lineDirection1, lineStart2, lineDirection2, actualPointOnLine1ToPack, actualPointOnLine2ToPack);
         JUnitTools.assertTuple3dEquals(expectedPointOnLine1ToPack, actualPointOnLine1ToPack, EPSILON);
         JUnitTools.assertTuple3dEquals(expectedPointOnLine2ToPack, actualPointOnLine2ToPack, EPSILON);
      }

      // Test the parallel case. There's an infinite number of solutions but only one minimum distance between the two lines.
      for (int i = 0; i < 100; i++)
      {
         Point3d lineStart1 = RandomTools.generateRandomPoint3d(random, -10.0, 10.0);
         Vector3d lineDirection1 = RandomTools.generateRandomVector(random, 1.0);

         // Make line2 == line1
         Point3d lineStart2 = new Point3d(lineStart1);
         Vector3d lineDirection2 = new Vector3d(lineDirection1);

         // Shift orthogonally line2 away from line1.
         Vector3d orthogonalToLine1 = RandomTools.generateRandomOrthogonalVector3d(random, lineDirection1, true);
         double expectedMinimumDistance = RandomTools.generateRandomDouble(random, 0.0, 10.0);
         lineStart2.scaleAdd(expectedMinimumDistance, orthogonalToLine1, lineStart1);

         GeometryTools.getClosestPointsForTwoLines(lineStart1, lineDirection1, lineStart2, lineDirection2, actualPointOnLine1ToPack, actualPointOnLine2ToPack);
         double actualMinimumDistance = actualPointOnLine1ToPack.distance(actualPointOnLine2ToPack);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);

         // Let's shift lineStart1 and lineStart2 along their respective line direction (the minimum distance should remain the same).
         lineStart1.scaleAdd(RandomTools.generateRandomDouble(random, 10.0), lineDirection1, lineStart1);
         lineStart2.scaleAdd(RandomTools.generateRandomDouble(random, 10.0), lineDirection2, lineStart2);

         GeometryTools.getClosestPointsForTwoLines(lineStart1, lineDirection1, lineStart2, lineDirection2, actualPointOnLine1ToPack, actualPointOnLine2ToPack);
         actualMinimumDistance = actualPointOnLine1ToPack.distance(actualPointOnLine2ToPack);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testIsPointInsideTriangleABC() throws Exception
   {
      Point2d inside = new Point2d();
      Point2d outside = new Point2d();

      for (int i = 0; i < 100; i++)
      {
         Point2d a = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         Point2d b = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         Point2d c = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);

         assertTrue(GeometryTools.isPointInsideTriangleABC(a, a, b, c));
         assertTrue(GeometryTools.isPointInsideTriangleABC(a, c, b, a));
         assertTrue(GeometryTools.isPointInsideTriangleABC(b, a, b, c));
         assertTrue(GeometryTools.isPointInsideTriangleABC(b, c, b, a));
         assertTrue(GeometryTools.isPointInsideTriangleABC(c, a, b, c));
         assertTrue(GeometryTools.isPointInsideTriangleABC(c, c, b, a));

         inside.interpolate(a, b, RandomTools.generateRandomDouble(random, 0.0, 1.0));
         inside.interpolate(inside, c, RandomTools.generateRandomDouble(random, 0.0, 1.0));
         assertTrue(GeometryTools.isPointInsideTriangleABC(inside, a, b, c));
         assertTrue(GeometryTools.isPointInsideTriangleABC(inside, c, b, a));

         outside.interpolate(a, b, RandomTools.generateRandomDouble(random, 1.0, 10.0));
         outside.interpolate(outside, c, RandomTools.generateRandomDouble(random, 0.0, 1.0));
         assertFalse(GeometryTools.isPointInsideTriangleABC(outside, a, b, c));
         assertFalse(GeometryTools.isPointInsideTriangleABC(outside, c, b, a));
         
         outside.interpolate(a, b, RandomTools.generateRandomDouble(random, -10.0, 0.0));
         outside.interpolate(outside, c, RandomTools.generateRandomDouble(random, 0.0, 1.0));
         assertFalse(GeometryTools.isPointInsideTriangleABC(outside, a, b, c));
         assertFalse(GeometryTools.isPointInsideTriangleABC(outside, c, b, a));

         outside.interpolate(a, b, RandomTools.generateRandomDouble(random, 0.0, 1.0));
         outside.interpolate(outside, c, RandomTools.generateRandomDouble(random, 1.0, 10.0));
         assertFalse(GeometryTools.isPointInsideTriangleABC(outside, a, b, c));
         assertFalse(GeometryTools.isPointInsideTriangleABC(outside, c, b, a));
         
         outside.interpolate(a, b, RandomTools.generateRandomDouble(random, 0.0, 1.0));
         outside.interpolate(outside, c, RandomTools.generateRandomDouble(random, -10.0, 0.0));
         assertFalse(GeometryTools.isPointInsideTriangleABC(outside, a, b, c));
         assertFalse(GeometryTools.isPointInsideTriangleABC(outside, c, b, a));
      }

      Point2d a = new Point2d(1.0, 0.0);
      Point2d b = new Point2d(1.0, 1.0);
      Point2d c = new Point2d(0.0, 1.0);

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
      // Test for right rectangle, should be half the area of the corresponding rectangle
      for (int i = 0; i < 100; i++)
      {
         Point2d a = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         Point2d b = new Point2d();
         Point2d c = new Point2d();
         Point2d d = new Point2d();

         Vector2d rectangleLength = RandomTools.generateRandomVector2d(random, 1.0);
         Vector2d rectangleWidth = new Vector2d(-rectangleLength.getY(), rectangleLength.getX());
         double length = RandomTools.generateRandomDouble(random, 0.0, 10.0);
         double width = RandomTools.generateRandomDouble(random, 0.0, 10.0);
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
      Vector3d actualVector;
      Vector3d expectedVector = new Vector3d();

      for (int i = 0; i < 25; i++)
      {
         actualVector = RandomTools.generateRandomVector(random, RandomTools.generateRandomDouble(random, Epsilons.ONE_TRILLIONTH, 10.0));

         expectedVector.normalize(actualVector);
         GeometryTools.normalizeSafelyZUp(actualVector);
         JUnitTools.assertTuple3dEquals(expectedVector, actualVector, Epsilons.ONE_TRILLIONTH);

         actualVector = RandomTools.generateRandomVector(random, 0.999 * Epsilons.ONE_TRILLIONTH);
         expectedVector.set(0.0, 0.0, 1.0);
         GeometryTools.normalizeSafelyZUp(actualVector);
         JUnitTools.assertTuple3dEquals(expectedVector, actualVector, Epsilons.ONE_TRILLIONTH);

         actualVector = new Vector3d();
         expectedVector.set(0.0, 0.0, 1.0);
         GeometryTools.normalizeSafelyZUp(actualVector);
         JUnitTools.assertTuple3dEquals(expectedVector, actualVector, Epsilons.ONE_TRILLIONTH);
      }
   }

   @Test
   public void testGetIntersectionBetweenLineAndPlane() throws Exception
   {
      for (int i = 0; i < 100; i++)
      {
         Point3d pointOnPlane = RandomTools.generateRandomPoint3d(random, -10.0, 10.0);
         Vector3d planeNormal = RandomTools.generateRandomVector(random, RandomTools.generateRandomDouble(random, 10.0));
         Vector3d parallelToPlane = RandomTools.generateRandomOrthogonalVector3d(random, planeNormal, true);

         Point3d expectedIntersection = new Point3d();
         expectedIntersection.scaleAdd(RandomTools.generateRandomDouble(random, 10.0), parallelToPlane, pointOnPlane);

         Vector3d lineDirection = RandomTools.generateRandomVector(random, RandomTools.generateRandomDouble(random, 10.0));
         Point3d pointOnLine = new Point3d();
         pointOnLine.scaleAdd(RandomTools.generateRandomDouble(random, 10.0), lineDirection, expectedIntersection);

         Point3d actualIntersection = GeometryTools.getIntersectionBetweenLineAndPlane(pointOnPlane, planeNormal, pointOnLine, lineDirection);
         JUnitTools.assertTuple3dEquals(expectedIntersection, actualIntersection, Epsilons.ONE_TRILLIONTH);
      }

      // Try parallel lines to plane
      for (int i = 0; i < 100; i++)
      {
         Point3d pointOnPlane = RandomTools.generateRandomPoint3d(random, -10.0, 10.0);
         Vector3d planeNormal = RandomTools.generateRandomVector(random, RandomTools.generateRandomDouble(random, 10.0));

         Vector3d lineDirection = RandomTools.generateRandomOrthogonalVector3d(random, planeNormal, false);
         Point3d pointOnLine = new Point3d();
         pointOnLine.scaleAdd(RandomTools.generateRandomDouble(random, 10.0), lineDirection, pointOnPlane);

         Point3d actualIntersection = GeometryTools.getIntersectionBetweenLineAndPlane(pointOnPlane, planeNormal, pointOnLine, lineDirection);
         assertNull(actualIntersection);

         pointOnLine.scaleAdd(RandomTools.generateRandomDouble(random, 1.0), planeNormal, pointOnLine);
         actualIntersection = GeometryTools.getIntersectionBetweenLineAndPlane(pointOnPlane, planeNormal, pointOnLine, lineDirection);
         assertNull(actualIntersection);
      }
   }

   @Test
   public void testGetIntersectionBetweenLineSegmentAndPlane2() throws Exception
   {
      Point3d endPoint0 = new Point3d();
      Point3d endPoint1 = new Point3d();

      for (int i = 0; i < 100; i++)
      {
         Point3d pointOnPlane = RandomTools.generateRandomPoint3d(random, -10.0, 10.0);
         Vector3d planeNormal = RandomTools.generateRandomVector(random, RandomTools.generateRandomDouble(random, 10.0));
         Vector3d parallelToPlane = RandomTools.generateRandomOrthogonalVector3d(random, planeNormal, true);

         Point3d expectedIntersection = new Point3d();
         expectedIntersection.scaleAdd(RandomTools.generateRandomDouble(random, 10.0), parallelToPlane, pointOnPlane);

         Vector3d lineDirection = RandomTools.generateRandomVector(random, RandomTools.generateRandomDouble(random, 10.0));

         // Expecting an actual intersection
         endPoint0.scaleAdd(RandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         endPoint1.scaleAdd(RandomTools.generateRandomDouble(random, -10.0, 0.0), lineDirection, expectedIntersection);
         Point3d actualIntersection = GeometryTools.getIntersectionBetweenLineSegmentAndPlane(pointOnPlane, planeNormal, endPoint0, endPoint1);
         JUnitTools.assertTuple3dEquals(expectedIntersection, actualIntersection, Epsilons.ONE_TRILLIONTH);
         actualIntersection = GeometryTools.getIntersectionBetweenLineSegmentAndPlane(pointOnPlane, planeNormal, endPoint1, endPoint0);
         JUnitTools.assertTuple3dEquals(expectedIntersection, actualIntersection, Epsilons.ONE_TRILLIONTH);

         // Expecting no intersection
         endPoint0.scaleAdd(RandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         endPoint1.scaleAdd(RandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         actualIntersection = GeometryTools.getIntersectionBetweenLineSegmentAndPlane(pointOnPlane, planeNormal, endPoint0, endPoint1);
         assertNull(actualIntersection);
         actualIntersection = GeometryTools.getIntersectionBetweenLineSegmentAndPlane(pointOnPlane, planeNormal, endPoint1, endPoint0);
         assertNull(actualIntersection);
      }

      // Try parallel lines to plane
      for (int i = 0; i < 100; i++)
      {
         Point3d pointOnPlane = RandomTools.generateRandomPoint3d(random, -10.0, 10.0);
         Vector3d planeNormal = RandomTools.generateRandomVector(random, RandomTools.generateRandomDouble(random, 10.0));

         Vector3d lineDirection = RandomTools.generateRandomOrthogonalVector3d(random, planeNormal, false);
         endPoint0.scaleAdd(RandomTools.generateRandomDouble(random, 10.0), lineDirection, pointOnPlane);
         endPoint1.scaleAdd(RandomTools.generateRandomDouble(random, 10.0), lineDirection, pointOnPlane);

         Point3d actualIntersection = GeometryTools.getIntersectionBetweenLineSegmentAndPlane(pointOnPlane, planeNormal, endPoint0, endPoint1);
         assertNull(actualIntersection);

         double distanceAwayFromPlane = RandomTools.generateRandomDouble(random, 1.0);
         endPoint0.scaleAdd(distanceAwayFromPlane, planeNormal, endPoint0);
         endPoint1.scaleAdd(distanceAwayFromPlane, planeNormal, endPoint0);
         actualIntersection = GeometryTools.getIntersectionBetweenLineSegmentAndPlane(pointOnPlane, planeNormal, endPoint0, endPoint1);
         assertNull(actualIntersection);
      }
   }

   public static void main(String[] args)
   {
      String targetTests = GeometryToolsTest.class.getName();
      String targetClasses = GeometryTools.class.getName();
      MutationTestingTools.doPITMutationTestAndOpenResult(targetTests, targetClasses);
   }
}
