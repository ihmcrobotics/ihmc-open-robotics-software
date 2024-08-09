package us.ihmc.robotics.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.util.ArrayList;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.log.LogTools;

public class GeometryToolsTest
{
   private static final int ITERATIONS = 1000;

   @BeforeEach
   public void setUp() throws Exception
   {
   }

   @AfterEach
   public void tearDown() throws Exception
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   private static final double EPSILON = 1e-6;

   @Test
   public void testGetDistanceBetweenPointAndPlane1()
   {
      FramePoint3D pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      FrameVector3D planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      FramePoint3D point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 3);
      double actual = GeometryTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      double expected = 3.0;
      assertEquals(expected, actual, EPSILON, "FAILED: Distance from point to plane");

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 3, 3, -3);
      actual = GeometryTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      expected = 3.0;
      assertEquals(expected, actual, EPSILON, "FAILED: Distance from point to plane");

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, -3);
      actual = GeometryTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      expected = 3.0;
      assertEquals(expected, actual, EPSILON, "FAILED: Distance from point to plane");

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 3);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, -3);
      actual = GeometryTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      expected = 6.0;
      assertEquals(expected, actual, EPSILON, "FAILED: Distance from point to plane");

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 1, 0, 0);
      point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 3, 0, 0);
      actual = GeometryTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      expected = 3.0;
      assertEquals(expected, actual, EPSILON, "FAILED: Distance from point to plane");

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 1, 0);
      point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 3, 0);
      actual = GeometryTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      expected = 3.0;
      assertEquals(expected, actual, EPSILON, "FAILED: Distance from point to plane");

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1, 1, 1);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 1, 0);
      point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 3, 0);
      actual = GeometryTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      expected = 2.0;
      assertEquals(expected, actual, EPSILON, "FAILED: Distance from point to plane");
   }

   @Test
   public void testIsLineSegmentIntersectingPlane1()
   {
      FramePoint3D pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      FrameVector3D planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      FramePoint3D lineStart = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, -1);
      FramePoint3D lineEnd = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 3);
      assertTrue(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, lineStart, lineEnd));

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 1, 0, 0);
      lineStart = new FramePoint3D(ReferenceFrame.getWorldFrame(), -6, 3, -3);
      lineEnd = new FramePoint3D(ReferenceFrame.getWorldFrame(), 6, 3, 6);
      assertTrue(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, lineStart, lineEnd));

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 1, 0);
      lineStart = new FramePoint3D(ReferenceFrame.getWorldFrame(), 6, -3, -3);
      lineEnd = new FramePoint3D(ReferenceFrame.getWorldFrame(), 6, 3, 6);
      assertTrue(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, lineStart, lineEnd));

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      lineStart = new FramePoint3D(ReferenceFrame.getWorldFrame(), 6, -3, 3);
      lineEnd = new FramePoint3D(ReferenceFrame.getWorldFrame(), 6, 3, 6);
      assertFalse(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, lineStart, lineEnd));

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      lineStart = new FramePoint3D(ReferenceFrame.getWorldFrame(), 6, -3, -3);
      lineEnd = new FramePoint3D(ReferenceFrame.getWorldFrame(), 6, 3, -1);
      assertFalse(GeometryTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, lineStart, lineEnd));
   }

   @Test
   public void testGetPerpendicularVectorFromLineToPoint1()
   {
      FramePoint3D point0 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      FramePoint3D lineStart0 = new FramePoint3D(ReferenceFrame.getWorldFrame(), -10, 10, 0);
      FramePoint3D lineEnd0 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 10, 10, 0);
      FramePoint3D intersectionPoint0 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 10, 0);
      FrameVector3D x0 = new FrameVector3D(point0.getReferenceFrame());
      x0.sub(point0, intersectionPoint0);
      FrameVector3D expectedReturn0 = x0;
      FrameVector3D actualReturn0 = GeometryTools.getPerpendicularVectorFromLineToPoint(point0, lineStart0, lineEnd0, intersectionPoint0);

      assertTrue(expectedReturn0.epsilonEquals(actualReturn0, EPSILON), "Test Failed");

      FramePoint3D point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 4, 2, 0);
      FramePoint3D lineStart = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      FramePoint3D lineEnd = new FramePoint3D(ReferenceFrame.getWorldFrame(), 10, 10, 0);
      FramePoint3D intersectionPoint = new FramePoint3D(ReferenceFrame.getWorldFrame(), 3, 3, 0);
      FrameVector3D x = new FrameVector3D(point.getReferenceFrame());
      x.sub(point, intersectionPoint);
      FrameVector3D expectedReturn = x;
      FrameVector3D actualReturn = GeometryTools.getPerpendicularVectorFromLineToPoint(point, lineStart, lineEnd, intersectionPoint);
      assertTrue(expectedReturn.epsilonEquals(actualReturn, EPSILON), "Test Failed");

      FramePoint3D point1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), -2.5, 1.5, 0);
      FramePoint3D lineStart1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      FramePoint3D lineEnd1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), -4, 4, 0);
      FramePoint3D intersectionPoint1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), -2, 2, 0);

      EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(new Point2D(-2.5, 1.5), new Point2D(0, 0), new Point2D(-4, 4));
      FrameVector3D x1 = new FrameVector3D(point1.getReferenceFrame());
      x1.sub(point1, intersectionPoint1);
      FrameVector3D expectedReturn1 = x1;
      FrameVector3D actualReturn1 = GeometryTools.getPerpendicularVectorFromLineToPoint(point1, lineStart1, lineEnd1, intersectionPoint1);

      assertTrue(expectedReturn1.epsilonEquals(actualReturn1, EPSILON), "Test Failed");
   }

   @Test
   public void testGetPlaneNormalGivenThreePoints()
   {
      FramePoint3D point1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 5, 0);
      FramePoint3D point2 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 5, 0);
      FramePoint3D point3 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 5, 0);
      FrameVector3D expectedReturn = null;
      FrameVector3D actualReturn = GeometryTools.getPlaneNormalGivenThreePoints(point1, point2, point3);
      assertEquals(expectedReturn, actualReturn, "test failed");

      FramePoint3D point91 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 3, 0);
      FramePoint3D point92 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 5, 0);
      FramePoint3D point93 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 1, 0);
      FrameVector3D expectedReturn9 = null;
      FrameVector3D actualReturn9 = GeometryTools.getPlaneNormalGivenThreePoints(point91, point92, point93);
      assertEquals(expectedReturn9, actualReturn9, "test failed");

      FramePoint3D point81 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 9, 0, 0);
      FramePoint3D point82 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 7, 0, 0);
      FramePoint3D point83 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 4, 0, 0);
      FrameVector3D expectedReturn8 = null;
      FrameVector3D actualReturn8 = GeometryTools.getPlaneNormalGivenThreePoints(point81, point82, point83);
      assertEquals(expectedReturn8, actualReturn8, "test failed");

      FramePoint3D point71 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 4);
      FramePoint3D point72 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 6);
      FramePoint3D point73 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 7);
      FrameVector3D expectedReturn7 = null;
      FrameVector3D actualReturn7 = GeometryTools.getPlaneNormalGivenThreePoints(point71, point72, point73);
      assertEquals(expectedReturn7, actualReturn7, "test failed");

      FramePoint3D point11 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 5, 46);
      FramePoint3D point12 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 587, 3);
      FramePoint3D point13 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 18, 8);
      FramePoint3D p1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 5, 5);
      FramePoint3D v1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1, 5, 5);
      FrameVector3D expectedReturn1 = new FrameVector3D(p1.getReferenceFrame());
      expectedReturn1.sub(p1, v1);
      FrameVector3D actualReturn1 = GeometryTools.getPlaneNormalGivenThreePoints(point11, point12, point13);
      assertTrue(expectedReturn1.epsilonEquals(actualReturn1, EPSILON), "Test Failed");

      FramePoint3D point21 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 65, 0, 46);
      FramePoint3D point22 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 43, 0, 3);
      FramePoint3D point23 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 13, 0, 8);
      FramePoint3D p2 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 1, 5);
      FramePoint3D v2 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 5);
      FrameVector3D expectedReturn2 = new FrameVector3D(p2.getReferenceFrame());
      expectedReturn2.sub(p2, v2);
      FrameVector3D actualReturn2 = GeometryTools.getPlaneNormalGivenThreePoints(point21, point22, point23);
      assertTrue(expectedReturn2.epsilonEquals(actualReturn2, EPSILON), "Test Failed");

      FramePoint3D point31 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 65, 56, 0);
      FramePoint3D point32 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 43, 3, 0);
      FramePoint3D point33 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 13, 87, 0);
      FramePoint3D p3 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 55, 0);
      FramePoint3D v3 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 55, 1);
      FrameVector3D expectedReturn3 = new FrameVector3D(p3.getReferenceFrame());
      expectedReturn3.sub(p3, v3);
      FrameVector3D actualReturn3 = GeometryTools.getPlaneNormalGivenThreePoints(point31, point32, point33);
      assertTrue(expectedReturn3.epsilonEquals(actualReturn3, EPSILON), "Test Failed");
   }

   @Test
   public void testIsPointOnLeftSideOfLine()
   {
      FramePoint3D lineStart = new FramePoint3D(ReferenceFrame.getWorldFrame(), 5.0, 0.0, 0.0);
      FramePoint3D lineEnd = new FramePoint3D(ReferenceFrame.getWorldFrame(), 5.0, 10.0, 0.0);

      FramePoint3D point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 10, 5, 0.0);

      boolean expectedReturn = false;
      boolean actualReturn = GeometryTools.isPointOnLeftSideOfLine(point, lineStart, lineEnd);
      assertEquals(expectedReturn, actualReturn, "return value");

      /** @todo fill in the test code */
   }

   @Test
   public void testClipToBoundingBox()
   {
      Tuple3DBasics tuple3d = new Point3D(1.0, -1.0, 0.0);
      GeometryTools.clipToBoundingBox(tuple3d, -0.5, 0.5, 0.5, -0.5, 0.0, 0.0);
      EuclidCoreTestTools.assertEquals("not equal", new Point3D(0.5, -0.5, 0.0), tuple3d, 0.0);
      tuple3d.set(1.0, -1.0, 0.0);
      GeometryTools.clipToBoundingBox(tuple3d, 0.5, -0.5, -0.5, 0.5, -0.1, 0.1);
      EuclidCoreTestTools.assertEquals("not equal", new Point3D(0.5, -0.5, 0.0), tuple3d, 0.0);
      tuple3d.set(1.0, -1.0, 2.0);
      GeometryTools.clipToBoundingBox(tuple3d, 0.5, -0.5, -0.5, 0.5, -0.1, 1.0);
      EuclidCoreTestTools.assertEquals("not equal", new Point3D(0.5, -0.5, 1.0), tuple3d, 0.0);
   }

   @Test
   public void testCombine()
   {
      Random random = new Random(1176L);
      ArrayList<Point2D> firstList = new ArrayList<Point2D>();
      for (int i = 0; i < 100; i++)
      {
         firstList.add(new Point2D(random.nextDouble(), random.nextDouble()));
      }

      ConvexPolygon2D firstPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(firstList));

      ArrayList<Point2D> secondList = new ArrayList<Point2D>();
      for (int i = 0; i < 200; i++)
      {
         secondList.add(new Point2D(random.nextDouble(), random.nextDouble()));
      }

      ConvexPolygon2D secondPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(secondList));

      ConvexPolygon2D result = new ConvexPolygon2D(firstPolygon, secondPolygon);

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

   @Test
   public void testNormalizeSafeZUp() throws Exception
   {
      Vector3D actualVector;
      Vector3D expectedVector = new Vector3D();
      Random random = new Random(1176L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         actualVector = EuclidCoreRandomTools.nextVector3D(random, RandomNumbers.nextDouble(random, Epsilons.ONE_TRILLIONTH, 10.0));

         expectedVector.setAndNormalize(actualVector);
         GeometryTools.normalizeSafelyZUp(actualVector);
         EuclidCoreTestTools.assertEquals(expectedVector, actualVector, Epsilons.ONE_TRILLIONTH);

         actualVector = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 0.999 * Epsilons.ONE_TRILLIONTH);
         expectedVector.set(0.0, 0.0, 1.0);
         GeometryTools.normalizeSafelyZUp(actualVector);
         EuclidCoreTestTools.assertEquals(expectedVector, actualVector, Epsilons.ONE_TRILLIONTH);

         actualVector = new Vector3D();
         expectedVector.set(0.0, 0.0, 1.0);
         GeometryTools.normalizeSafelyZUp(actualVector);
         EuclidCoreTestTools.assertEquals(expectedVector, actualVector, Epsilons.ONE_TRILLIONTH);
      }
   }

   @Test
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
         assertEquals(isTuple2dZero, GeometryTools.isZero(new Point2D(x, y), epsilon));
         assertEquals(isTuple2dZero, GeometryTools.isZero(new Point2D(-x, y), epsilon));
         assertEquals(isTuple2dZero, GeometryTools.isZero(new Point2D(-x, -y), epsilon));
         assertEquals(isTuple2dZero, GeometryTools.isZero(new Point2D(x, -y), epsilon));

         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(x, y, z), epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(x, y, -z), epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(x, -y, z), epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(x, -y, -z), epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(-x, y, z), epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(-x, y, -z), epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(-x, -y, z), epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(-x, -y, -z), epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(x, y, z), epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(x, y, -z), epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(x, -y, z), epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(x, -y, -z), epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(-x, y, z), epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(-x, y, -z), epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(-x, -y, z), epsilon));
         assertEquals(isTuple3dZero, GeometryTools.isZero(new Point3D(-x, -y, -z), epsilon));
      }
   }

   @Test
   public void testConstructFrameFromPointAndAxis()
   {
      Random random = new Random(1776L);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FramePoint3D randomPoint = new FramePoint3D(worldFrame);

      FrameVector3D randomVector = new FrameVector3D(worldFrame);

      int numberOfTests = 100000;

      for (int i = 0; i < numberOfTests; i++)
      {
         randomPoint.setIncludingFrame(EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 10.0, 10.0, 10.0));
         randomVector.setIncludingFrame(EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0));

         ReferenceFrame frameA = GeometryTools.constructReferenceFrameFromPointAndZAxis("frameA", randomPoint, randomVector);
         ReferenceFrame frameB = GeometryTools.constructReferenceFrameFromPointAndAxis("frameB", randomPoint, Axis3D.Z, randomVector);

         EuclidCoreTestTools.assertEquals(frameA.getTransformToRoot(), frameB.getTransformToRoot(), 1.0e-2);
      }
   }

   @Test
   public void testYawAboutPointRegression()
   {
      double epsilon = 1e-10;
      // Do not change value! For regression.
      Random r = new Random(2899234L);

      ReferenceFrame referenceFrame;
      FramePoint3D pointToYawAbout;
      FramePoint3D point;
      double yaw;
      FramePoint3D result;

      referenceFrame = EuclidFrameRandomTools.nextReferenceFrame("randomFrame", r, ReferenceFrame.getWorldFrame());
      pointToYawAbout = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      point = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      yaw = randomAngle(r);
      result = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      GeometryTools.yawAboutPoint(point, pointToYawAbout, yaw, result);
      System.out.println(result);
      assertEquals(-2681.624165883151, result.getX(), epsilon, "not equal");
      assertEquals(-1528.2007328131492, result.getY(), epsilon, "not equal");
      assertEquals(2998.298763316407, result.getZ(), epsilon, "not equal");

      referenceFrame = EuclidFrameRandomTools.nextReferenceFrame("randomFrame", r, ReferenceFrame.getWorldFrame());
      pointToYawAbout = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      point = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      yaw = randomAngle(r);
      result = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      GeometryTools.yawAboutPoint(point, pointToYawAbout, yaw, result);
      System.out.println(result);
      assertEquals(2868.1077772133904, result.getX(), epsilon, "not equal");
      assertEquals(-3773.703916968001, result.getY(), epsilon, "not equal");
      assertEquals(-3313.247345650209, result.getZ(), epsilon, "not equal");

      referenceFrame = EuclidFrameRandomTools.nextReferenceFrame("randomFrame", r, ReferenceFrame.getWorldFrame());
      pointToYawAbout = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      point = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      yaw = randomAngle(r);
      result = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      GeometryTools.yawAboutPoint(point, pointToYawAbout, yaw, result);
      System.out.println(result);
      assertEquals(9865.290784196699, result.getX(), epsilon, "not equal");
      assertEquals(1276.040690119471, result.getY(), epsilon, "not equal");
      assertEquals(-3096.5574256022164, result.getZ(), epsilon, "not equal");
   }

   @Test
   public void testPitchAboutPointRegression()
   {
      double epsilon = 1e-10;
      // Do not change value! For regression.
      Random r = new Random(689291994L);

      ReferenceFrame referenceFrame;
      FramePoint3D pointToPitchAbout;
      FramePoint3D point;
      double pitch;
      FramePoint3D result;

      referenceFrame = EuclidFrameRandomTools.nextReferenceFrame("randomFrame", r, ReferenceFrame.getWorldFrame());
      pointToPitchAbout = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      point = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      pitch = randomAngle(r);
      result = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      GeometryTools.pitchAboutPoint(point, pointToPitchAbout, pitch, result);
      System.out.println(result);
      assertEquals(-256.24551976827297, result.getX(), epsilon, "not equal");
      assertEquals(1443.7013411938358, result.getY(), epsilon, "not equal");
      assertEquals(11103.259343203952, result.getZ(), epsilon, "not equal");

      referenceFrame = EuclidFrameRandomTools.nextReferenceFrame("randomFrame", r, ReferenceFrame.getWorldFrame());
      pointToPitchAbout = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      point = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      pitch = randomAngle(r);
      result = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      GeometryTools.pitchAboutPoint(point, pointToPitchAbout, pitch, result);
      System.out.println(result);
      assertEquals(-2273.346187036131, result.getX(), epsilon, "not equal");
      assertEquals(3010.5651766598717, result.getY(), epsilon, "not equal");
      assertEquals(-3513.344540982049, result.getZ(), epsilon, "not equal");

      referenceFrame = EuclidFrameRandomTools.nextReferenceFrame("randomFrame", r, ReferenceFrame.getWorldFrame());
      pointToPitchAbout = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      point = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      pitch = randomAngle(r);
      result = new FramePoint3D(referenceFrame, randomScalar(r), randomScalar(r), randomScalar(r));
      GeometryTools.pitchAboutPoint(point, pointToPitchAbout, pitch, result);
      System.out.println(result);
      assertEquals(3978.4131392851787, result.getX(), epsilon, "not equal");
      assertEquals(682.5708442089929, result.getY(), epsilon, "not equal");
      assertEquals(8214.605434738955, result.getZ(), epsilon, "not equal");
   }

   @Test
   public void testYawAboutPoint()
   {
      ReferenceFrame theFrame = ReferenceFrameTools.constructARootFrame("theFrame");
      double epsilon = 1e-10;
      final FramePoint3D pointToYawAboutException = new FramePoint3D(theFrame, 0.0, 0.0, 0.0);
      final FramePoint3D pointException = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 1.0, 1.0);
      final FramePoint3D resultException = new FramePoint3D();
      final double yawException = Math.PI;
      assertThrows(ReferenceFrameMismatchException.class,
                   () -> GeometryTools.yawAboutPoint(pointException, pointToYawAboutException, yawException, resultException));

      FramePoint3D pointToYawAbout = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);
      FramePoint3D point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 1.0, 1.0);
      double yaw = Math.PI;

      FramePoint3D result = new FramePoint3D();
      GeometryTools.yawAboutPoint(point, pointToYawAbout, yaw, result);
      //      System.out.println(result);
      assertEquals(-1.0, result.getX(), epsilon, "These should be equal");
      assertEquals(-1.0, result.getY(), epsilon, "These should be equal");
      assertEquals(1.0, result.getZ(), epsilon, "These should be equal");

      //Check for reference frame mismatch
      FramePoint3D point2 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 1.0, 1.0);
      GeometryTools.yawAboutPoint(point, pointToYawAbout, yaw, point2);

      pointToYawAbout = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);
      point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.0, 1.0);
      yaw = Math.PI / 2;

      result = new FramePoint3D();
      GeometryTools.yawAboutPoint(point, pointToYawAbout, yaw, result);
      //      System.out.println(result);
      assertEquals(0.0, result.getX(), epsilon, "These should be equal");
      assertEquals(1.0, result.getY(), epsilon, "These should be equal");
      assertEquals(1.0, result.getZ(), epsilon, "These should be equal");
   }

   @Test
   public void testPitchAboutPoint()
   {
      ReferenceFrame theFrame = ReferenceFrameTools.constructARootFrame("theFrame");
      double epsilon = 1e-10;
      final FramePoint3D pointToPitchAboutException = new FramePoint3D(theFrame, 0.0, 0.0, 0.0);
      final FramePoint3D pointException = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 1.0, 1.0);
      final FramePoint3D resultException = new FramePoint3D();
      final double pitchException = Math.PI;
      assertThrows(ReferenceFrameMismatchException.class,
                   () -> GeometryTools.yawAboutPoint(pointException, pointToPitchAboutException, pitchException, resultException));

      FramePoint3D pointToPitchAbout = new FramePoint3D(theFrame, 0, 0, 0);
      FramePoint3D point = new FramePoint3D(theFrame, 1, 1, 1);
      double pitch = Math.PI;

      FramePoint3D result = new FramePoint3D();
      GeometryTools.pitchAboutPoint(point, pointToPitchAbout, pitch, result);
      //      System.out.println(result);
      assertEquals(-1.0, result.getX(), epsilon, "These should be equal");
      assertEquals(1.0, result.getY(), epsilon, "These should be equal");
      assertEquals(-1.0, result.getZ(), epsilon, "These should be equal");
   }

   private double randomScalar(Random random)
   {
      return (random.nextDouble() - 0.5) * 10000.0;
   }

   private double randomAngle(Random random)
   {
      return (random.nextDouble() - 0.5) * 2.0 * Math.PI;
   }

   @Test
   public void testYawAboutPoint_FramePoint2d_double()
   {
      ReferenceFrame theFrame = ReferenceFrameTools.constructARootFrame("theFrame");
      ReferenceFrame aFrame = ReferenceFrameTools.constructARootFrame("aFrame");
      double epsilon = 1e-10;

      FramePoint2D original = new FramePoint2D(theFrame, 5.0, 7.0);
      FramePoint2D pointToYawAbout = new FramePoint2D(theFrame);
      double yaw = Math.PI;

      FramePoint2D result = new FramePoint2D(theFrame);
      GeometryTools.yawAboutPoint(original, pointToYawAbout, yaw, result);
      assertEquals(result.getX(), -original.getX(), epsilon, "Should be equal");
      assertEquals(result.getY(), -original.getY(), epsilon, "Should be equal");
      try
      {
         FramePoint2D pointToYawAbout2 = new FramePoint2D(aFrame);
         GeometryTools.yawAboutPoint(original, pointToYawAbout2, yaw, result);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch (ReferenceFrameMismatchException rfme)
      {
         //Good
      }
   }

   @Test
   public void testArePoint3DsSameSideOfPlane3D()
   {
      Random random = new Random(435234657);

      for (int i = 0; i < ITERATIONS; i++)
      { // Create the queries such that they're on the same side of the plane.
         Point3D firstPointOnPlane = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Vector3D planeNormal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Vector3D firstPlaneTangent = EuclidCoreRandomTools.nextOrthogonalVector3D(random, planeNormal, true);
         Vector3D secondPlaneTangent = new Vector3D();
         secondPlaneTangent.cross(planeNormal, firstPlaneTangent);

         Point3D secondPointOnPlane = EuclidGeometryTools.orthogonalProjectionOnPlane3D(EuclidCoreRandomTools.nextPoint3D(random, 10.0),
                                                                                        firstPointOnPlane,
                                                                                        planeNormal);

         // Used to determine the side of the plane
         double sign = random.nextBoolean() ? -1.0 : 1.0;

         Point3D firstQuery = new Point3D(firstPointOnPlane);
         // Shifting the query to the side of the plane
         firstQuery.scaleAdd(sign * EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), planeNormal, firstQuery);
         // Shifting the query without changing side w.r.t. the plane
         firstQuery.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), firstPlaneTangent, firstQuery);
         firstQuery.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), secondPlaneTangent, firstQuery);

         Point3D secondQuery = new Point3D(firstPointOnPlane);
         // Shifting the query to the side of the plane
         secondQuery.scaleAdd(sign * EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), planeNormal, secondQuery);
         // Shifting the query without changing side w.r.t. the plane
         secondQuery.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), firstPlaneTangent, secondQuery);
         secondQuery.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), secondPlaneTangent, secondQuery);

         assertTrue(GeometryTools.arePoint3DsSameSideOfPlane3D(firstQuery, secondQuery, firstPointOnPlane, secondPointOnPlane, firstPlaneTangent));
      }
      for (int i = 0; i < ITERATIONS; i++)
      { // Create the queries such that they're each side of the plane.
         Point3D firstPointOnPlane = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Vector3D planeNormal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Vector3D firstPlaneTangent = EuclidCoreRandomTools.nextOrthogonalVector3D(random, planeNormal, true);
         Vector3D secondPlaneTangent = new Vector3D();
         secondPlaneTangent.cross(planeNormal, firstPlaneTangent);

         Point3D secondPointOnPlane = EuclidGeometryTools.orthogonalProjectionOnPlane3D(EuclidCoreRandomTools.nextPoint3D(random, 10.0),
                                                                                        firstPointOnPlane,
                                                                                        planeNormal);

         // Used to determine the side of the plane
         double sign = random.nextBoolean() ? -1.0 : 1.0;

         Point3D firstQuery = new Point3D(firstPointOnPlane);
         // Shifting the query to the side of the plane
         firstQuery.scaleAdd(sign * EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), planeNormal, firstQuery);
         // Shifting the query without changing side w.r.t. the plane
         firstQuery.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), firstPlaneTangent, firstQuery);
         firstQuery.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), secondPlaneTangent, firstQuery);

         Point3D secondQuery = new Point3D(firstPointOnPlane);
         // Shifting the query to the other side of the plane
         secondQuery.scaleAdd(-sign * EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), planeNormal, secondQuery);
         // Shifting the query without changing side w.r.t. the plane
         secondQuery.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), firstPlaneTangent, secondQuery);
         secondQuery.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), secondPlaneTangent, secondQuery);

         assertFalse(GeometryTools.arePoint3DsSameSideOfPlane3D(firstQuery, secondQuery, firstPointOnPlane, secondPointOnPlane, firstPlaneTangent));
      }
   }

   @Test
   public void testComputeBoundingBoxIntersection3D()
   {
      BoundingBox3D boundingBox1 = new BoundingBox3D(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
      BoundingBox3D boundingBox2 = new BoundingBox3D(0.5, 0.0, 0.0, 1.5, 1.0, 1.0);

      assertEquals(1.0, GeometryTools.computeBoundingBoxVolume3D(boundingBox1), EPSILON, "[FAILED] Volume should be 1.0");
      assertEquals(1.0, GeometryTools.computeBoundingBoxVolume3D(boundingBox2), EPSILON, "[FAILED] Volume should be 1.0");

      assertEquals(1/3.0, GeometryTools.computeIntersectionOverUnionOfTwoBoundingBoxes(boundingBox1, boundingBox2), EPSILON, "[FAILED] Intersection should be 1/3.0");
      assertEquals(0.5, GeometryTools.computeIntersectionOverSmallerOfTwoBoundingBoxes(boundingBox2, boundingBox1), EPSILON, "[FAILED] Intersection should be 0.5");

      boundingBox2 = new BoundingBox3D(0.5, 0.5, 0.5, 1.5, 1.5, 1.5);

      assertEquals(1.0, GeometryTools.computeBoundingBoxVolume3D(boundingBox2), EPSILON, "[FAILED] Volume should be 1.0");

      assertEquals(1/15.0, GeometryTools.computeIntersectionOverUnionOfTwoBoundingBoxes(boundingBox1, boundingBox2), EPSILON, "[FAILED] Intersection should be 1/3.0");
      assertEquals( 1/8.0, GeometryTools.computeIntersectionOverSmallerOfTwoBoundingBoxes(boundingBox2, boundingBox1), EPSILON, "[FAILED] Intersection should be 1/15.0");
   }

   @Test
   public void testPointProjectionsOntoPlane()
   {
      Point3D pointOnPlane = new Point3D(1.0, 1.0, 1.0);
      UnitVector3D planeNormal = new UnitVector3D(1.0, 1.0, 1.0);

      Vector4D plane = new Vector4D(planeNormal.getX(), planeNormal.getY(), planeNormal.getZ(), -pointOnPlane.dot(planeNormal));

      Point3D pointToProject = new Point3D(0.0, 0.0, 0.0);

      Point3D projectedPoint = GeometryTools.projectPointOntoPlane(plane, pointToProject);


      Point3D expectedProjection = new Point3D(1.0, 1.0, 1.0);

      LogTools.info("Projected point: {} {} {}", projectedPoint.getX(), projectedPoint.getY(), projectedPoint.getZ());
      LogTools.info("Expected projection: {} {} {}", expectedProjection.getX(), expectedProjection.getY(), expectedProjection.getZ());

      assertEquals(0.0, projectedPoint.distance(expectedProjection), EPSILON);
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(GeometryTools.class, GeometryToolsTest.class);
   }
}
