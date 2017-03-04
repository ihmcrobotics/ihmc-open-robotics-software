package us.ihmc.robotics.geometry;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
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

      EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(new Point2D(-2.5, 1.5), new Point2D(0, 0), new Point2D(-4, 4));
      FrameVector x1 = new FrameVector(point1.getReferenceFrame());
      x1.sub(point1, intersectionPoint1);
      FrameVector expectedReturn1 = x1;
      FrameVector actualReturn1 = GeometryTools.getPerpendicularVectorFromLineToPoint(point1, lineStart1, lineEnd1, intersectionPoint1);

      assertTrue("Test Failed", expectedReturn1.epsilonEquals(actualReturn1, EPSILON));
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

         Vector2D orthogonal = EuclidGeometryTools.perpendicularVector2D(lineDirection1);
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
         Vector2D orthogonal = EuclidGeometryTools.perpendicularVector2D(lineDirection);
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

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(GeometryTools.class, GeometryToolsTest.class);
   }
}
