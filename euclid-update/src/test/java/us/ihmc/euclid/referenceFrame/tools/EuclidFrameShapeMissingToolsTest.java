package us.ihmc.euclid.referenceFrame.tools;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;

import static org.junit.jupiter.api.Assertions.*;

public class EuclidFrameShapeMissingToolsTest
{
   private static final double EPSILON = 1e-6;

   @Test
   public void testGetDistanceBetweenPointAndPlane1()
   {
      FramePoint3D pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      FrameVector3D planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      FramePoint3D point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 3);
      double actual = EuclidFrameShapeMissingTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      double expected = 3.0;
      assertEquals(expected, actual, EPSILON, "FAILED: Distance from point to plane");

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 3, 3, -3);
      actual = EuclidFrameShapeMissingTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      expected = 3.0;
      assertEquals(expected, actual, EPSILON, "FAILED: Distance from point to plane");

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, -3);
      actual = EuclidFrameShapeMissingTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      expected = 3.0;
      assertEquals(expected, actual, EPSILON, "FAILED: Distance from point to plane");

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 3);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, -3);
      actual = EuclidFrameShapeMissingTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      expected = 6.0;
      assertEquals(expected, actual, EPSILON, "FAILED: Distance from point to plane");

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 1, 0, 0);
      point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 3, 0, 0);
      actual = EuclidFrameShapeMissingTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      expected = 3.0;
      assertEquals(expected, actual, EPSILON, "FAILED: Distance from point to plane");

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 1, 0);
      point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 3, 0);
      actual = EuclidFrameShapeMissingTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
      expected = 3.0;
      assertEquals(expected, actual, EPSILON, "FAILED: Distance from point to plane");

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1, 1, 1);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 1, 0);
      point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 3, 0);
      actual = EuclidFrameShapeMissingTools.distanceFromPointToPlane(point, pointOnPlane, planeNormal);
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
      assertTrue(EuclidFrameShapeMissingTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, lineStart, lineEnd));

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 1, 0, 0);
      lineStart = new FramePoint3D(ReferenceFrame.getWorldFrame(), -6, 3, -3);
      lineEnd = new FramePoint3D(ReferenceFrame.getWorldFrame(), 6, 3, 6);
      assertTrue(EuclidFrameShapeMissingTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, lineStart, lineEnd));

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 1, 0);
      lineStart = new FramePoint3D(ReferenceFrame.getWorldFrame(), 6, -3, -3);
      lineEnd = new FramePoint3D(ReferenceFrame.getWorldFrame(), 6, 3, 6);
      assertTrue(EuclidFrameShapeMissingTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, lineStart, lineEnd));

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      lineStart = new FramePoint3D(ReferenceFrame.getWorldFrame(), 6, -3, 3);
      lineEnd = new FramePoint3D(ReferenceFrame.getWorldFrame(), 6, 3, 6);
      assertFalse(EuclidFrameShapeMissingTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, lineStart, lineEnd));

      pointOnPlane = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      planeNormal = new FrameVector3D(pointOnPlane.getReferenceFrame(), 0, 0, 1);
      lineStart = new FramePoint3D(ReferenceFrame.getWorldFrame(), 6, -3, -3);
      lineEnd = new FramePoint3D(ReferenceFrame.getWorldFrame(), 6, 3, -1);
      assertFalse(EuclidFrameShapeMissingTools.isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, lineStart, lineEnd));
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
      FrameVector3D actualReturn0 = EuclidFrameShapeMissingTools.getPerpendicularVectorFromLineToPoint(point0, lineStart0, lineEnd0, intersectionPoint0);

      assertTrue(expectedReturn0.epsilonEquals(actualReturn0, EPSILON), "Test Failed");

      FramePoint3D point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 4, 2, 0);
      FramePoint3D lineStart = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      FramePoint3D lineEnd = new FramePoint3D(ReferenceFrame.getWorldFrame(), 10, 10, 0);
      FramePoint3D intersectionPoint = new FramePoint3D(ReferenceFrame.getWorldFrame(), 3, 3, 0);
      FrameVector3D x = new FrameVector3D(point.getReferenceFrame());
      x.sub(point, intersectionPoint);
      FrameVector3D expectedReturn = x;
      FrameVector3D actualReturn = EuclidFrameShapeMissingTools.getPerpendicularVectorFromLineToPoint(point, lineStart, lineEnd, intersectionPoint);
      assertTrue(expectedReturn.epsilonEquals(actualReturn, EPSILON), "Test Failed");

      FramePoint3D point1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), -2.5, 1.5, 0);
      FramePoint3D lineStart1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 0);
      FramePoint3D lineEnd1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), -4, 4, 0);
      FramePoint3D intersectionPoint1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), -2, 2, 0);

      EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(new Point2D(-2.5, 1.5), new Point2D(0, 0), new Point2D(-4, 4));
      FrameVector3D x1 = new FrameVector3D(point1.getReferenceFrame());
      x1.sub(point1, intersectionPoint1);
      FrameVector3D expectedReturn1 = x1;
      FrameVector3D actualReturn1 = EuclidFrameShapeMissingTools.getPerpendicularVectorFromLineToPoint(point1, lineStart1, lineEnd1, intersectionPoint1);

      assertTrue(expectedReturn1.epsilonEquals(actualReturn1, EPSILON), "Test Failed");
   }

   @Test
   public void testGetPlaneNormalGivenThreePoints()
   {
      FramePoint3D point1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 5, 0);
      FramePoint3D point2 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 5, 0);
      FramePoint3D point3 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 5, 0);
      FrameVector3D expectedReturn = null;
      FrameVector3D actualReturn = EuclidFrameShapeMissingTools.getPlaneNormalGivenThreePoints(point1, point2, point3);
      assertEquals(expectedReturn, actualReturn, "test failed");

      FramePoint3D point91 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 3, 0);
      FramePoint3D point92 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 5, 0);
      FramePoint3D point93 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 1, 0);
      FrameVector3D expectedReturn9 = null;
      FrameVector3D actualReturn9 = EuclidFrameShapeMissingTools.getPlaneNormalGivenThreePoints(point91, point92, point93);
      assertEquals(expectedReturn9, actualReturn9, "test failed");

      FramePoint3D point81 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 9, 0, 0);
      FramePoint3D point82 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 7, 0, 0);
      FramePoint3D point83 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 4, 0, 0);
      FrameVector3D expectedReturn8 = null;
      FrameVector3D actualReturn8 = EuclidFrameShapeMissingTools.getPlaneNormalGivenThreePoints(point81, point82, point83);
      assertEquals(expectedReturn8, actualReturn8, "test failed");

      FramePoint3D point71 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 4);
      FramePoint3D point72 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 6);
      FramePoint3D point73 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 7);
      FrameVector3D expectedReturn7 = null;
      FrameVector3D actualReturn7 = EuclidFrameShapeMissingTools.getPlaneNormalGivenThreePoints(point71, point72, point73);
      assertEquals(expectedReturn7, actualReturn7, "test failed");

      FramePoint3D point11 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 5, 46);
      FramePoint3D point12 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 587, 3);
      FramePoint3D point13 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 18, 8);
      FramePoint3D p1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 5, 5);
      FramePoint3D v1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1, 5, 5);
      FrameVector3D expectedReturn1 = new FrameVector3D(p1.getReferenceFrame());
      expectedReturn1.sub(p1, v1);
      FrameVector3D actualReturn1 = EuclidFrameShapeMissingTools.getPlaneNormalGivenThreePoints(point11, point12, point13);
      assertTrue(expectedReturn1.epsilonEquals(actualReturn1, EPSILON), "Test Failed");

      FramePoint3D point21 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 65, 0, 46);
      FramePoint3D point22 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 43, 0, 3);
      FramePoint3D point23 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 13, 0, 8);
      FramePoint3D p2 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 1, 5);
      FramePoint3D v2 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 0, 5);
      FrameVector3D expectedReturn2 = new FrameVector3D(p2.getReferenceFrame());
      expectedReturn2.sub(p2, v2);
      FrameVector3D actualReturn2 = EuclidFrameShapeMissingTools.getPlaneNormalGivenThreePoints(point21, point22, point23);
      assertTrue(expectedReturn2.epsilonEquals(actualReturn2, EPSILON), "Test Failed");

      FramePoint3D point31 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 65, 56, 0);
      FramePoint3D point32 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 43, 3, 0);
      FramePoint3D point33 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 13, 87, 0);
      FramePoint3D p3 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 55, 0);
      FramePoint3D v3 = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0, 55, 1);
      FrameVector3D expectedReturn3 = new FrameVector3D(p3.getReferenceFrame());
      expectedReturn3.sub(p3, v3);
      FrameVector3D actualReturn3 = EuclidFrameShapeMissingTools.getPlaneNormalGivenThreePoints(point31, point32, point33);
      assertTrue(expectedReturn3.epsilonEquals(actualReturn3, EPSILON), "Test Failed");
   }
}
