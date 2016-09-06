package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import org.junit.Test;

import us.ihmc.tools.testing.MutationTestingTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;

@DeployableTestClass(targets = {TestPlanTarget.Fast})
public class ConvexPolygon2dCalculatorTest
{
   private static final double epsilon = 1.0e-10;

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstruction()
   {
      new ConvexPolygon2dCalculator();
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetSignedDistance1()
   {
      // single point polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2d(0.0, 0.0));
      polygon.update();

      Point2d point = new Point2d(2.5, 1.0);
      double distance = ConvexPolygon2dCalculator.getSignedDistance(point, polygon);
      assertDistanceCorrect(-Math.sqrt(2.5*2.5 + 1.0*1.0), distance);
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetSignedDistance2()
   {
      // line polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2d(0.0, 0.0));
      polygon.addVertex(new Point2d(1.0, 0.0));
      polygon.update();

      Point2d point1 = new Point2d(2.5, 1.0);
      double distance1 = ConvexPolygon2dCalculator.getSignedDistance(point1, polygon);
      assertDistanceCorrect(-Math.sqrt(1.5*1.5 + 1.0*1.0), distance1);

      Point2d point2 = new Point2d(0.5, 1.0);
      double distance2 = ConvexPolygon2dCalculator.getSignedDistance(point2, polygon);
      assertDistanceCorrect(-1.0, distance2);
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetSignedDistance3()
   {
      // triangle polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2d(0.0, 0.0));
      polygon.addVertex(new Point2d(10.0, 0.0));
      polygon.addVertex(new Point2d(0.0, 10.0));
      polygon.update();

      Point2d point1 = new Point2d(10.0, 10.0);
      double distance1 = ConvexPolygon2dCalculator.getSignedDistance(point1, polygon);
      assertDistanceCorrect(-5.0 * Math.sqrt(2.0), distance1);

      Point2d point2 = new Point2d(1.2, 1.1);
      double distance2 = ConvexPolygon2dCalculator.getSignedDistance(point2, polygon);
      assertDistanceCorrect(1.1, distance2);

      Point2d point3 = new Point2d(0.05, 9.8);
      double distance3 = ConvexPolygon2dCalculator.getSignedDistance(point3, polygon);
      assertDistanceCorrect(0.05, distance3);

      Point2d point4 = new Point2d(9.8, 0.15);
      double distance4 = ConvexPolygon2dCalculator.getSignedDistance(point4, polygon);
      assertDistanceCorrect(0.5 * Math.sqrt(0.05 * 0.05 * 2.0), distance4);

      Point2d point5 = new Point2d(5.0, -0.15);
      double distance5 = ConvexPolygon2dCalculator.getSignedDistance(point5, polygon);
      assertDistanceCorrect(-0.15, distance5);

      Point2d point6 = new Point2d(15.0, -0.15);
      double distance6 = ConvexPolygon2dCalculator.getSignedDistance(point6, polygon);
      assertDistanceCorrect(-Math.sqrt(5.0 * 5.0 + 0.15 * 0.15), distance6);
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetClosestVertexPoint1()
   {
      Point2d vertex1 = new Point2d(0.0, 0.0);
      Point2d vertex2 = new Point2d(10.0, 0.0);
      Point2d vertex3 = new Point2d(0.0, 10.0);

      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(vertex1);
      polygon.addVertex(vertex2);
      polygon.addVertex(vertex3);
      polygon.update();

      Point2d point1 = new Point2d(-1.0, -1.0);
      assertPointsEqual(vertex1, ConvexPolygon2dCalculator.getClosestVertexCopy(point1, polygon));

      Point2d point2 = new Point2d(1.0, 1.0);
      assertPointsEqual(vertex1, ConvexPolygon2dCalculator.getClosestVertexCopy(point2, polygon));

      Point2d point3 = new Point2d(10.0, 0.0);
      assertPointsEqual(vertex2, ConvexPolygon2dCalculator.getClosestVertexCopy(point3, polygon));

      Point2d point4 = new Point2d(9.8, 0.0);
      assertPointsEqual(vertex2, ConvexPolygon2dCalculator.getClosestVertexCopy(point4, polygon));

      Point2d point5 = new Point2d(10.0, 11.0);
      assertPointsEqual(vertex3, ConvexPolygon2dCalculator.getClosestVertexCopy(point5, polygon));

      Point2d point6 = new Point2d(-3.0, 8.0);
      assertPointsEqual(vertex3, ConvexPolygon2dCalculator.getClosestVertexCopy(point6, polygon));
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetClosestVertexPoint2()
   {
      // make sure the method fails as expected with an empty polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      Point2d closestVertex = new Point2d();

      assertFalse(ConvexPolygon2dCalculator.getClosestVertex(new Point2d(), polygon, closestVertex));
      assertTrue(Double.isNaN(closestVertex.x) && Double.isNaN(closestVertex.y));
      assertTrue(ConvexPolygon2dCalculator.getClosestVertexCopy(new Point2d(), polygon) == null);
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetClosestVertexLine1()
   {
      Point2d vertex1 = new Point2d(0.0, 0.0);
      Point2d vertex2 = new Point2d(10.0, 0.0);
      Point2d vertex3 = new Point2d(0.0, 10.0);

      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(vertex1);
      polygon.addVertex(vertex2);
      polygon.addVertex(vertex3);
      polygon.update();

      Line2d line1 = new Line2d(new Point2d(-1.0, 1.0), new Point2d(1.0, -1.0));
      assertPointsEqual(vertex1, ConvexPolygon2dCalculator.getClosestVertexCopy(line1, polygon));

      Line2d line2 = new Line2d(new Point2d(9.0, 0.0), new Point2d(0.0, 1.0));
      assertPointsEqual(vertex2, ConvexPolygon2dCalculator.getClosestVertexCopy(line2, polygon));

      Line2d line3 = new Line2d(new Point2d(11.0, 0.0), new Point2d(0.0, 12.0));
      assertPointsEqual(vertex2, ConvexPolygon2dCalculator.getClosestVertexCopy(line3, polygon));

      Line2d line4 = new Line2d(new Point2d(12.0, 0.0), new Point2d(0.0, 11.0));
      assertPointsEqual(vertex3, ConvexPolygon2dCalculator.getClosestVertexCopy(line4, polygon));
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetClosestVertexLine2()
   {
      // make sure the method fails as expected with an empty polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      Point2d closestVertex = new Point2d();

      assertFalse(ConvexPolygon2dCalculator.getClosestVertex(new Line2d(), polygon, closestVertex));
      assertTrue(Double.isNaN(closestVertex.x) && Double.isNaN(closestVertex.y));
      assertTrue(ConvexPolygon2dCalculator.getClosestVertexCopy(new Line2d(), polygon) == null);
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsPointInside1()
   {
      // single point polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2d(1.0, 1.0));
      polygon.update();

      Point2d point1 = new Point2d(1.0, 1.0);
      assertTrue(ConvexPolygon2dCalculator.isPointInside(point1, epsilon, polygon));

      Point2d point2 = new Point2d(0.8, 0.9);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point2, polygon));

      Point2d point3 = new Point2d(0.8, 1.1);
      assertTrue(ConvexPolygon2dCalculator.isPointInside(point3, 0.3, polygon));

      Point2d point4 = new Point2d(1.0, 0.9);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point4, polygon));

      Point2d point5 = new Point2d(2.0, 1.0);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point5, polygon));
      assertTrue(ConvexPolygon2dCalculator.isPointInside(point5, 1.0, polygon));

      Point2d point6 = new Point2d(1.0, 2.0);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point6, polygon));
      assertTrue(ConvexPolygon2dCalculator.isPointInside(point6, 1.0, polygon));
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsPointInside2()
   {
      // line polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2d(0.0, 0.0));
      polygon.addVertex(new Point2d(1.0, 0.0));
      polygon.update();

      Point2d point1 = new Point2d(0.1, 0.0);
      assertTrue(ConvexPolygon2dCalculator.isPointInside(point1, epsilon, polygon));

      Point2d point2 = new Point2d(0.1, 0.1);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point2, epsilon, polygon));

      Point2d point3 = new Point2d(1.5, 0.0);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point3, epsilon, polygon));

      Point2d point4 = new Point2d(1.0, 0.0);
      assertTrue(ConvexPolygon2dCalculator.isPointInside(point4.x, point4.y, polygon));

      Point2d point5 = new Point2d(1.0, epsilon * 0.1);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point5.x, point5.y, polygon));

      Point2d point6 = new Point2d(1.0, epsilon * 0.1);
      assertTrue(ConvexPolygon2dCalculator.isPointInside(point6, epsilon, polygon));

      Point2d point7 = new Point2d(1.5, 0.0);
      assertTrue(ConvexPolygon2dCalculator.isPointInside(point7, 0.5, polygon));
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsPointInside3()
   {
      // triangle polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2d(0.0, 0.0));
      polygon.addVertex(new Point2d(5.0, 0.0));
      polygon.addVertex(new Point2d(3.0, 5.0));
      polygon.update();

      Point2d point1 = new Point2d(0.3, 0.0);
      assertTrue(ConvexPolygon2dCalculator.isPointInside(point1, epsilon, polygon));

      Point2d point2 = new Point2d(0.0, 0.0);
      assertTrue(ConvexPolygon2dCalculator.isPointInside(point2, epsilon, polygon));

      Point2d point3 = new Point2d(2.0, 2.0);
      assertTrue(ConvexPolygon2dCalculator.isPointInside(point3, polygon));

      Point2d point4 = new Point2d(1.0, 0.3);
      assertTrue(ConvexPolygon2dCalculator.isPointInside(point4, epsilon, polygon));

      Point2d point5 = new Point2d(-1.0, 4.0);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point5.x, point5.y, epsilon, polygon));

      Point2d point6 = new Point2d(6.0, 7.0);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point6, epsilon, polygon));

      Point2d point7 = new Point2d(10.0, 0.0);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point7, epsilon, polygon));

      Point2d point8 = new Point2d(0.1, 0.2);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point8, polygon));

      Point2d point9 = new Point2d(3.5, 4.9);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point9.x, point9.y, epsilon, polygon));

      Point2d point10 = new Point2d(3.5, -1.0);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point10, polygon));
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsPointInside4()
   {
      // empty polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();

      Point2d point1 = new Point2d(10.0, 0.0);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point1, epsilon, polygon));
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsPolygonInside1()
   {
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2d(0.0, 0.0));
      polygon.addVertex(new Point2d(2.0, 1.0));
      polygon.addVertex(new Point2d(1.0, 2.0));
      polygon.update();

      ConvexPolygon2d polygonToTest1 = new ConvexPolygon2d();
      polygonToTest1.addVertex(new Point2d(0.1, 0.1));
      polygonToTest1.addVertex(new Point2d(0.2, 0.2));
      polygonToTest1.update();
      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(polygonToTest1, polygon));

      ConvexPolygon2d polygonToTest2 = new ConvexPolygon2d(polygon);
      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(polygonToTest2, polygon));
      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(polygonToTest2, epsilon, polygon));
      assertFalse(ConvexPolygon2dCalculator.isPolygonInside(polygonToTest2, -epsilon, polygon));

      ConvexPolygon2d polygonToTest3 = new ConvexPolygon2d();
      polygonToTest3.addVertex(new Point2d(0.3, 0.9));
      polygonToTest3.addVertex(new Point2d(0.1, 0.1));
      polygonToTest3.addVertex(new Point2d(1.0, 1.2));
      polygonToTest3.update();
      assertFalse(ConvexPolygon2dCalculator.isPolygonInside(polygonToTest3, polygon));

      ConvexPolygon2d polygonToTest4 = new ConvexPolygon2d();
      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(polygonToTest4, polygon));

      ConvexPolygon2d polygonToTest5 = new ConvexPolygon2d();
      polygonToTest5.addVertex(new Point2d(-0.1, 0.1));
      polygonToTest5.update();
      assertFalse(ConvexPolygon2dCalculator.isPolygonInside(polygonToTest5, polygon));
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTranslatePolygon1()
   {
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2d(0.0, 0.0));
      polygon.addVertex(new Point2d(10.0, 0.0));
      polygon.addVertex(new Point2d(0.0, 10.0));
      polygon.update();

      Vector2d translation1 = new Vector2d(0.0, 0.0);
      ConvexPolygon2d polygon1 = ConvexPolygon2dCalculator.translatePolygonCopy(translation1, polygon);
      assertTrue(polygon1.epsilonEquals(polygon, epsilon));

      Vector2d translation2 = new Vector2d(1.0, 0.5);
      ConvexPolygon2d polygon2 = ConvexPolygon2dCalculator.translatePolygonCopy(translation2, polygon);
      assertTrue(polygon2.getVertex(2).epsilonEquals(new Point2d(1.0, 0.5), epsilon));
      assertTrue(polygon2.getVertex(1).epsilonEquals(new Point2d(11.0, 0.5), epsilon));
      assertTrue(polygon2.getVertex(0).epsilonEquals(new Point2d(1.0, 10.5), epsilon));
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTranslatePolygon2()
   {
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2d(0.0, 0.0));
      polygon.update();

      Vector2d translation1 = new Vector2d(-0.1, 0.0);
      ConvexPolygon2dCalculator.translatePolygon(translation1, polygon);
      assertTrue(polygon.getVertex(0).epsilonEquals(translation1, epsilon));
   }

   private static void assertDistanceCorrect(double expected, double actual)
   {
      assertEquals("Distance did not equal expected.", expected, actual, epsilon);
   }

   private static void assertPointsEqual(Point2d expected, Point2d actual)
   {
      assertTrue("Point did not match expected.", expected.epsilonEquals(actual, epsilon));
   }

   public static void main(String[] args)
   {
      String targetTests = "us.ihmc.robotics.geometry.ConvexPolygon2dCalculatorTest";
      String targetClasses = "us.ihmc.robotics.geometry.ConvexPolygon2dCalculator";
      MutationTestingTools.doPITMutationTestAndOpenResult(targetTests, targetClasses);
   }
}
