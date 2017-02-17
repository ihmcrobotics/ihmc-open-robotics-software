package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.tools.testing.MutationTestingTools;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class ConvexPolygon2dCalculatorTest
{
   private static final double epsilon = 1.0e-10;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testConstruction()
   {
      new ConvexPolygon2dCalculator();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetSignedDistance1()
   {
      // single point polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.update();

      Point2D point = new Point2D(2.5, 1.0);
      double distance = ConvexPolygon2dCalculator.getSignedDistance(point, polygon);
      assertDistanceCorrect(Math.sqrt(2.5 * 2.5 + 1.0 * 1.0), distance);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetSignedDistance2()
   {
      // line polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.addVertex(new Point2D(1.0, 0.0));
      polygon.update();

      Point2D point1 = new Point2D(2.5, 1.0);
      double distance1 = ConvexPolygon2dCalculator.getSignedDistance(point1, polygon);
      assertDistanceCorrect(Math.sqrt(1.5 * 1.5 + 1.0 * 1.0), distance1);

      Point2D point2 = new Point2D(0.5, 1.0);
      double distance2 = ConvexPolygon2dCalculator.getSignedDistance(point2, polygon);
      assertDistanceCorrect(1.0, distance2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetSignedDistance3()
   {
      // triangle polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.addVertex(new Point2D(10.0, 0.0));
      polygon.addVertex(new Point2D(0.0, 10.0));
      polygon.update();

      Point2D point1 = new Point2D(10.0, 10.0);
      double distance1 = ConvexPolygon2dCalculator.getSignedDistance(point1, polygon);
      assertDistanceCorrect(5.0 * Math.sqrt(2.0), distance1);

      Point2D point2 = new Point2D(1.2, 1.1);
      double distance2 = ConvexPolygon2dCalculator.getSignedDistance(point2, polygon);
      assertDistanceCorrect(-1.1, distance2);

      Point2D point3 = new Point2D(0.05, 9.8);
      double distance3 = ConvexPolygon2dCalculator.getSignedDistance(point3, polygon);
      assertDistanceCorrect(-0.05, distance3);

      Point2D point4 = new Point2D(9.8, 0.15);
      double distance4 = ConvexPolygon2dCalculator.getSignedDistance(point4, polygon);
      assertDistanceCorrect(-0.5 * Math.sqrt(0.05 * 0.05 * 2.0), distance4);

      Point2D point5 = new Point2D(5.0, -0.15);
      double distance5 = ConvexPolygon2dCalculator.getSignedDistance(point5, polygon);
      assertDistanceCorrect(0.15, distance5);

      Point2D point6 = new Point2D(15.0, -0.15);
      double distance6 = ConvexPolygon2dCalculator.getSignedDistance(point6, polygon);
      assertDistanceCorrect(Math.sqrt(5.0 * 5.0 + 0.15 * 0.15), distance6);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetClosestVertexPoint1()
   {
      Point2D vertex1 = new Point2D(0.0, 0.0);
      Point2D vertex2 = new Point2D(10.0, 0.0);
      Point2D vertex3 = new Point2D(0.0, 10.0);

      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(vertex1);
      polygon.addVertex(vertex2);
      polygon.addVertex(vertex3);
      polygon.update();

      Point2D point1 = new Point2D(-1.0, -1.0);
      assertPointsEqual(vertex1, ConvexPolygon2dCalculator.getClosestVertexCopy(point1, polygon));

      Point2D point2 = new Point2D(1.0, 1.0);
      assertPointsEqual(vertex1, ConvexPolygon2dCalculator.getClosestVertexCopy(point2, polygon));

      Point2D point3 = new Point2D(10.0, 0.0);
      assertPointsEqual(vertex2, ConvexPolygon2dCalculator.getClosestVertexCopy(point3, polygon));

      Point2D point4 = new Point2D(9.8, 0.0);
      assertPointsEqual(vertex2, ConvexPolygon2dCalculator.getClosestVertexCopy(point4, polygon));

      Point2D point5 = new Point2D(10.0, 11.0);
      assertPointsEqual(vertex3, ConvexPolygon2dCalculator.getClosestVertexCopy(point5, polygon));

      Point2D point6 = new Point2D(-3.0, 8.0);
      assertPointsEqual(vertex3, ConvexPolygon2dCalculator.getClosestVertexCopy(point6, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetClosestVertexPoint2()
   {
      // make sure the method fails as expected with an empty polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      Point2D closestVertex = new Point2D();

      assertFalse(ConvexPolygon2dCalculator.getClosestVertex(new Point2D(), polygon, closestVertex));
      assertTrue(ConvexPolygon2dCalculator.getClosestVertexCopy(new Point2D(), polygon) == null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetClosestVertexLine1()
   {
      Point2D vertex1 = new Point2D(0.0, 0.0);
      Point2D vertex2 = new Point2D(10.0, 0.0);
      Point2D vertex3 = new Point2D(0.0, 10.0);

      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(vertex1);
      polygon.addVertex(vertex2);
      polygon.addVertex(vertex3);
      polygon.update();

      Line2d line1 = new Line2d(new Point2D(-1.0, 1.0), new Point2D(1.0, -1.0));
      assertPointsEqual(vertex1, ConvexPolygon2dCalculator.getClosestVertexCopy(line1, polygon));

      Line2d line2 = new Line2d(new Point2D(9.0, 0.0), new Point2D(0.0, 1.0));
      assertPointsEqual(vertex2, ConvexPolygon2dCalculator.getClosestVertexCopy(line2, polygon));

      Line2d line3 = new Line2d(new Point2D(11.0, 0.0), new Point2D(0.0, 12.0));
      assertPointsEqual(vertex2, ConvexPolygon2dCalculator.getClosestVertexCopy(line3, polygon));

      Line2d line4 = new Line2d(new Point2D(12.0, 0.0), new Point2D(0.0, 11.0));
      assertPointsEqual(vertex3, ConvexPolygon2dCalculator.getClosestVertexCopy(line4, polygon));

      Line2d line5 = new Line2d(new Point2D(-1.0, 13.0), new Point2D(1.0, 14.0));
      assertPointsEqual(vertex3, ConvexPolygon2dCalculator.getClosestVertexCopy(line5, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetClosestVertexLine2()
   {
      // make sure the method fails as expected with an empty polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      Point2D closestVertex = new Point2D();

      assertFalse(ConvexPolygon2dCalculator.getClosestVertex(new Line2d(), polygon, closestVertex));
      assertTrue(ConvexPolygon2dCalculator.getClosestVertexCopy(new Line2d(), polygon) == null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIsPointInside1()
   {
      // single point polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Point2D point1 = new Point2D(1.0, 1.0);
      assertTrue(ConvexPolygon2dCalculator.isPointInside(point1, epsilon, polygon));

      Point2D point2 = new Point2D(0.8, 0.9);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point2, polygon));

      Point2D point3 = new Point2D(0.8, 1.1);
      assertTrue(ConvexPolygon2dCalculator.isPointInside(point3, 0.3, polygon));

      Point2D point4 = new Point2D(1.0, 0.9);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point4, polygon));

      Point2D point5 = new Point2D(2.0, 1.0);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point5, polygon));
      assertTrue(ConvexPolygon2dCalculator.isPointInside(point5, 1.0, polygon));

      Point2D point6 = new Point2D(1.0, 2.0);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point6, polygon));
      assertTrue(ConvexPolygon2dCalculator.isPointInside(point6, 1.0, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIsPointInside2()
   {
      // line polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.addVertex(new Point2D(1.0, 0.0));
      polygon.update();

      Point2D point1 = new Point2D(0.1, 0.0);
      assertTrue(ConvexPolygon2dCalculator.isPointInside(point1, epsilon, polygon));

      Point2D point2 = new Point2D(0.1, 0.1);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point2, epsilon, polygon));

      Point2D point3 = new Point2D(1.5, 0.0);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point3, epsilon, polygon));

      Point2D point4 = new Point2D(1.0, 0.0);
      assertTrue(ConvexPolygon2dCalculator.isPointInside(point4.getX(), point4.getY(), polygon));

      Point2D point5 = new Point2D(1.0, epsilon * 0.1);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point5.getX(), point5.getY(), polygon));

      Point2D point6 = new Point2D(1.0, epsilon * 0.1);
      assertTrue(ConvexPolygon2dCalculator.isPointInside(point6, epsilon, polygon));

      Point2D point7 = new Point2D(1.5, 0.0);
      assertTrue(ConvexPolygon2dCalculator.isPointInside(point7, 0.5, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIsPointInside3()
   {
      // triangle polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.addVertex(new Point2D(5.0, 0.0));
      polygon.addVertex(new Point2D(3.0, 5.0));
      polygon.update();

      Point2D point1 = new Point2D(0.3, 0.0);
      assertTrue(ConvexPolygon2dCalculator.isPointInside(point1, epsilon, polygon));

      Point2D point2 = new Point2D(0.0, 0.0);
      assertTrue(ConvexPolygon2dCalculator.isPointInside(point2, epsilon, polygon));

      Point2D point3 = new Point2D(2.0, 2.0);
      assertTrue(ConvexPolygon2dCalculator.isPointInside(point3, polygon));

      Point2D point4 = new Point2D(1.0, 0.3);
      assertTrue(ConvexPolygon2dCalculator.isPointInside(point4, epsilon, polygon));

      Point2D point5 = new Point2D(-1.0, 4.0);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point5.getX(), point5.getY(), epsilon, polygon));

      Point2D point6 = new Point2D(6.0, 7.0);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point6, epsilon, polygon));

      Point2D point7 = new Point2D(10.0, 0.0);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point7, epsilon, polygon));

      Point2D point8 = new Point2D(0.1, 0.2);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point8, polygon));

      Point2D point9 = new Point2D(3.5, 4.9);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point9.getX(), point9.getY(), epsilon, polygon));

      Point2D point10 = new Point2D(3.5, -1.0);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point10, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIsPointInside4()
   {
      // empty polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();

      Point2D point1 = new Point2D(10.0, 0.0);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point1, epsilon, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIsPointInside5()
   {
      // foot polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(-0.06, -0.08));
      polygon.addVertex(new Point2D(0.14, -0.08));
      polygon.addVertex(new Point2D(0.14, -0.19));
      polygon.addVertex(new Point2D(-0.06, -0.19));
      polygon.update();

      Point2D point1 = new Point2D(0.03, 0.0);
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point1, 0.02, polygon));

      Point2D point2 = new Point2D(0.03, -0.09);
      assertTrue(ConvexPolygon2dCalculator.isPointInside(point2, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIsPointInBoundingBox1()
   {
      // single point polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Point2D point1 = new Point2D(1.0, 1.0);
      assertTrue(ConvexPolygon2dCalculator.isPointInBoundingBox(point1, epsilon, polygon));

      Point2D point2 = new Point2D(0.8, 0.9);
      assertFalse(ConvexPolygon2dCalculator.isPointInBoundingBox(point2, polygon));

      Point2D point3 = new Point2D(0.8, 1.1);
      assertTrue(ConvexPolygon2dCalculator.isPointInBoundingBox(point3, 0.3, polygon));

      Point2D point4 = new Point2D(1.0, 0.9);
      assertFalse(ConvexPolygon2dCalculator.isPointInBoundingBox(point4, polygon));

      Point2D point5 = new Point2D(2.0, 1.0);
      assertFalse(ConvexPolygon2dCalculator.isPointInBoundingBox(point5, polygon));
      assertTrue(ConvexPolygon2dCalculator.isPointInBoundingBox(point5, 1.0, polygon));

      Point2D point6 = new Point2D(1.0, 2.0);
      assertFalse(ConvexPolygon2dCalculator.isPointInBoundingBox(point6, polygon));
      assertTrue(ConvexPolygon2dCalculator.isPointInBoundingBox(point6, 1.0, polygon));

      Point2D point7 = new Point2D(1.0 + epsilon, 1.0);
      assertTrue(ConvexPolygon2dCalculator.isPointInBoundingBox(point7, epsilon, polygon));

      Point2D point8 = new Point2D(1.0 - epsilon, 1.0);
      assertFalse(ConvexPolygon2dCalculator.isPointInBoundingBox(point8, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIsPointInBoundingBox2()
   {
      // line polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Point2D point1 = new Point2D(1.0, 0.0);
      assertTrue(ConvexPolygon2dCalculator.isPointInBoundingBox(point1, epsilon, polygon));

      Point2D point2 = new Point2D(0.0, 1.0);
      assertTrue(ConvexPolygon2dCalculator.isPointInBoundingBox(point2, epsilon, polygon));

      Point2D point3 = new Point2D(0.0, 0.0);
      assertFalse(ConvexPolygon2dCalculator.isPointInBoundingBox(point3, -epsilon, polygon));

      Point2D point4 = new Point2D(0.5, 0.5);
      assertTrue(ConvexPolygon2dCalculator.isPointInBoundingBox(point4.getX(), point4.getY(), polygon));

      Point2D point5 = new Point2D(1.0, -epsilon * 0.1);
      assertFalse(ConvexPolygon2dCalculator.isPointInBoundingBox(point5.getX(), point5.getY(), polygon));

      Point2D point6 = new Point2D(0.0, -epsilon * 0.1);
      assertTrue(ConvexPolygon2dCalculator.isPointInBoundingBox(point6, epsilon, polygon));

      Point2D point7 = new Point2D(0.7, -epsilon * 2.0);
      assertTrue(ConvexPolygon2dCalculator.isPointInBoundingBox(point7, 2.0 * epsilon, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIsPointInBoundingBox3()
   {
      // triangle polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.addVertex(new Point2D(5.0, 0.0));
      polygon.addVertex(new Point2D(3.0, 5.0));
      polygon.update();

      Point2D point1 = new Point2D(0.3, 0.0);
      assertTrue(ConvexPolygon2dCalculator.isPointInBoundingBox(point1, epsilon, polygon));

      Point2D point2 = new Point2D(0.0, 0.0);
      assertTrue(ConvexPolygon2dCalculator.isPointInBoundingBox(point2, epsilon, polygon));

      Point2D point3 = new Point2D(2.0, 2.0);
      assertTrue(ConvexPolygon2dCalculator.isPointInBoundingBox(point3, polygon));

      Point2D point4 = new Point2D(1.0, 0.3);
      assertTrue(ConvexPolygon2dCalculator.isPointInBoundingBox(point4, epsilon, polygon));

      Point2D point5 = new Point2D(-1.0, 4.0);
      assertFalse(ConvexPolygon2dCalculator.isPointInBoundingBox(point5.getX(), point5.getY(), epsilon, polygon));

      Point2D point6 = new Point2D(6.0, 7.0);
      assertFalse(ConvexPolygon2dCalculator.isPointInBoundingBox(point6, epsilon, polygon));

      Point2D point7 = new Point2D(10.0, 0.0);
      assertFalse(ConvexPolygon2dCalculator.isPointInBoundingBox(point7, epsilon, polygon));

      Point2D point8 = new Point2D(0.1, 0.2);
      assertTrue(ConvexPolygon2dCalculator.isPointInBoundingBox(point8, polygon));

      Point2D point9 = new Point2D(3.5, 4.9);
      assertTrue(ConvexPolygon2dCalculator.isPointInBoundingBox(point9.getX(), point9.getY(), epsilon, polygon));

      Point2D point10 = new Point2D(3.5, -1.0);
      assertFalse(ConvexPolygon2dCalculator.isPointInBoundingBox(point10, polygon));

      Point2D point11 = new Point2D(-0.1, 1.0);
      assertFalse(ConvexPolygon2dCalculator.isPointInBoundingBox(point11, polygon));

      Point2D point12 = new Point2D(0.0, 1.0);
      assertTrue(ConvexPolygon2dCalculator.isPointInBoundingBox(point12, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIsPolygonInside1()
   {
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.addVertex(new Point2D(2.0, 1.0));
      polygon.addVertex(new Point2D(1.0, 2.0));
      polygon.update();

      ConvexPolygon2d polygonToTest1 = new ConvexPolygon2d();
      polygonToTest1.addVertex(new Point2D(0.1, 0.1));
      polygonToTest1.addVertex(new Point2D(0.2, 0.2));
      polygonToTest1.update();
      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(polygonToTest1, polygon));

      ConvexPolygon2d polygonToTest2 = new ConvexPolygon2d(polygon);
      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(polygonToTest2, polygon));
      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(polygonToTest2, epsilon, polygon));
      assertFalse(ConvexPolygon2dCalculator.isPolygonInside(polygonToTest2, -epsilon, polygon));

      ConvexPolygon2d polygonToTest3 = new ConvexPolygon2d();
      polygonToTest3.addVertex(new Point2D(0.3, 0.9));
      polygonToTest3.addVertex(new Point2D(0.1, 0.1));
      polygonToTest3.addVertex(new Point2D(1.0, 1.2));
      polygonToTest3.update();
      assertFalse(ConvexPolygon2dCalculator.isPolygonInside(polygonToTest3, polygon));

      ConvexPolygon2d polygonToTest4 = new ConvexPolygon2d();
      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(polygonToTest4, polygon));

      ConvexPolygon2d polygonToTest5 = new ConvexPolygon2d();
      polygonToTest5.addVertex(new Point2D(-0.1, 0.1));
      polygonToTest5.update();
      assertFalse(ConvexPolygon2dCalculator.isPolygonInside(polygonToTest5, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testTranslatePolygon1()
   {
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.addVertex(new Point2D(10.0, 0.0));
      polygon.addVertex(new Point2D(0.0, 10.0));
      polygon.update();

      Vector2D translation1 = new Vector2D(0.0, 0.0);
      ConvexPolygon2d polygon1 = ConvexPolygon2dCalculator.translatePolygonCopy(translation1, polygon);
      assertTrue(polygon1.epsilonEquals(polygon, epsilon));

      Vector2D translation2 = new Vector2D(1.0, 0.5);
      ConvexPolygon2d polygon2 = ConvexPolygon2dCalculator.translatePolygonCopy(translation2, polygon);
      assertTrue(polygon2.getVertex(2).epsilonEquals(new Point2D(1.0, 0.5), epsilon));
      assertTrue(polygon2.getVertex(1).epsilonEquals(new Point2D(11.0, 0.5), epsilon));
      assertTrue(polygon2.getVertex(0).epsilonEquals(new Point2D(1.0, 10.5), epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testTranslatePolygon2()
   {
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.update();

      Vector2D translation1 = new Vector2D(-0.1, 0.0);
      ConvexPolygon2dCalculator.translatePolygon(translation1, polygon);
      assertTrue(polygon.getVertex(0).epsilonEquals(translation1, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testCanObserverSeeEdge1()
   {
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.addVertex(new Point2D(1.0, 0.0));
      polygon.addVertex(new Point2D(0.0, 1.0));
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      // observer inside polygon can not see any outside edges
      Point2D observer1 = new Point2D(0.5, 0.5);
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
         assertFalse(ConvexPolygon2dCalculator.canObserverSeeEdge(i, observer1, polygon));

      // this observer should be able to see the edge starting at vertex (0.0, 0.0)
      Point2D observer2 = new Point2D(-0.5, 0.5);
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         if (polygon.getVertex(i).epsilonEquals(new Point2D(0.0, 0.0), epsilon))
            assertTrue(ConvexPolygon2dCalculator.canObserverSeeEdge(i, observer2, polygon));
         else
            assertFalse(ConvexPolygon2dCalculator.canObserverSeeEdge(i, observer2, polygon));
      }

      // this observer should be able to see the edges starting at vertex (0.0, 1.0) and at (1.0, 1.0)
      Point2D observer3 = new Point2D(1.5, 1.5);
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         if (polygon.getVertex(i).epsilonEquals(new Point2D(0.0, 1.0), epsilon))
            assertTrue(ConvexPolygon2dCalculator.canObserverSeeEdge(i, observer3, polygon));
         else if (polygon.getVertex(i).epsilonEquals(new Point2D(1.0, 1.0), epsilon))
            assertTrue(ConvexPolygon2dCalculator.canObserverSeeEdge(i, observer3, polygon));
         else
            assertFalse(ConvexPolygon2dCalculator.canObserverSeeEdge(i, observer3, polygon));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testCanObserverSeeEdge2()
   {
      // line polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(1.0, 0.0));
      polygon.addVertex(new Point2D(0.0, 1.0));
      polygon.update();

      // should be able to see one edge
      Point2D observer1 = new Point2D(0.0, 0.0);
      boolean seeEdge1 = ConvexPolygon2dCalculator.canObserverSeeEdge(0, observer1, polygon);
      boolean seeEdge2 = ConvexPolygon2dCalculator.canObserverSeeEdge(1, observer1, polygon);
      assertTrue((seeEdge1 || seeEdge2) && !(seeEdge1 && seeEdge2));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testCanObserverSeeEdge3()
   {
      // point polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Point2D observer1 = new Point2D(0.0, 0.0);
      assertFalse(ConvexPolygon2dCalculator.canObserverSeeEdge(0, observer1, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetVertexOnSide1()
   {
      // add vertices in clockwise order so updating the polygon does not change indices
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(0.0, 1.0));
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.addVertex(new Point2D(1.0, 0.0));
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.update();

      Point2D observer1 = new Point2D(0.5, -0.5);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(0, 0, observer1, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(0, 1, observer1, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(0, 2, observer1, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(0, 3, observer1, polygon), 3);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(1, 1, observer1, polygon), 1);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(1, 2, observer1, polygon), 1);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(1, 3, observer1, polygon), 3);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(2, 2, observer1, polygon), 2);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(2, 3, observer1, polygon), 3);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(3, 3, observer1, polygon), 3);

      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(0, 0, observer1, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(1, 0, observer1, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(2, 0, observer1, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(3, 0, observer1, polygon), 3);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(1, 1, observer1, polygon), 1);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(2, 1, observer1, polygon), 1);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(3, 1, observer1, polygon), 3);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(2, 2, observer1, polygon), 2);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(3, 2, observer1, polygon), 3);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(3, 3, observer1, polygon), 3);

      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnRight(0, 0, observer1, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnRight(0, 1, observer1, polygon), 1);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnRight(0, 2, observer1, polygon), 2);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnRight(0, 3, observer1, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnRight(1, 1, observer1, polygon), 1);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnRight(1, 2, observer1, polygon), 2);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnRight(1, 3, observer1, polygon), 1);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnRight(2, 2, observer1, polygon), 2);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnRight(2, 3, observer1, polygon), 2);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnRight(3, 3, observer1, polygon), 3);

      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnRight(0, 0, observer1, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnRight(1, 0, observer1, polygon), 1);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnRight(2, 0, observer1, polygon), 2);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnRight(3, 0, observer1, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnRight(1, 1, observer1, polygon), 1);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnRight(2, 1, observer1, polygon), 2);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnRight(3, 1, observer1, polygon), 1);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnRight(2, 2, observer1, polygon), 2);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnRight(3, 2, observer1, polygon), 2);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnRight(3, 3, observer1, polygon), 3);

      Point2D observer2 = new Point2D(0.5, 0.5);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(0, 0, observer2, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(0, 1, observer2, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(0, 2, observer2, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(0, 3, observer2, polygon), 3);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetVertexOnSide2()
   {
      // add vertices in clockwise order so updating the polygon does not change indices
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(0.0, 1.0));
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Point2D observer1 = new Point2D(0.0, 0.0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(0, 1, observer1, polygon), 0);

      Point2D observer2 = new Point2D(0.0, 2.0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(0, 1, observer2, polygon), 1);

      Point2D observer3 = new Point2D(10.0, 0.0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(0, 1, observer3, polygon), 0);

      Point2D observer4 = new Point2D(2.0, 2.0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(0, 1, observer4, polygon), 1);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetMiddleIndexCounterClockwise1()
   {
      // do not update polygon to keep number of vertices
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      for (int i = 0; i < 6; i++)
         polygon.addVertex(new Point2D());
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(0, 0, polygon), 3);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(1, 1, polygon), 4);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(2, 2, polygon), 5);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(3, 3, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(4, 4, polygon), 1);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(5, 5, polygon), 2);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(1, 0, polygon), 1);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(2, 0, polygon), 1);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(3, 0, polygon), 2);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(4, 0, polygon), 2);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(5, 0, polygon), 3);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(2, 1, polygon), 2);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(3, 1, polygon), 2);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(4, 1, polygon), 3);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(5, 1, polygon), 3);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(3, 2, polygon), 3);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(4, 2, polygon), 3);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(5, 2, polygon), 4);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(4, 3, polygon), 4);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(5, 3, polygon), 4);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(5, 4, polygon), 5);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(0, 1, polygon), 4);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(0, 2, polygon), 4);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(0, 3, polygon), 5);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(0, 4, polygon), 5);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(0, 5, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(1, 2, polygon), 5);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(1, 3, polygon), 5);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(1, 4, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(1, 5, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(2, 3, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(2, 4, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(2, 5, polygon), 1);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(3, 4, polygon), 1);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(3, 5, polygon), 1);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(4, 5, polygon), 2);

      polygon.clear();
      for (int i = 0; i < 3; i++)
         polygon.addVertex(new Point2D());
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(0, 0, polygon), 2);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(1, 1, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(2, 2, polygon), 1);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(0, 1, polygon), 2);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(0, 2, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(1, 2, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(1, 0, polygon), 1);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(2, 0, polygon), 1);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(2, 1, polygon), 2);

      polygon.clear();
      polygon.addVertex(new Point2D());
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(0, 0, polygon), 0);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetLineOfSightVertices1()
   {
      Point2D vertex1 = new Point2D(0.0, 1.0);
      Point2D vertex2 = new Point2D(1.0, 1.0);
      Point2D vertex3 = new Point2D(1.5, 0.5);
      Point2D vertex4 = new Point2D(1.0, 0.0);
      Point2D vertex5 = new Point2D(0.0, 0.0);

      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(vertex1);
      polygon.addVertex(vertex2);
      polygon.addVertex(vertex3);
      polygon.addVertex(vertex4);
      polygon.addVertex(vertex5);
      polygon.update();

      Point2D observer1 = new Point2D(-0.5, 0.5);
      assertPointsEqual(new Point2D[] {vertex1, vertex5}, ConvexPolygon2dCalculator.getLineOfSightVerticesCopy(observer1, polygon), true);

      Point2D observer2 = new Point2D(1.0, -0.5);
      assertPointsEqual(new Point2D[] {vertex5, vertex3}, ConvexPolygon2dCalculator.getLineOfSightVerticesCopy(observer2, polygon), true);

      Point2D observer3 = new Point2D(-1.0, -2.0 + epsilon);
      assertPointsEqual(new Point2D[] {vertex1, vertex4}, ConvexPolygon2dCalculator.getLineOfSightVerticesCopy(observer3, polygon), true);

      Point2D observer4 = new Point2D(-1.0, -2.0 - epsilon);
      assertPointsEqual(new Point2D[] {vertex1, vertex3}, ConvexPolygon2dCalculator.getLineOfSightVerticesCopy(observer4, polygon), true);

      Point2D observer5 = new Point2D(1.5 + epsilon, 0.5);
      assertPointsEqual(new Point2D[] {vertex4, vertex2}, ConvexPolygon2dCalculator.getLineOfSightVerticesCopy(observer5, polygon), true);

      Point2D observer6 = vertex3;
      assertPointsEqual(null, ConvexPolygon2dCalculator.getLineOfSightVerticesCopy(observer6, polygon), true);

      Point2D observer7 = new Point2D(0.5, 0.5);
      assertPointsEqual(null, ConvexPolygon2dCalculator.getLineOfSightVerticesCopy(observer7, polygon), true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetLineOfSightVertices2()
   {
      // empty polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();

      Point2D observer1 = new Point2D(0.5, 0.5);
      assertIndicesCorrect(null, ConvexPolygon2dCalculator.getLineOfSightVertexIndicesCopy(observer1, polygon));
      assertPointsEqual(null, ConvexPolygon2dCalculator.getLineOfSightVerticesCopy(observer1, polygon), true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetLineOfSightVertexIndices1()
   {
      Point2D vertex = new Point2D(-0.5, 0.5);

      // point polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(vertex);
      polygon.update();

      Point2D observer1 = vertex;
      assertIndicesCorrect(null, ConvexPolygon2dCalculator.getLineOfSightVertexIndicesCopy(observer1, polygon));

      Point2D observer2 = new Point2D(0.5, 0.5);
      assertIndicesCorrect(new int[] {0, 0}, ConvexPolygon2dCalculator.getLineOfSightVertexIndicesCopy(observer2, polygon));
      assertPointsEqual(new Point2D[] {vertex}, ConvexPolygon2dCalculator.getLineOfSightVerticesCopy(observer2, polygon), true);

      int[] result = new int[] {-1, 7};
      ConvexPolygon2dCalculator.getLineOfSightVertexIndices(observer2, result, polygon);
      assertIndicesCorrect(new int[] {0, 0}, result);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetLineOfSightVertexIndices4()
   {
      // add vertices in clockwise order so updating the polygon does not change indices
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(0.0, 1.0));
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Point2D observer1 = new Point2D(-1.0, 1.0);
      assertIndicesCorrect(new int[] {0, 1}, ConvexPolygon2dCalculator.getLineOfSightVertexIndicesCopy(observer1, polygon));

      Point2D observer2 = new Point2D(0.5, 0.0);
      assertIndicesCorrect(new int[] {0, 1}, ConvexPolygon2dCalculator.getLineOfSightVertexIndicesCopy(observer2, polygon));

      Point2D observer3 = new Point2D(0.5, 1.5);
      assertIndicesCorrect(new int[] {1, 0}, ConvexPolygon2dCalculator.getLineOfSightVertexIndicesCopy(observer3, polygon));

      Point2D observer4 = new Point2D(0.5, 1.0);
      assertIndicesCorrect(null, ConvexPolygon2dCalculator.getLineOfSightVertexIndicesCopy(observer4, polygon));

      Point2D observer5 = new Point2D(1.0, 1.0);
      assertIndicesCorrect(null, ConvexPolygon2dCalculator.getLineOfSightVertexIndicesCopy(observer5, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetIntersectionLambda1()
   {
      Random random = new Random(84587278988L);
      for (int i = 0; i < 1000; i++)
      {
         Point2D point1 = new Point2D(random.nextGaussian(), random.nextGaussian());
         Vector2D direction1 = new Vector2D(random.nextGaussian(), random.nextGaussian());
         Point2D point2 = new Point2D(random.nextGaussian(), random.nextGaussian());
         Vector2D direction2 = new Vector2D(random.nextGaussian(), random.nextGaussian());

         double lambda = ConvexPolygon2dCalculator.getIntersectionLambda(point1.getX(), point1.getY(), direction1.getX(), direction1.getY(), point2.getX(), point2.getY(), direction2.getX(),
               direction2.getY());
         Point2D intersection = new Point2D(point1);
         direction1.scale(lambda);
         intersection.add(direction1);

         Line2d line1 = new Line2d(point1, direction1);
         Line2d line2 = new Line2d(point2, direction2);
         assertPointsEqual(line1.intersectionWith(line2), intersection);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetIntersectionLambda2()
   {
      Random random = new Random(8458475566478988L);
      for (int i = 0; i < 1000; i++)
      {
         Point2D point1 = new Point2D(random.nextGaussian(), random.nextGaussian());
         Vector2D direction1 = new Vector2D(random.nextGaussian(), random.nextGaussian());
         Point2D point2 = new Point2D(random.nextGaussian(), random.nextGaussian());
         Vector2D direction2 = new Vector2D(direction1);
         direction2.scale(random.nextGaussian());

         double lambda = ConvexPolygon2dCalculator.getIntersectionLambda(point1.getX(), point1.getY(), direction1.getX(), direction1.getY(), point2.getX(), point2.getY(), direction2.getX(),
               direction2.getY());

         assertTrue("Lines are parallel expected lambda to ne NaN.", Double.isNaN(lambda));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetIntersectionLambda3()
   {
      // check directions aligned with axes
      {
         Point2D point1 = new Point2D(1.0, 1.0);
         Vector2D direction1 = new Vector2D(0.5, 0.5);
         Point2D point2 = new Point2D(point1);
         Vector2D direction2 = new Vector2D(0, 1.0);
         Point2D expected = new Point2D(point1);

         double lambda = ConvexPolygon2dCalculator.getIntersectionLambda(point1.getX(), point1.getY(), direction1.getX(), direction1.getY(), point2.getX(), point2.getY(), direction2.getX(),
               direction2.getY());

         Point2D intersection = new Point2D(point1);
         direction1.scale(lambda);
         intersection.add(direction1);

         assertPointsEqual(expected, intersection);
      }

      {
         Point2D point1 = new Point2D(-1.0, -1.0);
         Vector2D direction1 = new Vector2D(0.5, 0.5);
         Point2D point2 = new Point2D(point1);
         Vector2D direction2 = new Vector2D(1.0, 0.0);
         Point2D expected = new Point2D(point1);

         double lambda = ConvexPolygon2dCalculator.getIntersectionLambda(point1.getX(), point1.getY(), direction1.getX(), direction1.getY(), point2.getX(), point2.getY(), direction2.getX(),
               direction2.getY());

         Point2D intersection = new Point2D(point1);
         direction1.scale(lambda);
         intersection.add(direction1);

         assertPointsEqual(expected, intersection);
      }

      {
         Point2D point1 = new Point2D(0.0, 1.0);
         Vector2D direction1 = new Vector2D(0.0, 2.0);
         Point2D point2 = new Point2D(0.0, 0.0);
         Vector2D direction2 = new Vector2D(0.5, 0.0);
         Point2D expected = new Point2D(0.0, 0.0);

         double lambda = ConvexPolygon2dCalculator.getIntersectionLambda(point1.getX(), point1.getY(), direction1.getX(), direction1.getY(), point2.getX(), point2.getY(), direction2.getX(),
               direction2.getY());

         Point2D intersection = new Point2D(point1);
         direction1.scale(lambda);
         intersection.add(direction1);

         assertPointsEqual(expected, intersection);
      }

      {
         Point2D point1 = new Point2D(1.0, 0.0);
         Vector2D direction1 = new Vector2D(2.0, 0.0);
         Point2D point2 = new Point2D(0.0, 0.0);
         Vector2D direction2 = new Vector2D(0.0, 0.5);
         Point2D expected = new Point2D(0.0, 0.0);

         double lambda = ConvexPolygon2dCalculator.getIntersectionLambda(point1.getX(), point1.getY(), direction1.getX(), direction1.getY(), point2.getX(), point2.getY(), direction2.getX(),
               direction2.getY());

         Point2D intersection = new Point2D(point1);
         direction1.scale(lambda);
         intersection.add(direction1);

         assertPointsEqual(expected, intersection);
      }

      {
         Point2D point1 = new Point2D(0.0, 0.0);
         Vector2D direction1 = new Vector2D(0.0, 1.0);
         Point2D point2 = new Point2D(point1);
         Vector2D direction2 = new Vector2D(0.0, 1.0);

         double lambda = ConvexPolygon2dCalculator.getIntersectionLambda(point1.getX(), point1.getY(), direction1.getX(), direction1.getY(), point2.getX(), point2.getY(), direction2.getX(),
               direction2.getY());
         assertTrue("Lines are parallel expected lambda to ne NaN.", Double.isNaN(lambda));
      }

      {
         Point2D point1 = new Point2D(1.0, 0.0);
         Vector2D direction1 = new Vector2D(0.0, 1.0);
         Point2D point2 = new Point2D(2.0, 0.0);
         Vector2D direction2 = new Vector2D(0.0, 1.0);

         double lambda = ConvexPolygon2dCalculator.getIntersectionLambda(point1.getX(), point1.getY(), direction1.getX(), direction1.getY(), point2.getX(), point2.getY(), direction2.getX(),
               direction2.getY());
         assertTrue("Lines are parallel expected lambda to ne NaN.", Double.isNaN(lambda));
      }

      {
         Point2D point1 = new Point2D(1.0, 2.0);
         Vector2D direction1 = new Vector2D(1.0, 0.0);
         Point2D point2 = new Point2D(1.0, 1.0);
         Vector2D direction2 = new Vector2D(1.0, 0.0);

         double lambda = ConvexPolygon2dCalculator.getIntersectionLambda(point1.getX(), point1.getY(), direction1.getX(), direction1.getY(), point2.getX(), point2.getY(), direction2.getX(),
               direction2.getY());
         assertTrue("Lines are parallel expected lambda to ne NaN.", Double.isNaN(lambda));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testDoesLineIntersectEdge1()
   {
      // add in order so update does not change indices:
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(0.0, 1.0));
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.addVertex(new Point2D(1.5, 0.5));
      polygon.addVertex(new Point2D(1.0, 0.0));
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.update();

      Line2d line1 = new Line2d(new Point2D(0.0, 0.1), new Vector2D(1.0, 1.0));
      assertTrue(ConvexPolygon2dCalculator.doesLineIntersectEdge(line1, 0, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line1, 1, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line1, 2, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line1, 3, polygon));
      assertTrue(ConvexPolygon2dCalculator.doesLineIntersectEdge(line1, 4, polygon));

      Line2d line2 = new Line2d(new Point2D(0.9, 1.0), new Vector2D(1.0, -1.0));
      assertTrue(ConvexPolygon2dCalculator.doesLineIntersectEdge(line2, 0, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line2, 1, polygon));
      assertTrue(ConvexPolygon2dCalculator.doesLineIntersectEdge(line2, 2, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line2, 3, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line2, 4, polygon));

      Line2d line3 = new Line2d(new Point2D(0.2, 0.6), new Vector2D(1.0, 0.0));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line3, 0, polygon));
      assertTrue(ConvexPolygon2dCalculator.doesLineIntersectEdge(line3, 1, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line3, 2, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line3, 3, polygon));
      assertTrue(ConvexPolygon2dCalculator.doesLineIntersectEdge(line3, 4, polygon));

      Line2d line4 = new Line2d(new Point2D(0.0, -0.3), new Vector2D(0.0, 0.25));
      assertTrue(ConvexPolygon2dCalculator.doesLineIntersectEdge(line4, 0, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line4, 1, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line4, 2, polygon));
      assertTrue(ConvexPolygon2dCalculator.doesLineIntersectEdge(line4, 3, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line4, 4, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testDoesLineIntersectEdge2()
   {
      // add in order so update does not change indices:
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(0.0, 1.0));
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Line2d line1 = new Line2d(new Point2D(0.0, 0.3), new Vector2D(1.0, 0.0));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line1, 0, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line1, 1, polygon));

      Line2d line2 = new Line2d(new Point2D(0.0, 0.3), new Vector2D(0.0, 1.0));
      assertTrue(ConvexPolygon2dCalculator.doesLineIntersectEdge(line2, 0, polygon));
      assertTrue(ConvexPolygon2dCalculator.doesLineIntersectEdge(line2, 1, polygon));

      Line2d line3 = new Line2d(new Point2D(0.0, 0.3), new Vector2D(0.0, -1.0));
      assertTrue(ConvexPolygon2dCalculator.doesLineIntersectEdge(line3, 0, polygon));
      assertTrue(ConvexPolygon2dCalculator.doesLineIntersectEdge(line3, 1, polygon));

      Line2d line4 = new Line2d(new Point2D(2.0, 0.3), new Vector2D(0.0, -1.0));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line4, 0, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line4, 1, polygon));

      Line2d line5 = new Line2d(new Point2D(-epsilon, 0.3), new Vector2D(0.0, -1.0));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line5, 0, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line5, 1, polygon));

      Line2d line6 = new Line2d(new Point2D(0.0, 0.3), new Vector2D(1.0, 0.0));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line6, 0, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line6, 1, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testDoesLineIntersectEdge3()
   {
      ConvexPolygon2d polygon = new ConvexPolygon2d();

      Line2d line5 = new Line2d(new Point2D(0.0, 0.0), new Vector2D(1.0, 0.0));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line5, 0, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetIntersectingEdges1()
   {
      Point2D vertex1 = new Point2D(0.0, 1.0);
      Point2D vertex2 = new Point2D(1.0, 1.0);
      Point2D vertex3 = new Point2D(1.0, 0.0);
      Point2D vertex4 = new Point2D(0.0, 0.0);

      LineSegment2d edge1 = new LineSegment2d(vertex1, vertex2);
      LineSegment2d edge2 = new LineSegment2d(vertex2, vertex3);
      LineSegment2d edge3 = new LineSegment2d(vertex3, vertex4);
      LineSegment2d edge4 = new LineSegment2d(vertex4, vertex1);

      // add in order so update does not change indices:
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(vertex1);
      polygon.addVertex(vertex2);
      polygon.addVertex(vertex3);
      polygon.addVertex(vertex4);
      polygon.update();

      LineSegment2d result1 = new LineSegment2d();
      LineSegment2d result2 = new LineSegment2d();

      Line2d line1 = new Line2d(new Point2D(0.5, 0.5), new Vector2D(-1.0, 0.0));
      LineSegment2d[] expected1 = new LineSegment2d[] {edge4, edge2};
      assertEdgesEqual(expected1, ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line1, polygon), false);
      assertEquals(2, ConvexPolygon2dCalculator.getIntersectingEdges(line1, result1, result2, polygon));
      assertEdgesEqual(expected1, new LineSegment2d[] {result1, result2}, false);

      Line2d line2 = new Line2d(new Point2D(0.5, 1.5), new Vector2D(1.0, 0.0));
      LineSegment2d[] expected2 = null;
      assertEdgesEqual(expected2, ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line2, polygon), false);
      assertEquals(0, ConvexPolygon2dCalculator.getIntersectingEdges(line2, result1, result2, polygon));

      Line2d line3 = new Line2d(new Point2D(0.0, 2.0), new Vector2D(1.0, -1.0));
      LineSegment2d[] expected3 = new LineSegment2d[] {edge1};
      assertEdgesEqual(expected3, ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line3, polygon), false);
      assertEquals(1, ConvexPolygon2dCalculator.getIntersectingEdges(line3, result1, result2, polygon));
      assertEdgesEqual(expected3, new LineSegment2d[] {result1}, false);

      Line2d line4 = new Line2d(new Point2D(0.0, 0.0), new Vector2D(1.0, 1.0));
      LineSegment2d[] expected4 = new LineSegment2d[] {edge1, edge3};
      assertEdgesEqual(expected4, ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line4, polygon), false);
      assertEquals(2, ConvexPolygon2dCalculator.getIntersectingEdges(line4, result1, result2, polygon));
      assertEdgesEqual(expected4, new LineSegment2d[] {result1, result2}, false);

      Line2d line5 = new Line2d(new Point2D(-0.5, -0.5), new Vector2D(0.7, 0.7));
      LineSegment2d[] expected5 = new LineSegment2d[] {edge1, edge3};
      assertEdgesEqual(expected5, ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line5, polygon), false);
      assertEquals(2, ConvexPolygon2dCalculator.getIntersectingEdges(line5, result1, result2, polygon));
      assertEdgesEqual(expected5, new LineSegment2d[] {result1, result2}, false);

      Line2d line6 = new Line2d(new Point2D(0.0, -0.5), new Vector2D(0.0, 0.7));
      LineSegment2d[] expected6 = new LineSegment2d[] {edge1, edge3};
      assertEdgesEqual(expected6, ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line6, polygon), false);
      assertEquals(2, ConvexPolygon2dCalculator.getIntersectingEdges(line6, result1, result2, polygon));
      assertEdgesEqual(expected6, new LineSegment2d[] {result1, result2}, false);

      Line2d line7 = new Line2d(new Point2D(-0.5, 1.5), new Vector2D(1.0, -1.0));
      LineSegment2d[] expected7 = new LineSegment2d[] {edge2, edge4};
      assertEdgesEqual(expected7, ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line7, polygon), false);
      assertEquals(2, ConvexPolygon2dCalculator.getIntersectingEdges(line7, result1, result2, polygon));
      assertEdgesEqual(expected7, new LineSegment2d[] {result1, result2}, false);

      Line2d line8 = new Line2d(new Point2D(1.0, 0.5), new Vector2D(0.0, -0.2));
      LineSegment2d[] expected8 = new LineSegment2d[] {edge1, edge3};
      assertEdgesEqual(expected8, ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line8, polygon), false);
      assertEquals(2, ConvexPolygon2dCalculator.getIntersectingEdges(line8, result1, result2, polygon));
      assertEdgesEqual(expected8, new LineSegment2d[] {result1, result2}, false);

      Line2d line9 = new Line2d(new Point2D(-0.3, 1.0), new Vector2D(0.2, 0.0));
      LineSegment2d[] expected9 = new LineSegment2d[] {edge4, edge2};
      assertEdgesEqual(expected9, ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line9, polygon), false);
      assertEquals(2, ConvexPolygon2dCalculator.getIntersectingEdges(line9, result1, result2, polygon));
      assertEdgesEqual(expected9, new LineSegment2d[] {result1, result2}, false);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetIntersectingEdges2()
   {
      // line polygon
      Point2D vertex1 = new Point2D(1.0, 1.0);
      Point2D vertex2 = new Point2D(1.0, 0.0);

      LineSegment2d edge1 = new LineSegment2d(vertex1, vertex2);
      LineSegment2d edge2 = new LineSegment2d(vertex2, vertex1);

      // add in order so update does not change indices:
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(vertex1);
      polygon.addVertex(vertex2);
      polygon.update();

      LineSegment2d result1 = new LineSegment2d();
      LineSegment2d result2 = new LineSegment2d();

      Line2d line1 = new Line2d(new Point2D(0.5, 1.5), new Vector2D(0.0, 0.1));
      LineSegment2d[] expected1 = null;
      assertEdgesEqual(expected1, ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line1, polygon), false);
      assertEquals(0, ConvexPolygon2dCalculator.getIntersectingEdges(line1, result1, result2, polygon));

      Line2d line2 = new Line2d(new Point2D(-0.5, 0.0), new Vector2D(0.75, 0.25));
      LineSegment2d[] expected2 = new LineSegment2d[] {edge1, edge2};
      assertEdgesEqual(expected2, ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line2, polygon), false);
      assertEquals(2, ConvexPolygon2dCalculator.getIntersectingEdges(line2, result1, result2, polygon));
      assertEdgesEqual(expected2, new LineSegment2d[] {result1, result2}, false);

      Line2d line3 = new Line2d(new Point2D(1.0, -0.5), new Vector2D(0.0, 0.1));
      LineSegment2d[] expected3 = null;
      assertEdgesEqual(expected3, ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line3, polygon), false);
      assertEquals(0, ConvexPolygon2dCalculator.getIntersectingEdges(line3, result1, result2, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetIntersectingEdges3()
   {
      // point polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(-1.0, -0.5));
      polygon.update();

      LineSegment2d result1 = new LineSegment2d();
      LineSegment2d result2 = new LineSegment2d();

      Line2d line1 = new Line2d(new Point2D(0.0, 0.0), new Vector2D(-0.5, -0.25));
      assertEquals(ConvexPolygon2dCalculator.getIntersectingEdges(line1, result1, result2, polygon), 0);
      assertTrue(ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line1, polygon) == null);

      Line2d line2 = new Line2d(new Point2D(0.5, 1.5), new Vector2D(0.0, 0.1));
      assertEquals(ConvexPolygon2dCalculator.getIntersectingEdges(line2, result1, result2, polygon), 0);
      assertTrue(ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line2, polygon) == null);

      Line2d line3 = new Line2d(new Point2D(-1.0, -0.5), new Vector2D(1.0, 0.1));
      assertEquals(ConvexPolygon2dCalculator.getIntersectingEdges(line3, result1, result2, polygon), 0);
      assertTrue(ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line3, polygon) == null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetIntersectingEdges4()
   {
      // empty polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();

      LineSegment2d result1 = new LineSegment2d();
      LineSegment2d result2 = new LineSegment2d();

      Line2d line1 = new Line2d(new Point2D(0.5, 1.5), new Vector2D(0.0, 0.1));
      assertEquals(ConvexPolygon2dCalculator.getIntersectingEdges(line1, result1, result2, polygon), 0);
      assertTrue(ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line1, polygon) == null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIntersectionWithLine1()
   {
      // add in order so vertices do not get changed when update is called.
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.addVertex(new Point2D(-1.0, 0.0));
      polygon.addVertex(new Point2D(0.0, 1.0));
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Point2D result1 = new Point2D(0.6, 0.4);
      Point2D result2 = new Point2D(0.1, 0.9);

      Line2d line1 = new Line2d(new Point2D(0.0, 0.5), new Vector2D(0.1, 0.0));
      Point2D[] expected1 = new Point2D[] {new Point2D(-0.5, 0.5), new Point2D(0.5, 0.5)};
      assertPointsEqual(expected1, ConvexPolygon2dCalculator.intersectionWithLineCopy(line1, polygon), false);

      Line2d line2 = new Line2d(new Point2D(1.0, 0.0), new Vector2D(0.0, -8.0));
      Point2D[] expected2 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected2, ConvexPolygon2dCalculator.intersectionWithLineCopy(line2, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLine(line2, result1, result2, polygon) == 1);
      assertPointsEqual(expected2[0], result1);

      Line2d line3 = new Line2d(new Point2D(0.0, 1.0), new Vector2D(0.5, 0.0));
      Point2D[] expected3 = new Point2D[] {new Point2D(0.0, 1.0), new Point2D(1.0, 1.0)};
      assertPointsEqual(expected3, ConvexPolygon2dCalculator.intersectionWithLineCopy(line3, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLine(line3, result1, result2, polygon) == 2);
      assertPointsEqual(expected3[0], result1);
      assertPointsEqual(expected3[1], result2);

      Line2d line4 = new Line2d(new Point2D(0.5, 10.0), new Vector2D(0.0, 0.1));
      Point2D[] expected4 = new Point2D[] {new Point2D(0.5, 1.0), new Point2D(0.5, 0.5)};
      assertPointsEqual(expected4, ConvexPolygon2dCalculator.intersectionWithLineCopy(line4, polygon), false);

      Line2d line5 = new Line2d(new Point2D(-1.0, -0.5), new Vector2D(1.0, 1.0));
      Point2D[] expected5 = new Point2D[] {new Point2D(-0.5, 0.0), new Point2D(0.5, 1.0)};
      assertPointsEqual(expected5, ConvexPolygon2dCalculator.intersectionWithLineCopy(line5, polygon), false);

      Line2d line6 = new Line2d(new Point2D(0.0, -1.5), new Vector2D(1.0, 1.0));
      Point2D[] expected6 = null;
      result1.set(0.0, 0.0);
      result2.set(0.0, 0.0);
      assertPointsEqual(expected6, ConvexPolygon2dCalculator.intersectionWithLineCopy(line6, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLine(line6, result1, result2, polygon) == 0);

      Line2d line7 = new Line2d(new Point2D(0.0, -1.5), new Vector2D(0.0, 2.0));
      Point2D[] expected7 = new Point2D[] {new Point2D(0.0, 0.0), new Point2D(0.0, 1.0)};
      assertPointsEqual(expected7, ConvexPolygon2dCalculator.intersectionWithLineCopy(line7, polygon), false);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIntersectionWithLine2()
   {
      // line polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(1.0, 0.0));
      polygon.addVertex(new Point2D(-1.0, 0.0));
      polygon.update();

      Line2d line1 = new Line2d(new Point2D(-1.0, 1.0), new Vector2D(0.0, -0.8));
      Point2D[] expected1 = new Point2D[] {new Point2D(-1.0, 0.0)};
      assertPointsEqual(expected1, ConvexPolygon2dCalculator.intersectionWithLineCopy(line1, polygon), false);

      Line2d line2 = new Line2d(new Point2D(-0.5, 1.0), new Vector2D(0.0, -0.8));
      Point2D[] expected2 = new Point2D[] {new Point2D(-0.5, 0.0)};
      assertPointsEqual(expected2, ConvexPolygon2dCalculator.intersectionWithLineCopy(line2, polygon), false);

      Line2d line3 = new Line2d(new Point2D(1.5, 1.0), new Vector2D(0.0, -0.8));
      Point2D[] expected3 = null;
      assertPointsEqual(expected3, ConvexPolygon2dCalculator.intersectionWithLineCopy(line3, polygon), false);

      Line2d line4 = new Line2d(new Point2D(-0.8, 0.0), new Vector2D(0.1, 0.0));
      Point2D[] expected4 = new Point2D[] {new Point2D(-1.0, 0.0), new Point2D(1.0, 0.0)};
      assertPointsEqual(expected4, ConvexPolygon2dCalculator.intersectionWithLineCopy(line4, polygon), false);

      Line2d line5 = new Line2d(new Point2D(1.0, 0.0), new Vector2D(0.0, -0.1));
      Point2D[] expected5 = new Point2D[] {new Point2D(1.0, 0.0)};
      assertPointsEqual(expected5, ConvexPolygon2dCalculator.intersectionWithLineCopy(line5, polygon), false);

      Line2d line6 = new Line2d(new Point2D(-1.0, 0.0), new Vector2D(0.0, -0.1));
      Point2D[] expected6 = new Point2D[] {new Point2D(-1.0, 0.0)};
      assertPointsEqual(expected6, ConvexPolygon2dCalculator.intersectionWithLineCopy(line6, polygon), false);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIntersectionWithLine3()
   {
      // point polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(1.0, 0.0));
      polygon.update();

      Line2d line1 = new Line2d(new Point2D(3.0, 1.0), new Vector2D(-2.0, -1.0));
      Point2D[] expected1 = new Point2D[] {new Point2D(1.0, 0.0)};
      assertPointsEqual(expected1, ConvexPolygon2dCalculator.intersectionWithLineCopy(line1, polygon), false);

      Line2d line2 = new Line2d(new Point2D(2.0, 1.0), new Vector2D(-1.3, -0.8));
      Point2D[] expected2 = null;
      assertPointsEqual(expected2, ConvexPolygon2dCalculator.intersectionWithLineCopy(line2, polygon), false);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIntersectionWithLine4()
   {
      // empty polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();

      Line2d line1 = new Line2d(new Point2D(3.0, 1.0), new Vector2D(-1.6, -0.8));
      Point2D[] expected1 = null;
      assertPointsEqual(expected1, ConvexPolygon2dCalculator.intersectionWithLineCopy(line1, polygon), false);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIntersectionWithRay1()
   {
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(-1.0, -1.0));
      polygon.addVertex(new Point2D(1.0, -1.0));
      polygon.addVertex(new Point2D(-1.0, 1.0));
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Point2D result1 = new Point2D();
      Point2D result2 = new Point2D();

      Line2d ray1 = new Line2d(new Point2D(0.0, 0.0), new Vector2D(0.2, 0.0));
      Point2D[] expected1 = new Point2D[] {new Point2D(1.0, 0.0)};
      assertPointsEqual(expected1, ConvexPolygon2dCalculator.intersectionWithRayCopy(ray1, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithRay(ray1, result1, result2, polygon) == 1);

      Line2d ray2 = new Line2d(new Point2D(-1.0, 0.0), new Vector2D(0.2, 0.0));
      Point2D[] expected2 = new Point2D[] {new Point2D(1.0, 0.0), new Point2D(-1.0, 0.0)};
      assertPointsEqual(expected2, ConvexPolygon2dCalculator.intersectionWithRayCopy(ray2, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithRay(ray2, result1, result2, polygon) == 2);

      Line2d ray3 = new Line2d(new Point2D(2.0, 0.0), new Vector2D(0.2, 0.0));
      Point2D[] expected3 = null;
      assertPointsEqual(expected3, ConvexPolygon2dCalculator.intersectionWithRayCopy(ray3, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithRay(ray3, result1, result2, polygon) == 0);

      Line2d ray4 = new Line2d(new Point2D(1.0, 1.0), new Vector2D(0.2, -0.1));
      Point2D[] expected4 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected4, ConvexPolygon2dCalculator.intersectionWithRayCopy(ray4, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithRay(ray4, result1, result2, polygon) == 1);

      Line2d ray5 = new Line2d(new Point2D(1.5, 1.0), new Vector2D(0.2, -0.1));
      Point2D[] expected5 = null;
      assertPointsEqual(expected5, ConvexPolygon2dCalculator.intersectionWithRayCopy(ray5, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithRay(ray5, result1, result2, polygon) == 0);

      Line2d ray6 = new Line2d(new Point2D(-1.0, -2.0), new Vector2D(0.3, 0.3));
      Point2D[] expected6 = new Point2D[] {new Point2D(0.0, -1.0), new Point2D(1.0, 0.0)};
      assertPointsEqual(expected6, ConvexPolygon2dCalculator.intersectionWithRayCopy(ray6, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithRay(ray6, result1, result2, polygon) == 2);

      Line2d ray7 = new Line2d(new Point2D(-1.0, -2.0), new Vector2D(0.0, 1.7));
      Point2D[] expected7 = new Point2D[] {new Point2D(-1.0, -1.0), new Point2D(-1.0, 1.0)};
      assertPointsEqual(expected7, ConvexPolygon2dCalculator.intersectionWithRayCopy(ray7, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithRay(ray7, result1, result2, polygon) == 2);

      Line2d ray8 = new Line2d(new Point2D(-0.5, 0.5), new Vector2D(-0.3, -0.3));
      Point2D[] expected8 = new Point2D[] {new Point2D(-1.0, 0.0)};
      assertPointsEqual(expected8, ConvexPolygon2dCalculator.intersectionWithRayCopy(ray8, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithRay(ray8, result1, result2, polygon) == 1);

      Line2d ray9 = new Line2d(new Point2D(-0.5, 0.5), new Vector2D(0.15, 0.3));
      Point2D[] expected9 = new Point2D[] {new Point2D(-0.25, 1.0)};
      assertPointsEqual(expected9, ConvexPolygon2dCalculator.intersectionWithRayCopy(ray9, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithRay(ray9, result1, result2, polygon) == 1);

      Line2d ray10 = new Line2d(new Point2D(0.5, 0.5), new Vector2D(-0.15, 0.3));
      Point2D[] expected10 = new Point2D[] {new Point2D(0.25, 1.0)};
      assertPointsEqual(expected10, ConvexPolygon2dCalculator.intersectionWithRayCopy(ray10, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithRay(ray10, result1, result2, polygon) == 1);

      Line2d ray11 = new Line2d(new Point2D(0.5, 0.5), new Vector2D(0.15, 0.3));
      Point2D[] expected11 = new Point2D[] {new Point2D(0.75, 1.0)};
      assertPointsEqual(expected11, ConvexPolygon2dCalculator.intersectionWithRayCopy(ray11, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithRay(ray11, result1, result2, polygon) == 1);

      Line2d ray12 = new Line2d(new Point2D(0.5, 0.5), new Vector2D(0.15, -0.3));
      Point2D[] expected12 = new Point2D[] {new Point2D(1.0, -0.5)};
      assertPointsEqual(expected12, ConvexPolygon2dCalculator.intersectionWithRayCopy(ray12, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithRay(ray12, result1, result2, polygon) == 1);

      Line2d ray13 = new Line2d(new Point2D(0.5, 0.5), new Vector2D(0.0, -0.3));
      Point2D[] expected13 = new Point2D[] {new Point2D(0.5, -1.0)};
      assertPointsEqual(expected13, ConvexPolygon2dCalculator.intersectionWithRayCopy(ray13, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithRay(ray13, result1, result2, polygon) == 1);

      Line2d ray14 = new Line2d(new Point2D(0.5, 0.5), new Vector2D(0.0, 0.3));
      Point2D[] expected14 = new Point2D[] {new Point2D(0.5, 1.0)};
      assertPointsEqual(expected14, ConvexPolygon2dCalculator.intersectionWithRayCopy(ray14, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithRay(ray14, result1, result2, polygon) == 1);

      Line2d ray15 = new Line2d(new Point2D(1.5, 1.5), new Vector2D(0.0, 0.3));
      Point2D[] expected15 = null;
      assertPointsEqual(expected15, ConvexPolygon2dCalculator.intersectionWithRayCopy(ray15, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithRay(ray15, result1, result2, polygon) == 0);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testOrthogonalProjection1()
   {
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(-1.0, -1.0));
      polygon.addVertex(new Point2D(1.0, -1.0));
      polygon.addVertex(new Point2D(-1.0, 1.0));
      polygon.update();

      Point2D point1 = new Point2D(0.5, 0.5);
      assertPointsEqual(new Point2D(0.0, 0.0), ConvexPolygon2dCalculator.orthogonalProjectionCopy(point1, polygon));

      Point2D point2 = new Point2D(-0.25, -0.25);
      assertPointsEqual(point2, ConvexPolygon2dCalculator.orthogonalProjectionCopy(point2, polygon));

      Point2D point3 = new Point2D(-2.0, -2.0);
      assertPointsEqual(new Point2D(-1.0, -1.0), ConvexPolygon2dCalculator.orthogonalProjectionCopy(point3, polygon));

      Point2D point4 = new Point2D(-0.9, -2.0);
      assertPointsEqual(new Point2D(-0.9, -1.0), ConvexPolygon2dCalculator.orthogonalProjectionCopy(point4, polygon));

      Point2D point5 = new Point2D(-1.1, -2.0);
      assertPointsEqual(new Point2D(-1.0, -1.0), ConvexPolygon2dCalculator.orthogonalProjectionCopy(point5, polygon));

      Point2D point6 = new Point2D(1.8, -1.0);
      assertPointsEqual(new Point2D(1.0, -1.0), ConvexPolygon2dCalculator.orthogonalProjectionCopy(point6, polygon));

      Point2D point7 = new Point2D(1.8, -0.8);
      assertPointsEqual(new Point2D(1.0, -1.0), ConvexPolygon2dCalculator.orthogonalProjectionCopy(point7, polygon));

      Point2D point8 = new Point2D(0.5, 0.0);
      assertPointsEqual(new Point2D(0.25, -0.25), ConvexPolygon2dCalculator.orthogonalProjectionCopy(point8, polygon));

      Point2D point9 = new Point2D(0.0, 0.5);
      assertPointsEqual(new Point2D(-0.25, 0.25), ConvexPolygon2dCalculator.orthogonalProjectionCopy(point9, polygon));

      Point2D point10 = new Point2D(0.0, 0.0);
      assertPointsEqual(point10, ConvexPolygon2dCalculator.orthogonalProjectionCopy(point10, polygon));

      Point2D point11 = new Point2D(1.0, -1.0);
      assertPointsEqual(point11, ConvexPolygon2dCalculator.orthogonalProjectionCopy(point11, polygon));

      Point2D point12 = new Point2D(-1.1, 0.0);
      assertPointsEqual(new Point2D(-1.0, 0.0), ConvexPolygon2dCalculator.orthogonalProjectionCopy(point12, polygon));

      Point2D point13 = new Point2D(-1.5, 3.0);
      assertPointsEqual(new Point2D(-1.0, 1.0), ConvexPolygon2dCalculator.orthogonalProjectionCopy(point13, polygon));

      Point2D point14 = new Point2D(3.0, -1.5);
      assertPointsEqual(new Point2D(1.0, -1.0), ConvexPolygon2dCalculator.orthogonalProjectionCopy(point14, polygon));

      Point2D point15 = new Point2D(1.6, -1.5);
      assertPointsEqual(new Point2D(1.0, -1.0), ConvexPolygon2dCalculator.orthogonalProjectionCopy(point15, polygon));

      Point2D point16 = new Point2D(-2.0, 0.9);
      assertPointsEqual(new Point2D(-1.0, 0.9), ConvexPolygon2dCalculator.orthogonalProjectionCopy(point16, polygon));

      Point2D point17 = new Point2D(-2.0, -0.9);
      assertPointsEqual(new Point2D(-1.0, -0.9), ConvexPolygon2dCalculator.orthogonalProjectionCopy(point17, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testOrthogonalProjection2()
   {
      // empty polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      ConvexPolygon2dCalculator.orthogonalProjectionCopy(new Point2D(), polygon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testOrthogonalProjection3()
   {
      // single point polygon
      Point2D vertex = new Point2D(1.0, 2.0);
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(vertex);
      polygon.update();

      assertPointsEqual(vertex, ConvexPolygon2dCalculator.orthogonalProjectionCopy(new Point2D(0.0, 0.0), polygon));
      assertPointsEqual(vertex, ConvexPolygon2dCalculator.orthogonalProjectionCopy(new Point2D(1.0, -0.2), polygon));
      assertPointsEqual(vertex, ConvexPolygon2dCalculator.orthogonalProjectionCopy(new Point2D(1.0, 2.0), polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testOrthogonalProjection4()
   {
      // line polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(1.0, 2.0));
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Point2D point1 = new Point2D(1.0, -1.0);
      assertPointsEqual(new Point2D(1.0, 1.0), ConvexPolygon2dCalculator.orthogonalProjectionCopy(point1, polygon));

      Point2D point2 = new Point2D(3.0, 2.1);
      assertPointsEqual(new Point2D(1.0, 2.0), ConvexPolygon2dCalculator.orthogonalProjectionCopy(point2, polygon));

      Point2D point3 = new Point2D(0.2, 1.2);
      assertPointsEqual(new Point2D(1.0, 1.2), ConvexPolygon2dCalculator.orthogonalProjectionCopy(point3, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetClosestPointToRay1()
   {
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(-1.0, 0.0));
      polygon.addVertex(new Point2D(0.0, 1.0));
      polygon.addVertex(new Point2D(2.0, 0.0));
      polygon.addVertex(new Point2D(1.0, -1.0));
      polygon.update();

      Line2d ray1 = new Line2d(new Point2D(5.0, -3.0), new Vector2D(0.0, 1.0));
      assertPointsEqual(new Point2D(2.0, 0.0), ConvexPolygon2dCalculator.getClosestPointToRayCopy(ray1, polygon));

      Line2d ray2 = new Line2d(new Point2D(1.0, 1.0), new Vector2D(0.5, 0.5));
      assertPointsEqual(new Point2D(4.0/5.0, 3.0/5.0), ConvexPolygon2dCalculator.getClosestPointToRayCopy(ray2, polygon));

      Line2d ray3 = new Line2d(new Point2D(1.0, 1.0), new Vector2D(-0.5, 0.1));
      assertPointsEqual(new Point2D(0.0, 1.0), ConvexPolygon2dCalculator.getClosestPointToRayCopy(ray3, polygon));

      Line2d ray4 = new Line2d(new Point2D(-0.75, 0.75), new Vector2D(0.0, 0.1));
      assertPointsEqual(new Point2D(-0.5, 0.5), ConvexPolygon2dCalculator.getClosestPointToRayCopy(ray4, polygon));

      Line2d ray5 = new Line2d(new Point2D(-0.75, 0.75), new Vector2D(0.3, 0.3));
      assertPointsEqual(new Point2D(-0.5, 0.5), ConvexPolygon2dCalculator.getClosestPointToRayCopy(ray5, polygon));

      Line2d ray6 = new Line2d(new Point2D(-0.75, 0.75), new Vector2D(-0.3, -0.3));
      assertPointsEqual(new Point2D(-0.5, 0.5), ConvexPolygon2dCalculator.getClosestPointToRayCopy(ray6, polygon));

      Line2d ray7 = new Line2d(new Point2D(-0.75, 0.75), new Vector2D(0.3, 0.31));
      assertPointsEqual(new Point2D(-0.5, 0.5), ConvexPolygon2dCalculator.getClosestPointToRayCopy(ray7, polygon));

      Line2d ray8 = new Line2d(new Point2D(-0.75, 0.75), new Vector2D(0.3, 0.29));
      assertPointsEqual(new Point2D(0.0, 1.0), ConvexPolygon2dCalculator.getClosestPointToRayCopy(ray8, polygon));

      Line2d ray9 = new Line2d(new Point2D(1.75, -0.75), new Vector2D(1.0, 1.0));
      assertPointsEqual(new Point2D(1.5, -0.5), ConvexPolygon2dCalculator.getClosestPointToRayCopy(ray9, polygon));

      Line2d ray10 = new Line2d(new Point2D(1.75, -0.75), new Vector2D(-0.3, -0.3));
      assertPointsEqual(new Point2D(1.5, -0.5), ConvexPolygon2dCalculator.getClosestPointToRayCopy(ray10, polygon));

      Line2d ray11 = new Line2d(new Point2D(1.0, -1.2), new Vector2D(-2.0, 1.0));
      assertPointsEqual(new Point2D(1.0, -1.0), ConvexPolygon2dCalculator.getClosestPointToRayCopy(ray11, polygon));

      Line2d ray12 = new Line2d(new Point2D(1.0, -1.2), new Vector2D(2.0, -1.0));
      assertPointsEqual(new Point2D(1.0, -1.0), ConvexPolygon2dCalculator.getClosestPointToRayCopy(ray12, polygon));

      Line2d ray13 = new Line2d(new Point2D(-0.1, -0.7), new Vector2D(-2.0, 1.0));
      assertPointsEqual(new Point2D(0.0, -0.5), ConvexPolygon2dCalculator.getClosestPointToRayCopy(ray13, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetClosestPointToRay2()
   {
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      assertTrue(ConvexPolygon2dCalculator.getClosestPointToRayCopy(new Line2d(), polygon) == null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetClosestPointToRay3()
   {
      Point2D vertex = new Point2D(1.0, -1.0);

      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(vertex);
      polygon.update();

      Line2d ray1 = new Line2d(new Point2D(5.0, -3.0), new Vector2D(0.0, 1.0));
      assertPointsEqual(vertex, ConvexPolygon2dCalculator.getClosestPointToRayCopy(ray1, polygon));

      Line2d ray2 = new Line2d(new Point2D(0.0, 0.0), new Vector2D(1.0, 0.0));
      assertPointsEqual(vertex, ConvexPolygon2dCalculator.getClosestPointToRayCopy(ray2, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetClosestPointToRay4()
   {
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(2.0, -5.0));
      polygon.addVertex(new Point2D(1.0, -6.0));
      polygon.update();

      Line2d ray1 = new Line2d(new Point2D(1.0, -5.0), new Vector2D(1.0, 0.1));
      assertPointsEqual(new Point2D(2.0, -5.0), ConvexPolygon2dCalculator.getClosestPointToRayCopy(ray1, polygon));

      Line2d ray2 = new Line2d(new Point2D(1.25, -5.25), new Vector2D(0.75, 0.3));
      assertPointsEqual(new Point2D(2.0, -5.0), ConvexPolygon2dCalculator.getClosestPointToRayCopy(ray2, polygon));

      Line2d ray3 = new Line2d(new Point2D(1.25, -5.25), new Vector2D(0.75, 0.8));
      assertPointsEqual(new Point2D(1.5, -5.5), ConvexPolygon2dCalculator.getClosestPointToRayCopy(ray3, polygon));

      Line2d ray4 = new Line2d(new Point2D(1.25, -5.25), new Vector2D(1.0, 1.0));
      assertPointsEqual(new Point2D(1.5, -5.5), ConvexPolygon2dCalculator.getClosestPointToRayCopy(ray4, polygon));

      Line2d ray5 = new Line2d(new Point2D(1.25, -5.25), new Vector2D(-1.0, -1.0));
      assertPointsEqual(new Point2D(1.5, -5.5), ConvexPolygon2dCalculator.getClosestPointToRayCopy(ray5, polygon));

      Line2d ray6 = new Line2d(new Point2D(1.75, -5.75), new Vector2D(1.0, 1.0));
      assertPointsEqual(new Point2D(1.5, -5.5), ConvexPolygon2dCalculator.getClosestPointToRayCopy(ray6, polygon));

      Line2d ray7 = new Line2d(new Point2D(1.75, -5.75), new Vector2D(-1.0, -1.0));
      assertPointsEqual(new Point2D(1.5, -5.5), ConvexPolygon2dCalculator.getClosestPointToRayCopy(ray7, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetEdgeNormal()
   {
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(-1.0, 0.0));
      polygon.addVertex(new Point2D(0.0, 1.0));
      polygon.addVertex(new Point2D(2.0, 0.0));
      polygon.addVertex(new Point2D(1.0, -1.0));
      polygon.update();

      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         Vector2D normal = new Vector2D();
         ConvexPolygon2dCalculator.getEdgeNormal(i, normal, polygon);

         Vector2D expected = new Vector2D();
         Point2DReadOnly edgeStart = polygon.getVertex(i);
         Point2DReadOnly edgeEnd = polygon.getNextVertex(i);
         Vector2D edgeVector = new Vector2D();
         edgeVector.sub(edgeEnd, edgeStart);
         GeometryTools.getPerpendicularVector(edgeVector, expected);
         expected.normalize();

         assertTrue("Expected normal Vector did not match computed one.", expected.epsilonEquals(normal, epsilon));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetClosestEdge1()
   {
      Point2D vertex1 = new Point2D(0.0, 0.0);
      Point2D vertex2 = new Point2D(-1.0, 0.0);
      Point2D vertex3 = new Point2D(0.0, 1.0);
      Point2D vertex4 = new Point2D(1.0, 1.0);

      // add in order so vertices do not get changed when update is called.
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(vertex1);
      polygon.addVertex(vertex2);
      polygon.addVertex(vertex3);
      polygon.addVertex(vertex4);
      polygon.update();

      LineSegment2d edge1 = new LineSegment2d(vertex1, vertex2);
      LineSegment2d edge2 = new LineSegment2d(vertex2, vertex3);
      LineSegment2d edge3 = new LineSegment2d(vertex3, vertex4);
      LineSegment2d edge4 = new LineSegment2d(vertex4, vertex1);

      Point2D point1 = new Point2D(0.5, 0.1);
      assertEdgesEqual(edge4, ConvexPolygon2dCalculator.getClosestEdgeCopy(point1, polygon));

      Point2D point2 = new Point2D(-0.5, -0.5);
      assertEdgesEqual(edge1, ConvexPolygon2dCalculator.getClosestEdgeCopy(point2, polygon));

      Point2D point3 = new Point2D(-0.5, 0.5);
      assertEdgesEqual(edge2, ConvexPolygon2dCalculator.getClosestEdgeCopy(point3, polygon));

      Point2D point4 = new Point2D(-0.5, 0.25);
      assertEdgesEqual(edge2, ConvexPolygon2dCalculator.getClosestEdgeCopy(point4, polygon));

      Point2D point5 = new Point2D(-0.1, 3.0);
      assertEdgesEqual(edge2, ConvexPolygon2dCalculator.getClosestEdgeCopy(point5, polygon));

      Point2D point6 = new Point2D(0.1, 0.8);
      assertEdgesEqual(edge3, ConvexPolygon2dCalculator.getClosestEdgeCopy(point6, polygon));

      Point2D point7 = new Point2D(-0.11, 0.2);
      assertEdgesEqual(edge1, ConvexPolygon2dCalculator.getClosestEdgeCopy(point7, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetClosestEdge2()
   {
      Point2D vertex1 = new Point2D(2.0, 2.0);
      Point2D vertex2 = new Point2D(3.0, 3.0);

      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(vertex1);
      polygon.addVertex(vertex2);
      polygon.update();

      LineSegment2d edge1 = new LineSegment2d(vertex1, vertex2);

      Point2D point1 = new Point2D(0.5, 0.1);
      assertEdgesEqual(edge1, ConvexPolygon2dCalculator.getClosestEdgeCopy(point1, polygon));

      Point2D point2 = new Point2D(4.0, 4.0);
      assertEdgesEqual(edge1, ConvexPolygon2dCalculator.getClosestEdgeCopy(point2, polygon));

      Point2D point3 = new Point2D(1.0, 1.0);
      assertEdgesEqual(edge1, ConvexPolygon2dCalculator.getClosestEdgeCopy(point3, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetClosestEdge3()
   {
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D());
      polygon.update();
      assertTrue(ConvexPolygon2dCalculator.getClosestEdgeCopy(new Point2D(), polygon) == null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetClosestEdge4()
   {
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      assertTrue(ConvexPolygon2dCalculator.getClosestEdgeCopy(new Point2D(), polygon) == null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIntersectionWithLineSegment1()
   {
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(-1.0, -1.0));
      polygon.addVertex(new Point2D(1.0, -1.0));
      polygon.addVertex(new Point2D(-1.0, 1.0));
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Point2D result1 = new Point2D();
      Point2D result2 = new Point2D();

      LineSegment2d segment1 = new LineSegment2d(new Point2D(0.0, 0.0), new Point2D(2.0, 0.0));
      Point2D[] expected1 = new Point2D[] {new Point2D(1.0, 0.0)};
      assertPointsEqual(expected1, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment1, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment1, result1, result2, polygon) == 1);

      LineSegment2d segment2 = new LineSegment2d(new Point2D(-2.0, 0.0), new Point2D(2.0, 0.0));
      Point2D[] expected2 = new Point2D[] {new Point2D(-1.0, 0.0), new Point2D(1.0, 0.0)};
      assertPointsEqual(expected2, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment2, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment2, result1, result2, polygon) == 2);

      LineSegment2d segment3 = new LineSegment2d(new Point2D(-0.5, 0.0), new Point2D(0.5, 0.0));
      Point2D[] expected3 = null;
      assertPointsEqual(expected3, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment3, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment3, result1, result2, polygon) == 0);

      LineSegment2d segment4 = new LineSegment2d(new Point2D(-3.5, 0.0), new Point2D(-1.5, 0.0));
      Point2D[] expected4 = null;
      assertPointsEqual(expected4, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment4, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment4, result1, result2, polygon) == 0);

      LineSegment2d segment5 = new LineSegment2d(new Point2D(-1.5, 0.0), new Point2D(0.0, 1.5));
      Point2D[] expected5 = new Point2D[] {new Point2D(-1.0, 0.5), new Point2D(-0.5, 1.0)};
      assertPointsEqual(expected5, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment5, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment5, result1, result2, polygon) == 2);

      LineSegment2d segment6 = new LineSegment2d(new Point2D(-1.0, 0.5), new Point2D(-0.5, 1.0));
      Point2D[] expected6 = new Point2D[] {new Point2D(-1.0, 0.5), new Point2D(-0.5, 1.0)};
      assertPointsEqual(expected6, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment6, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment6, result1, result2, polygon) == 2);

      LineSegment2d segment7 = new LineSegment2d(new Point2D(-1.5, 1.0), new Point2D(1.5, 1.0));
      Point2D[] expected7 = new Point2D[] {new Point2D(-1.0, 1.0), new Point2D(1.0, 1.0)};
      assertPointsEqual(expected7, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment7, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment7, result1, result2, polygon) == 2);

      LineSegment2d segment8 = new LineSegment2d(new Point2D(-2.5, 1.0), new Point2D(-1.5, 1.0));
      Point2D[] expected8 = null;
      assertPointsEqual(expected8, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment8, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment8, result1, result2, polygon) == 0);

      LineSegment2d segment9 = new LineSegment2d(new Point2D(1.0, 0.0), new Point2D(1.0, 2.0));
      Point2D[] expected9 = new Point2D[] {new Point2D(1.0, 0.0), new Point2D(1.0, 1.0)};
      assertPointsEqual(expected9, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment9, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment9, result1, result2, polygon) == 2);

      LineSegment2d segment10 = new LineSegment2d(new Point2D(1.0, 0.0), new Point2D(1.0, 0.5));
      Point2D[] expected10 = new Point2D[] {new Point2D(1.0, 0.0), new Point2D(1.0, 0.5)};
      assertPointsEqual(expected10, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment10, polygon), false);
      result1.set(expected10[0]);
      result2.set(expected10[0]);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment10, result1, result2, polygon) == 2);
      result1.set(expected10[1]);
      result2.set(expected10[1]);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment10, result1, result2, polygon) == 2);

      LineSegment2d segment11 = new LineSegment2d(new Point2D(-0.5, 1.0), new Point2D(-1.0, 0.5));
      Point2D[] expected11 = new Point2D[] {new Point2D(-1.0, 0.5), new Point2D(-0.5, 1.0)};
      assertPointsEqual(expected11, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment11, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment11, result1, result2, polygon) == 2);

      LineSegment2d segment12 = new LineSegment2d(new Point2D(-1.5, 0.5), new Point2D(1.5, 0.5));
      Point2D[] expected12 = new Point2D[] {new Point2D(-1.0, 0.5), new Point2D(1.0, 0.5)};
      assertPointsEqual(expected12, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment12, polygon), false);
      result1.set(expected12[0]);
      result2.set(expected12[0]);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment12, result1, result2, polygon) == 2);
      result1.set(expected12[1]);
      result2.set(expected12[1]);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment12, result1, result2, polygon) == 2);

      LineSegment2d segment13 = new LineSegment2d(new Point2D(0.0, -1.5), new Point2D(1.5, -1.5));
      Point2D[] expected13 = null;
      assertPointsEqual(expected13, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment13, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment13, result1, result2, polygon) == 0);

      LineSegment2d segment14 = new LineSegment2d(new Point2D(0.0, 1.5), new Point2D(1.5, 1.5));
      Point2D[] expected14 = null;
      assertPointsEqual(expected14, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment14, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment14, result1, result2, polygon) == 0);

      LineSegment2d segment15 = new LineSegment2d(new Point2D(1.0, 1.0), new Point2D(0.5, 1.0));
      Point2D[] expected15 = new Point2D[] {new Point2D(1.0, 1.0), new Point2D(0.5, 1.0)};
      assertPointsEqual(expected15, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment15, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment15, result1, result2, polygon) == 2);

      LineSegment2d segment16 = new LineSegment2d(new Point2D(1.0, 1.0), new Point2D(1.0, 0.5));
      Point2D[] expected16 = new Point2D[] {new Point2D(1.0, 1.0), new Point2D(1.0, 0.5)};
      assertPointsEqual(expected16, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment16, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment16, result1, result2, polygon) == 2);

      LineSegment2d segment17 = new LineSegment2d(new Point2D(0.5, 1.0), new Point2D(1.0, 1.0));
      Point2D[] expected17 = new Point2D[] {new Point2D(1.0, 1.0), new Point2D(0.5, 1.0)};
      assertPointsEqual(expected17, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment17, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment17, result1, result2, polygon) == 2);

      LineSegment2d segment18 = new LineSegment2d(new Point2D(1.0, 0.5), new Point2D(1.0, 1.0));
      Point2D[] expected18 = new Point2D[] {new Point2D(1.0, 1.0), new Point2D(1.0, 0.5)};
      assertPointsEqual(expected18, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment18, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment18, result1, result2, polygon) == 2);

      LineSegment2d segment19 = new LineSegment2d(new Point2D(-1.5, 1.0), new Point2D(-0.5, 1.0));
      Point2D[] expected19 = new Point2D[] {new Point2D(-1.0, 1.0), new Point2D(-0.5, 1.0)};
      assertPointsEqual(expected19, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment19, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment19, result1, result2, polygon) == 2);

      LineSegment2d segment20 = new LineSegment2d(new Point2D(-1.5, 1.0), new Point2D(-1.0, 1.0));
      Point2D[] expected20 = new Point2D[] {new Point2D(-1.0, 1.0)};
      assertPointsEqual(expected20, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment20, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment20, result1, result2, polygon) == 1);

      LineSegment2d segment21 = new LineSegment2d(new Point2D(-1.0, 1.0), new Point2D(-1.5, 1.0));
      Point2D[] expected21 = new Point2D[] {new Point2D(-1.0, 1.0)};
      assertPointsEqual(expected21, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment21, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment21, result1, result2, polygon) == 1);

      LineSegment2d segment22 = new LineSegment2d(new Point2D(1.0, 1.0), new Point2D(1.5, 1.0));
      Point2D[] expected22 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected22, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment22, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment22, result1, result2, polygon) == 1);

      LineSegment2d segment23 = new LineSegment2d(new Point2D(1.5, 1.0), new Point2D(1.0, 1.0));
      Point2D[] expected23 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected23, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment23, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment23, result1, result2, polygon) == 1);

      LineSegment2d segment24 = new LineSegment2d(new Point2D(1.5, 1.5), new Point2D(1.0, 1.0));
      Point2D[] expected24 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected24, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment24, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment24, result1, result2, polygon) == 1);

      LineSegment2d segment25 = new LineSegment2d(new Point2D(0.5, 1.5), new Point2D(1.0, 1.0));
      Point2D[] expected25 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected25, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment25, polygon), false);
      result1.set(expected25[0]);
      result2.set(expected25[0]);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment25, result1, result2, polygon) == 1);

      LineSegment2d segment26 = new LineSegment2d(new Point2D(-1.0, -1.0), new Point2D(0.8, 1.0));
      Point2D[] expected26 = new Point2D[] {new Point2D(0.8, 1.0), new Point2D(-1.0, -1.0)};
      assertPointsEqual(expected26, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment26, polygon), false);
      result1.set(expected26[0]);
      result2.set(expected26[0]);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment26, result1, result2, polygon) == 2);
      result1.set(expected26[1]);
      result2.set(expected26[1]);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment26, result1, result2, polygon) == 2);

      LineSegment2d segment27 = new LineSegment2d(new Point2D(1.0, 1.0), new Point2D(-1.0, -1.0));
      Point2D[] expected27 = new Point2D[] {new Point2D(1.0, 1.0), new Point2D(-1.0, -1.0)};
      assertPointsEqual(expected27, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment27, polygon), false);
      result1.set(expected27[0]);
      result2.set(expected27[0]);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment27, result1, result2, polygon) == 2);
      result1.set(expected27[1]);
      result2.set(expected27[1]);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment27, result1, result2, polygon) == 2);

      LineSegment2d segment28 = new LineSegment2d(new Point2D(1.0, -0.5), new Point2D(1.0, 0.0));
      Point2D[] expected28 = new Point2D[] {new Point2D(1.0, 0.0), new Point2D(1.0, -0.5)};
      assertPointsEqual(expected28, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment28, polygon), false);
      result1.set(expected28[0]);
      result2.set(expected28[0]);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment28, result1, result2, polygon) == 2);
      result1.set(expected28[1]);
      result2.set(expected28[1]);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment28, result1, result2, polygon) == 2);

      LineSegment2d segment29 = new LineSegment2d(new Point2D(1.0, -1.5), new Point2D(1.0, 0.5));
      Point2D[] expected29 = new Point2D[] {new Point2D(1.0, 0.5), new Point2D(1.0, -1.0)};
      assertPointsEqual(expected29, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment29, polygon), false);
      result1.set(expected29[0]);
      result2.set(expected29[0]);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment29, result1, result2, polygon) == 2);
      result1.set(expected29[1]);
      result2.set(expected29[1]);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment29, result1, result2, polygon) == 2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIntersectionWithLineSegment2()
   {
      // empty polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();

      Point2D result1 = new Point2D();
      Point2D result2 = new Point2D();

      LineSegment2d segment1 = new LineSegment2d();
      Point2D[] expected1 = null;
      assertPointsEqual(expected1, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment1, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment1, result1, result2, polygon) == 0);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIntersectionWithLineSegment3()
   {
      // point polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Point2D result1 = new Point2D();
      Point2D result2 = new Point2D();

      LineSegment2d segment1 = new LineSegment2d(new Point2D(1.0, 0.0), new Point2D(2.0, 0.0));
      Point2D[] expected1 = null;
      assertPointsEqual(expected1, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment1, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment1, result1, result2, polygon) == 0);

      LineSegment2d segment2 = new LineSegment2d(new Point2D(1.0, 1.0), new Point2D(2.0, 0.0));
      Point2D[] expected2 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected2, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment2, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment2, result1, result2, polygon) == 1);

      LineSegment2d segment3 = new LineSegment2d(new Point2D(0.0, 0.0), new Point2D(1.0, 1.0));
      Point2D[] expected3 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected3, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment3, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment3, result1, result2, polygon) == 1);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIntersectionWithLineSegment4()
   {
      // line polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.addVertex(new Point2D(3.0, 3.0));
      polygon.update();

      Point2D result1 = new Point2D();
      Point2D result2 = new Point2D();

      LineSegment2d segment1 = new LineSegment2d(new Point2D(0.0, 0.0), new Point2D(3.0, 3.0));
      Point2D[] expected1 = new Point2D[] {new Point2D(1.0, 1.0), new Point2D(3.0, 3.0)};
      assertPointsEqual(expected1, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment1, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment1, result1, result2, polygon) == 2);

      LineSegment2d segment2 = new LineSegment2d(new Point2D(1.5, 1.5), new Point2D(2.5, 2.5));
      Point2D[] expected2 = new Point2D[] {new Point2D(1.5, 1.5), new Point2D(2.5, 2.5)};
      assertPointsEqual(expected2, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment2, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment2, result1, result2, polygon) == 2);

      LineSegment2d segment3 = new LineSegment2d(new Point2D(0.5, 0.5), new Point2D(3.5, 3.5));
      Point2D[] expected3 = new Point2D[] {new Point2D(1.0, 1.0), new Point2D(3.0, 3.0)};
      assertPointsEqual(expected3, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment3, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment3, result1, result2, polygon) == 2);

      LineSegment2d segment4 = new LineSegment2d(new Point2D(1.0, 1.0), new Point2D(3.0, 3.0));
      Point2D[] expected4 = new Point2D[] {new Point2D(1.0, 1.0), new Point2D(3.0, 3.0)};
      assertPointsEqual(expected4, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment4, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment4, result1, result2, polygon) == 2);

      LineSegment2d segment5 = new LineSegment2d(new Point2D(0.0, 0.0), new Point2D(0.5, 0.5));
      Point2D[] expected5 = null;
      assertPointsEqual(expected5, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment5, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment5, result1, result2, polygon) == 0);

      LineSegment2d segment6 = new LineSegment2d(new Point2D(0.5, 0.5), new Point2D(1.0, 1.0));
      Point2D[] expected6 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected6, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment6, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment6, result1, result2, polygon) == 1);

      LineSegment2d segment7 = new LineSegment2d(new Point2D(2.0, 0.5), new Point2D(2.0, 5.0));
      Point2D[] expected7 = new Point2D[] {new Point2D(2.0, 2.0)};
      assertPointsEqual(expected7, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment7, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment7, result1, result2, polygon) == 1);

      LineSegment2d segment8 = new LineSegment2d(new Point2D(2.0, 0.5), new Point2D(1.0, 1.0));
      Point2D[] expected8 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected8, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment8, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment8, result1, result2, polygon) == 1);

      LineSegment2d segment9 = new LineSegment2d(new Point2D(4.0, 4.0), new Point2D(2.0, 2.0));
      Point2D[] expected9 = new Point2D[] {new Point2D(3.0, 3.0), new Point2D(2.0, 2.0)};
      assertPointsEqual(expected9, ConvexPolygon2dCalculator.intersectionWithLineSegmentCopy(segment9, polygon), false);
      assertTrue(ConvexPolygon2dCalculator.intersectionWithLineSegment(segment9, result1, result2, polygon) == 2);
   }

   private static void assertEdgesEqual(LineSegment2d[] expected, LineSegment2d[] actual, boolean enforceOrder)
   {
      if (expected == null || actual == null)
      {
         assertTrue("Expected did not equal actual. One of them was null.", expected == actual);
         return;
      }

      assertEquals("Array lengths are not equal.", expected.length, actual.length);
      int points = expected.length;
      for (int i = 0; i < points; i++)
      {
         if (enforceOrder)
         {
            assertEdgesEqual(expected[i], actual[i]);
            continue;
         }

         boolean foundPoint = false;
         for (int j = 0; j < points; j++)
         {
            if (expected[i].epsilonEquals(actual[j], epsilon))
               foundPoint = true;
         }
         assertTrue("Did not find edge.", foundPoint);
      }
   }

   private static void assertEdgesEqual(LineSegment2d expected, LineSegment2d actual)
   {
      assertTrue("Edge did not match expected.", expected.epsilonEquals(actual, epsilon));
   }

   private static void assertPointsEqual(Point2D[] expected, Point2D[] actual, boolean enforceOrder)
   {
      if (expected == null || actual == null)
      {
         assertTrue("Expected did not equal actual. One of them was null.", expected == actual);
         return;
      }

      assertEquals("Array lengths are not equal.", expected.length, actual.length);
      int points = expected.length;
      for (int i = 0; i < points; i++)
      {
         if (enforceOrder)
         {
            assertPointsEqual(expected[i], actual[i]);
            continue;
         }

         boolean foundPoint = false;
         for (int j = 0; j < points; j++)
         {
            if (expected[i].epsilonEquals(actual[j], epsilon))
               foundPoint = true;
         }
         assertTrue("Did not find point.", foundPoint);
      }
   }

   private static void assertIndicesCorrect(int[] expected, int[] actual)
   {
      if (expected == null || actual == null)
      {
         assertTrue("Expected did not equal actual. One of them was null.", expected == actual);
         return;
      }

      assertEquals("Array lengths are not equal.", expected.length, actual.length);
      for (int i = 0; i < expected.length; i++)
         assertIndexCorrect(expected[i], actual[i]);
   }

   private static void assertIndexCorrect(int expected, int actual)
   {
      assertEquals("Index does not equal expected.", expected, actual);
   }

   private static void assertDistanceCorrect(double expected, double actual)
   {
      assertEquals("Distance does not equal expected.", expected, actual, epsilon);
   }

   private static void assertPointsEqual(Point2D expected, Point2D actual)
   {
      if (expected == null && actual == null)
         return;

      double localEpsilon = epsilon * expected.distance(new Point2D());
      assertTrue("Point does not match expected.", expected.epsilonEquals(actual, localEpsilon));
   }

   public static void main(String[] args)
   {
      String targetTests = "us.ihmc.robotics.geometry.ConvexPolygon2dCalculatorTest";
      String targetClasses = "us.ihmc.robotics.geometry.ConvexPolygon2dCalculator";
      MutationTestingTools.doPITMutationTestAndOpenResult(targetTests, targetClasses);
   }
}
