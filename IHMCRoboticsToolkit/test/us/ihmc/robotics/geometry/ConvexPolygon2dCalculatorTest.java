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
   public void testIsPointInside5()
   {
      // foot polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2d(-0.0682, -0.084));
      polygon.addVertex(new Point2d(0.1418, -0.0834));
      polygon.addVertex(new Point2d(0.1421, -0.1934));
      polygon.addVertex(new Point2d(-0.0679, -0.194));
      polygon.update();

      Point2d point1 = new Point2d(0.0342, -0.0006);
      System.out.println(ConvexPolygon2dCalculator.getSignedDistance(point1, polygon));
      assertFalse(ConvexPolygon2dCalculator.isPointInside(point1, 0.02, polygon));

      Point2d point2 = new Point2d(0.0351, -0.0983);
      assertTrue(ConvexPolygon2dCalculator.isPointInside(point2, polygon));
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

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCanObserverSeeEdge1()
   {
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2d(0.0, 0.0));
      polygon.addVertex(new Point2d(1.0, 0.0));
      polygon.addVertex(new Point2d(0.0, 1.0));
      polygon.addVertex(new Point2d(1.0, 1.0));
      polygon.update();

      // observer inside polygon can not see any outside edges
      Point2d observer1 = new Point2d(0.5, 0.5);
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
         assertFalse(ConvexPolygon2dCalculator.canObserverSeeEdge(i, observer1, polygon));

      // this observer should be able to see the edge starting at vertex (0.0, 0.0)
      Point2d observer2 = new Point2d(-0.5, 0.5);
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         if (polygon.getVertex(i).epsilonEquals(new Point2d(0.0, 0.0), epsilon))
            assertTrue(ConvexPolygon2dCalculator.canObserverSeeEdge(i, observer2, polygon));
         else
            assertFalse(ConvexPolygon2dCalculator.canObserverSeeEdge(i, observer2, polygon));
      }

      // this observer should be able to see the edges starting at vertex (0.0, 1.0) and at (1.0, 1.0)
      Point2d observer3 = new Point2d(1.5, 1.5);
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         if (polygon.getVertex(i).epsilonEquals(new Point2d(0.0, 1.0), epsilon))
            assertTrue(ConvexPolygon2dCalculator.canObserverSeeEdge(i, observer3, polygon));
         else if (polygon.getVertex(i).epsilonEquals(new Point2d(1.0, 1.0), epsilon))
            assertTrue(ConvexPolygon2dCalculator.canObserverSeeEdge(i, observer3, polygon));
         else
            assertFalse(ConvexPolygon2dCalculator.canObserverSeeEdge(i, observer3, polygon));
      }
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCanObserverSeeEdge2()
   {
      // line polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2d(1.0, 0.0));
      polygon.addVertex(new Point2d(0.0, 1.0));
      polygon.update();

      // should be able to see one edge
      Point2d observer1 = new Point2d(0.0, 0.0);
      boolean seeEdge1 = ConvexPolygon2dCalculator.canObserverSeeEdge(0, observer1, polygon);
      boolean seeEdge2 = ConvexPolygon2dCalculator.canObserverSeeEdge(1, observer1, polygon);
      assertTrue((seeEdge1 || seeEdge2) && !(seeEdge1 && seeEdge2));
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCanObserverSeeEdge3()
   {
      // point polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2d(1.0, 1.0));
      polygon.update();

      Point2d observer1 = new Point2d(0.0, 0.0);
      assertFalse(ConvexPolygon2dCalculator.canObserverSeeEdge(0, observer1, polygon));
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetVertexOnSide1()
   {
      // add vertices in clockwise order so updating the polygon does not change indices
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2d(0.0, 1.0));
      polygon.addVertex(new Point2d(1.0, 1.0));
      polygon.addVertex(new Point2d(1.0, 0.0));
      polygon.addVertex(new Point2d(0.0, 0.0));
      polygon.update();

      Point2d observer1 = new Point2d(0.5, -0.5);
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

      Point2d observer2 = new Point2d(0.5, 0.5);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(0, 0, observer2, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(0, 1, observer2, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(0, 2, observer2, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(0, 3, observer2, polygon), 3);
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetVertexOnSide2()
   {
      // add vertices in clockwise order so updating the polygon does not change indices
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2d(0.0, 1.0));
      polygon.addVertex(new Point2d(1.0, 1.0));
      polygon.update();

      Point2d observer1 = new Point2d(0.0, 0.0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(0, 1, observer1, polygon), 0);

      Point2d observer2 = new Point2d(0.0, 2.0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(0, 1, observer2, polygon), 1);

      Point2d observer3 = new Point2d(10.0, 0.0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(0, 1, observer3, polygon), 0);

      Point2d observer4 = new Point2d(2.0, 2.0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getVertexOnLeft(0, 1, observer4, polygon), 1);
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetMiddleIndexCounterClockwise1()
   {
      // do not update polygon to keep number of vertices
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      for (int i = 0; i < 6; i++)
         polygon.addVertex(new Point2d());
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
         polygon.addVertex(new Point2d());
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
      polygon.addVertex(new Point2d());
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(0, 0, polygon), 0);

   }

   private static void assertIndexCorrect(int expected, int actual)
   {
      assertEquals("Index did not equal expected.", expected, actual);
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
