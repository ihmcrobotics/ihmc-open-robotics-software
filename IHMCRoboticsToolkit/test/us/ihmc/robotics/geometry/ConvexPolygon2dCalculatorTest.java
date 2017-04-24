package us.ihmc.robotics.geometry;

import static org.junit.Assert.*;

import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

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
      assertPointsEqual(new Point2D[] {vertex5, vertex1}, polygon.lineOfSightVertices(observer1), true);

      Point2D observer2 = new Point2D(1.0, -0.5);
      assertPointsEqual(new Point2D[] {vertex3, vertex5}, polygon.lineOfSightVertices(observer2), true);

      Point2D observer3 = new Point2D(-1.0, -2.0 + epsilon);
      assertPointsEqual(new Point2D[] {vertex4, vertex1}, polygon.lineOfSightVertices(observer3), true);

      Point2D observer4 = new Point2D(-1.0, -2.0 - epsilon);
      assertPointsEqual(new Point2D[] {vertex3, vertex1}, polygon.lineOfSightVertices(observer4), true);

      Point2D observer5 = new Point2D(1.5 + epsilon, 0.5);
      assertPointsEqual(new Point2D[] {vertex2, vertex4}, polygon.lineOfSightVertices(observer5), true);

      Point2D observer6 = vertex3;
      assertPointsEqual(null, polygon.lineOfSightVertices(observer6), true);

      Point2D observer7 = new Point2D(0.5, 0.5);
      assertPointsEqual(null, polygon.lineOfSightVertices(observer7), true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetLineOfSightVertices2()
   {
      // empty polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();

      Point2D observer1 = new Point2D(0.5, 0.5);
      assertPointsEqual(null, polygon.lineOfSightVertices(observer1), true);
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
      assertPointsEqual(null, polygon.lineOfSightVertices(observer1), true);

      Point2D observer2 = new Point2D(0.5, 0.5);
      assertPointsEqual(new Point2D[] {vertex, vertex}, polygon.lineOfSightVertices(observer2), true);

      assertIndicesCorrect(new int[] {0, 0}, polygon.lineOfSightIndices(observer2));
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
      assertIndicesCorrect(new int[] {1, 0}, polygon.lineOfSightIndices(observer1));

      Point2D observer2 = new Point2D(0.5, 0.0);
      assertIndicesCorrect(new int[] {1, 0}, polygon.lineOfSightIndices(observer2));

      Point2D observer3 = new Point2D(0.5, 1.5);
      assertIndicesCorrect(new int[] {0, 1}, polygon.lineOfSightIndices(observer3));

      Point2D observer4 = new Point2D(0.5, 1.0);
      assertIndicesCorrect(null, polygon.lineOfSightIndices(observer4));

      Point2D observer5 = new Point2D(1.0, 1.0);
      assertIndicesCorrect(null, polygon.lineOfSightIndices(observer5));
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

      Line2d line5 = new Line2d(new Point2D(-1.0e-6, 0.3), new Vector2D(0.0, -1.0));
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
         EuclidGeometryTools.perpendicularVector2D(edgeVector, expected);
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

   private static void assertPointsEqual(Point2D expected, Point2D actual)
   {
      if (expected == null && actual == null)
         return;

      double localEpsilon = epsilon * expected.distance(new Point2D());
      assertTrue("Point does not match expected.", expected.epsilonEquals(actual, localEpsilon));
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(ConvexPolygon2dCalculator.class, ConvexPolygon2dCalculatorTest.class);
   }
}
