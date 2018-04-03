package us.ihmc.robotics.geometry;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
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
   public void testIsPolygonInside1()
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.addVertex(new Point2D(2.0, 1.0));
      polygon.addVertex(new Point2D(1.0, 2.0));
      polygon.update();

      ConvexPolygon2D polygonToTest1 = new ConvexPolygon2D();
      polygonToTest1.addVertex(new Point2D(0.1, 0.1));
      polygonToTest1.addVertex(new Point2D(0.2, 0.2));
      polygonToTest1.update();
      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(polygonToTest1, polygon));

      ConvexPolygon2D polygonToTest2 = new ConvexPolygon2D(polygon);
      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(polygonToTest2, polygon));
      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(polygonToTest2, epsilon, polygon));
      assertFalse(ConvexPolygon2dCalculator.isPolygonInside(polygonToTest2, -epsilon, polygon));

      ConvexPolygon2D polygonToTest3 = new ConvexPolygon2D();
      polygonToTest3.addVertex(new Point2D(0.3, 0.9));
      polygonToTest3.addVertex(new Point2D(0.1, 0.1));
      polygonToTest3.addVertex(new Point2D(1.0, 1.2));
      polygonToTest3.update();
      assertFalse(ConvexPolygon2dCalculator.isPolygonInside(polygonToTest3, polygon));

      ConvexPolygon2D polygonToTest4 = new ConvexPolygon2D();
      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(polygonToTest4, polygon));

      ConvexPolygon2D polygonToTest5 = new ConvexPolygon2D();
      polygonToTest5.addVertex(new Point2D(-0.1, 0.1));
      polygonToTest5.update();
      assertFalse(ConvexPolygon2dCalculator.isPolygonInside(polygonToTest5, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testTranslatePolygon1()
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.addVertex(new Point2D(10.0, 0.0));
      polygon.addVertex(new Point2D(0.0, 10.0));
      polygon.update();

      Vector2D translation1 = new Vector2D(0.0, 0.0);
      ConvexPolygon2D polygon1 = polygon.translateCopy(translation1);
      assertTrue(polygon1.epsilonEquals(polygon, epsilon));

      Vector2D translation2 = new Vector2D(1.0, 0.5);
      ConvexPolygon2D polygon2 = polygon.translateCopy(translation2);
      assertTrue(polygon2.getVertex(2).epsilonEquals(new Point2D(1.0, 0.5), epsilon));
      assertTrue(polygon2.getVertex(1).epsilonEquals(new Point2D(11.0, 0.5), epsilon));
      assertTrue(polygon2.getVertex(0).epsilonEquals(new Point2D(1.0, 10.5), epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetMiddleIndexCounterClockwise1()
   {
      Random random = new Random(234);
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      while (polygon.getNumberOfVertices() < 6)
      {
         polygon.addVertex(EuclidCoreRandomTools.nextPoint2D(random));
         polygon.update();
      }
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

      polygon.clearAndUpdate();
      while (polygon.getNumberOfVertices() < 3)
      {
         polygon.addVertex(EuclidCoreRandomTools.nextPoint2D(random));
         polygon.update();
      }
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(0, 0, polygon), 2);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(1, 1, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(2, 2, polygon), 1);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(0, 1, polygon), 2);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(0, 2, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(1, 2, polygon), 0);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(1, 0, polygon), 1);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(2, 0, polygon), 1);
      assertIndexCorrect(ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(2, 1, polygon), 2);

      polygon.clearAndUpdate();
      while (polygon.getNumberOfVertices() < 1)
      {
         polygon.addVertex(EuclidCoreRandomTools.nextPoint2D(random));
         polygon.update();
      }
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

      ConvexPolygon2D polygon = new ConvexPolygon2D();
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
      ConvexPolygon2D polygon = new ConvexPolygon2D();

      Point2D observer1 = new Point2D(0.5, 0.5);
      assertPointsEqual(null, polygon.lineOfSightVertices(observer1), true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetLineOfSightVertexIndices1()
   {
      Point2D vertex = new Point2D(-0.5, 0.5);

      // point polygon
      ConvexPolygon2D polygon = new ConvexPolygon2D();
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
      ConvexPolygon2D polygon = new ConvexPolygon2D();
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
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(0.0, 1.0));
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.addVertex(new Point2D(1.5, 0.5));
      polygon.addVertex(new Point2D(1.0, 0.0));
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.update();

      Line2D line1 = new Line2D(new Point2D(0.0, 0.1), new Vector2D(1.0, 1.0));
      assertTrue(ConvexPolygon2dCalculator.doesLineIntersectEdge(line1, 0, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line1, 1, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line1, 2, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line1, 3, polygon));
      assertTrue(ConvexPolygon2dCalculator.doesLineIntersectEdge(line1, 4, polygon));

      Line2D line2 = new Line2D(new Point2D(0.9, 1.0), new Vector2D(1.0, -1.0));
      assertTrue(ConvexPolygon2dCalculator.doesLineIntersectEdge(line2, 0, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line2, 1, polygon));
      assertTrue(ConvexPolygon2dCalculator.doesLineIntersectEdge(line2, 2, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line2, 3, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line2, 4, polygon));

      Line2D line3 = new Line2D(new Point2D(0.2, 0.6), new Vector2D(1.0, 0.0));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line3, 0, polygon));
      assertTrue(ConvexPolygon2dCalculator.doesLineIntersectEdge(line3, 1, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line3, 2, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line3, 3, polygon));
      assertTrue(ConvexPolygon2dCalculator.doesLineIntersectEdge(line3, 4, polygon));

      Line2D line4 = new Line2D(new Point2D(0.0, -0.3), new Vector2D(0.0, 0.25));
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
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(0.0, 1.0));
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Line2D line1 = new Line2D(new Point2D(0.0, 0.3), new Vector2D(1.0, 0.0));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line1, 0, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line1, 1, polygon));

      Line2D line2 = new Line2D(new Point2D(0.0, 0.3), new Vector2D(0.0, 1.0));
      assertTrue(ConvexPolygon2dCalculator.doesLineIntersectEdge(line2, 0, polygon));
      assertTrue(ConvexPolygon2dCalculator.doesLineIntersectEdge(line2, 1, polygon));

      Line2D line3 = new Line2D(new Point2D(0.0, 0.3), new Vector2D(0.0, -1.0));
      assertTrue(ConvexPolygon2dCalculator.doesLineIntersectEdge(line3, 0, polygon));
      assertTrue(ConvexPolygon2dCalculator.doesLineIntersectEdge(line3, 1, polygon));

      Line2D line4 = new Line2D(new Point2D(2.0, 0.3), new Vector2D(0.0, -1.0));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line4, 0, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line4, 1, polygon));

      Line2D line5 = new Line2D(new Point2D(-1.0e-6, 0.3), new Vector2D(0.0, -1.0));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line5, 0, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line5, 1, polygon));

      Line2D line6 = new Line2D(new Point2D(0.0, 0.3), new Vector2D(1.0, 0.0));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line6, 0, polygon));
      assertFalse(ConvexPolygon2dCalculator.doesLineIntersectEdge(line6, 1, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testDoesLineIntersectEdge3()
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();

      Line2D line5 = new Line2D(new Point2D(0.0, 0.0), new Vector2D(1.0, 0.0));
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

      LineSegment2D edge1 = new LineSegment2D(vertex1, vertex2);
      LineSegment2D edge2 = new LineSegment2D(vertex2, vertex3);
      LineSegment2D edge3 = new LineSegment2D(vertex3, vertex4);
      LineSegment2D edge4 = new LineSegment2D(vertex4, vertex1);

      // add in order so update does not change indices:
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(vertex1);
      polygon.addVertex(vertex2);
      polygon.addVertex(vertex3);
      polygon.addVertex(vertex4);
      polygon.update();

      LineSegment2D result1 = new LineSegment2D();
      LineSegment2D result2 = new LineSegment2D();

      Line2D line1 = new Line2D(new Point2D(0.5, 0.5), new Vector2D(-1.0, 0.0));
      LineSegment2D[] expected1 = new LineSegment2D[] {edge4, edge2};
      assertEdgesEqual(expected1, ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line1, polygon), false);
      assertEquals(2, ConvexPolygon2dCalculator.getIntersectingEdges(line1, result1, result2, polygon));
      assertEdgesEqual(expected1, new LineSegment2D[] {result1, result2}, false);

      Line2D line2 = new Line2D(new Point2D(0.5, 1.5), new Vector2D(1.0, 0.0));
      LineSegment2D[] expected2 = null;
      assertEdgesEqual(expected2, ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line2, polygon), false);
      assertEquals(0, ConvexPolygon2dCalculator.getIntersectingEdges(line2, result1, result2, polygon));

      Line2D line3 = new Line2D(new Point2D(0.0, 2.0), new Vector2D(1.0, -1.0));
      LineSegment2D[] expected3 = new LineSegment2D[] {edge1};
      assertEdgesEqual(expected3, ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line3, polygon), false);
      assertEquals(1, ConvexPolygon2dCalculator.getIntersectingEdges(line3, result1, result2, polygon));
      assertEdgesEqual(expected3, new LineSegment2D[] {result1}, false);

      Line2D line4 = new Line2D(new Point2D(0.0, 0.0), new Vector2D(1.0, 1.0));
      LineSegment2D[] expected4 = new LineSegment2D[] {edge1, edge3};
      assertEdgesEqual(expected4, ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line4, polygon), false);
      assertEquals(2, ConvexPolygon2dCalculator.getIntersectingEdges(line4, result1, result2, polygon));
      assertEdgesEqual(expected4, new LineSegment2D[] {result1, result2}, false);

      Line2D line5 = new Line2D(new Point2D(-0.5, -0.5), new Vector2D(0.7, 0.7));
      LineSegment2D[] expected5 = new LineSegment2D[] {edge1, edge3};
      assertEdgesEqual(expected5, ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line5, polygon), false);
      assertEquals(2, ConvexPolygon2dCalculator.getIntersectingEdges(line5, result1, result2, polygon));
      assertEdgesEqual(expected5, new LineSegment2D[] {result1, result2}, false);

      Line2D line6 = new Line2D(new Point2D(0.0, -0.5), new Vector2D(0.0, 0.7));
      LineSegment2D[] expected6 = new LineSegment2D[] {edge1, edge3};
      assertEdgesEqual(expected6, ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line6, polygon), false);
      assertEquals(2, ConvexPolygon2dCalculator.getIntersectingEdges(line6, result1, result2, polygon));
      assertEdgesEqual(expected6, new LineSegment2D[] {result1, result2}, false);

      Line2D line7 = new Line2D(new Point2D(-0.5, 1.5), new Vector2D(1.0, -1.0));
      LineSegment2D[] expected7 = new LineSegment2D[] {edge2, edge4};
      assertEdgesEqual(expected7, ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line7, polygon), false);
      assertEquals(2, ConvexPolygon2dCalculator.getIntersectingEdges(line7, result1, result2, polygon));
      assertEdgesEqual(expected7, new LineSegment2D[] {result1, result2}, false);

      Line2D line8 = new Line2D(new Point2D(1.0, 0.5), new Vector2D(0.0, -0.2));
      LineSegment2D[] expected8 = new LineSegment2D[] {edge1, edge3};
      assertEdgesEqual(expected8, ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line8, polygon), false);
      assertEquals(2, ConvexPolygon2dCalculator.getIntersectingEdges(line8, result1, result2, polygon));
      assertEdgesEqual(expected8, new LineSegment2D[] {result1, result2}, false);

      Line2D line9 = new Line2D(new Point2D(-0.3, 1.0), new Vector2D(0.2, 0.0));
      LineSegment2D[] expected9 = new LineSegment2D[] {edge4, edge2};
      assertEdgesEqual(expected9, ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line9, polygon), false);
      assertEquals(2, ConvexPolygon2dCalculator.getIntersectingEdges(line9, result1, result2, polygon));
      assertEdgesEqual(expected9, new LineSegment2D[] {result1, result2}, false);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetIntersectingEdges2()
   {
      // line polygon
      Point2D vertex1 = new Point2D(1.0, 1.0);
      Point2D vertex2 = new Point2D(1.0, 0.0);

      LineSegment2D edge1 = new LineSegment2D(vertex1, vertex2);
      LineSegment2D edge2 = new LineSegment2D(vertex2, vertex1);

      // add in order so update does not change indices:
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(vertex1);
      polygon.addVertex(vertex2);
      polygon.update();

      LineSegment2D result1 = new LineSegment2D();
      LineSegment2D result2 = new LineSegment2D();

      Line2D line1 = new Line2D(new Point2D(0.5, 1.5), new Vector2D(0.0, 0.1));
      LineSegment2D[] expected1 = null;
      assertEdgesEqual(expected1, ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line1, polygon), false);
      assertEquals(0, ConvexPolygon2dCalculator.getIntersectingEdges(line1, result1, result2, polygon));

      Line2D line2 = new Line2D(new Point2D(-0.5, 0.0), new Vector2D(0.75, 0.25));
      LineSegment2D[] expected2 = new LineSegment2D[] {edge1, edge2};
      assertEdgesEqual(expected2, ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line2, polygon), false);
      assertEquals(2, ConvexPolygon2dCalculator.getIntersectingEdges(line2, result1, result2, polygon));
      assertEdgesEqual(expected2, new LineSegment2D[] {result1, result2}, false);

      Line2D line3 = new Line2D(new Point2D(1.0, -0.5), new Vector2D(0.0, 0.1));
      LineSegment2D[] expected3 = null;
      assertEdgesEqual(expected3, ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line3, polygon), false);
      assertEquals(0, ConvexPolygon2dCalculator.getIntersectingEdges(line3, result1, result2, polygon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetIntersectingEdges3()
   {
      // point polygon
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(-1.0, -0.5));
      polygon.update();

      LineSegment2D result1 = new LineSegment2D();
      LineSegment2D result2 = new LineSegment2D();

      Line2D line1 = new Line2D(new Point2D(0.0, 0.0), new Vector2D(-0.5, -0.25));
      assertEquals(ConvexPolygon2dCalculator.getIntersectingEdges(line1, result1, result2, polygon), 0);
      assertTrue(ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line1, polygon) == null);

      Line2D line2 = new Line2D(new Point2D(0.5, 1.5), new Vector2D(0.0, 0.1));
      assertEquals(ConvexPolygon2dCalculator.getIntersectingEdges(line2, result1, result2, polygon), 0);
      assertTrue(ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line2, polygon) == null);

      Line2D line3 = new Line2D(new Point2D(-1.0, -0.5), new Vector2D(1.0, 0.1));
      assertEquals(ConvexPolygon2dCalculator.getIntersectingEdges(line3, result1, result2, polygon), 0);
      assertTrue(ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line3, polygon) == null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetIntersectingEdges4()
   {
      // empty polygon
      ConvexPolygon2D polygon = new ConvexPolygon2D();

      LineSegment2D result1 = new LineSegment2D();
      LineSegment2D result2 = new LineSegment2D();

      Line2D line1 = new Line2D(new Point2D(0.5, 1.5), new Vector2D(0.0, 0.1));
      assertEquals(ConvexPolygon2dCalculator.getIntersectingEdges(line1, result1, result2, polygon), 0);
      assertTrue(ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line1, polygon) == null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetEdgeNormal()
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
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

   private static void assertEdgesEqual(LineSegment2D[] expected, LineSegment2D[] actual, boolean enforceOrder)
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

   private static void assertEdgesEqual(LineSegment2D expected, LineSegment2D actual)
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
