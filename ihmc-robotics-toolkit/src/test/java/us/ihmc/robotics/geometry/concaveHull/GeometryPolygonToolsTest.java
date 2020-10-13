package us.ihmc.robotics.geometry.concaveHull;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;
import us.ihmc.robotics.geometry.concavePolygon2D.GeometryPolygonTools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class GeometryPolygonToolsTest
{
   @Test
   public void testConcavePolygonAreaAndCentroid()
   {
      ConvexPolygon2D polygon1 = new ConvexPolygon2D();
      ConvexPolygon2D polygon2 = new ConvexPolygon2D();

      polygon1.addVertex(-1.0, 1.0);
      polygon1.addVertex(-1.0, -1.0);
      polygon1.addVertex(1.0, 1.0);
      polygon1.addVertex(1.0, -1.0);
      polygon1.update();

      polygon2.addVertex(1.0, 0.5);
      polygon2.addVertex(1.0, -0.5);
      polygon2.addVertex(2.0, -0.5);
      polygon2.addVertex(2.0, 0.5);
      polygon2.update();

      List<Point2DReadOnly> concaveHull = new ArrayList<>();
      concaveHull.add(new Point2D(-1.0, 1.0));
      concaveHull.add(new Point2D(1.0, 1.0));
      concaveHull.add(new Point2D(1.0, 0.5));
      concaveHull.add(new Point2D(2.0, 0.5));
      concaveHull.add(new Point2D(2.0, -0.5));
      concaveHull.add(new Point2D(1.0, -0.5));
      concaveHull.add(new Point2D(1.0, -1.0));
      concaveHull.add(new Point2D(-1.0, -1.0));

      double totalArea = polygon1.getArea() + polygon2.getArea();
      Point2D totalCentroid = new Point2D();
      Point2D scaledCentroid1 = new Point2D();
      Point2D scaledCentroid2 = new Point2D();
      scaledCentroid1.set(polygon1.getCentroid());
      scaledCentroid1.scale(polygon1.getArea() / totalArea);
      scaledCentroid2.set(polygon2.getCentroid());
      scaledCentroid2.scale(polygon2.getArea() / totalArea);
      totalCentroid.add(scaledCentroid1, scaledCentroid2);

      Point2D centroid = new Point2D();
      double actualArea = EuclidGeometryPolygonTools.computeConvexPolygon2DArea(concaveHull, concaveHull.size(), true, centroid);

      assertEquals(totalArea, actualArea, 1e-7);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(totalCentroid, centroid, 1e-7);
   }

   @Test
   public void testPolygonInsideOtherPolygonTricky()
   {
      ConcavePolygon2D uPolygon = new ConcavePolygon2D();
      uPolygon.addVertex(-1.0, 1.0);
      uPolygon.addVertex(-0.9, 1.0);
      uPolygon.addVertex(-0.9, -0.9);
      uPolygon.addVertex(0.9, -0.9);
      uPolygon.addVertex(0.9, 1.0);
      uPolygon.addVertex(1.0, 1.0);
      uPolygon.addVertex(1.0, -1.0);
      uPolygon.addVertex(-1.0, -1.0);
      uPolygon.update();

      ConcavePolygon2D hat = new ConcavePolygon2D();
      hat.addVertex(-1.0, 1.0);
      hat.addVertex(1.0, 1.0);
      hat.addVertex(1.0, 0.9);
      hat.addVertex(-1.0, 0.9);
      hat.update();

      assertFalse(GeometryPolygonTools.isPolygonInsideOtherPolygon(hat, uPolygon));
   }

   @Test
   public void testTrickyIsPointInside()
   {
      ConcavePolygon2D uPolygon = new ConcavePolygon2D();
      uPolygon.addVertex(-1.0, 1.0);
      uPolygon.addVertex(-0.9, 1.0);
      uPolygon.addVertex(-0.9, -0.9);
      uPolygon.addVertex(0.9, -0.9);
      uPolygon.addVertex(0.9, 1.0);
      uPolygon.addVertex(1.0, 1.0);
      uPolygon.addVertex(1.0, -1.0);
      uPolygon.addVertex(-1.0, -1.0);
      uPolygon.update();

      assertFalse(uPolygon.isPointInside(0.0, 1.0));

      assertFalse(uPolygon.isPointInside(-0.8995, 1.0));
      assertFalse(uPolygon.isPointInsideEpsilon(-0.8995, 1.0, 1e-5));
      //assertTrue(uPolygon.isPointInsideEpsilon(-0.8995, 1.0, 5e-4));

      Point2D pointToTest = new Point2D(-0.8995000000000001, 1.0);
      assertFalse(GeometryPolygonTools.isPoint2DInsideSimplePolygon2D(pointToTest, uPolygon.getVertexBufferView(), uPolygon.getNumberOfVertices(), 1e-7));
   }

   @Test
   public void testTrickyIsPointInside2()
   {
      ConcavePolygon2D polygon = new ConcavePolygon2D();
      polygon.addVertex(1.5, 1.5);
      polygon.addVertex(1.5, 0.5);
      polygon.addVertex(1.0, 0.5);
      polygon.addVertex(1.0, -1.0);
      polygon.addVertex(-1.0, -1.0);
      polygon.addVertex(-1.0, 1.0);
      polygon.addVertex(0.5, 1.0);
      polygon.addVertex(0.5, 1.5);
      polygon.update();

      assertTrue(polygon.isPointInside(0.5, 0.5));
   }

   @Test
   public void testEasyInteriorPolygon()
   {
      ConcavePolygon2D outerPolygon = new ConcavePolygon2D();
      ConcavePolygon2D innerPolygon = new ConcavePolygon2D();

      outerPolygon.addVertex(-1.0, 1.0);
      outerPolygon.addVertex(1.0, 1.0);
      outerPolygon.addVertex(1.0, -1.0);
      outerPolygon.addVertex(-1.0, -1.0);
      outerPolygon.update();

      innerPolygon.addVertex(-0.5, 0.5);
      innerPolygon.addVertex(0.5, 0.5);
      innerPolygon.addVertex(0.5, -0.5);
      innerPolygon.addVertex(-0.5, -0.5);
      innerPolygon.update();

      assertTrue(GeometryPolygonTools.isPolygonInsideOtherPolygon(innerPolygon, outerPolygon));

      innerPolygon.clear();
      innerPolygon.addVertex(-0.5, 0.5);
      innerPolygon.addVertex(1.5, 0.5);
      innerPolygon.addVertex(1.5, -0.5);
      innerPolygon.addVertex(-0.5, -0.5);
      innerPolygon.update();

      assertFalse(GeometryPolygonTools.isPolygonInsideOtherPolygon(innerPolygon, outerPolygon));
   }

   @Test
   public void testIsPolygonInsideOtherPolygon()
   {
      ConcavePolygon2D polygon1 = new ConcavePolygon2D();
      polygon1.addVertex(-0.1, 1.0);
      polygon1.addVertex(0.1, 1.0);
      polygon1.addVertex(0.1, -1.0);
      polygon1.addVertex(-0.1, -1.0);
      polygon1.update();

      ConcavePolygon2D polygon2 = new ConcavePolygon2D();
      polygon2.addVertex(0.9, 1.0);
      polygon2.addVertex(1.1, 1.0);
      polygon2.addVertex(1.1, -1.0);
      polygon2.addVertex(0.9, -1.0);
      polygon2.update();

      assertFalse(GeometryPolygonTools.isPolygonInsideOtherPolygon(polygon1, polygon2));
      assertFalse(GeometryPolygonTools.isPolygonInsideOtherPolygon(polygon2, polygon1));

   }

   @Test
   public void testIsPointInside()
   {
      ConvexPolygon2D convexPolygon = new ConvexPolygon2D();
      convexPolygon.addVertex(-0.1, 1.0);
      convexPolygon.addVertex(0.0, 1.1);
      convexPolygon.addVertex(0.1, 1.0);
      convexPolygon.addVertex(0.1, -1.0);
      convexPolygon.addVertex(0.0, -1.1);
      convexPolygon.addVertex(-0.1, -1.0);
      convexPolygon.update();

      ConcavePolygon2D concavePolygon = new ConcavePolygon2D(convexPolygon);
      ConcavePolygon2D concavePolygonOther = new ConcavePolygon2D();
      concavePolygonOther.addVertex(-0.1, 1.0);
      concavePolygonOther.addVertex(0.0, 1.1);
      concavePolygonOther.addVertex(0.1, 1.0);
      concavePolygonOther.addVertex(0.1, -1.0);
      concavePolygonOther.addVertex(0.0, -1.1);
      concavePolygonOther.addVertex(-0.1, -1.0);
      concavePolygonOther.update();


      assertTrue(convexPolygon.isPointInside(0.08, -1.0));
      assertTrue(concavePolygonOther.isPointInside(0.08, -1.0));

      // test interior points
      Random random = new Random(1738L);
      for (int i = 0; i < 10000; i++)
      {
         Point2DReadOnly interiorPoint = getRandomInteriorPoint(random, convexPolygon);
         assertTrue(convexPolygon.isPointInside(interiorPoint));
         assertTrue(concavePolygon.isPointInside(interiorPoint));
         assertTrue(concavePolygonOther.isPointInside(interiorPoint));
         assertTrue(GeometryPolygonTools.isPoint2DInsideSimplePolygon2D(interiorPoint, concavePolygon.getVertexBufferView(), concavePolygon.getNumberOfVertices()));
      }

      // test point on edge
      for (int i = 0; i < convexPolygon.getNumberOfVertices(); i++)
      {
         for (int j = 0; j < 100; j++)
         {
            Point2DReadOnly vertex = convexPolygon.getVertex(i);
            Point2DReadOnly nextVertex = convexPolygon.getNextVertex(i);

            Point2D edgePoint = new Point2D();
            edgePoint.interpolate(vertex, nextVertex, j / 100);

            assertTrue(convexPolygon.isPointInside(edgePoint));
            assertTrue(concavePolygon.isPointInside(edgePoint));
            assertTrue(concavePolygonOther.isPointInside(edgePoint));

            assertTrue(GeometryPolygonTools.isPoint2DInsideSimplePolygon2D(edgePoint, concavePolygon.getVertexBufferView(), concavePolygon.getNumberOfVertices()));
         }
      }
   }

   private static Point2DReadOnly getRandomInteriorPoint(Random random, ConvexPolygon2DReadOnly polygon2D)
   {
      int numberOfPoints = polygon2D.getNumberOfVertices();
      double maxAlpha = 1.0;
      Point2D randomPoint = new Point2D();
      for (int i = 0; i < numberOfPoints; i++)
      {
         double alpha = RandomNumbers.nextDouble(random, 0.0, maxAlpha);
         maxAlpha -= alpha;

         randomPoint.scaleAdd(alpha, polygon2D.getVertex(i), randomPoint);
      }

      return randomPoint;
   }


   @Test
   public void doPolygonIntersectTest()
   {
      ConcavePolygon2D polygonToClip = new ConcavePolygon2D();
      polygonToClip.addVertex(-1.0, 1.0);
      polygonToClip.addVertex(1.0, 1.0);
      polygonToClip.addVertex(1.0, -1.0);
      polygonToClip.addVertex(-1.0, -1.0);
      polygonToClip.update();

      assertTrue(polygonToClip.isPointInside(0.5, 0.5));

      ConcavePolygon2D clippingPolygon = new ConcavePolygon2D();
      clippingPolygon.addVertex(1.0 + -0.5, 0.5);
      clippingPolygon.addVertex(1.0 + 0.5, 0.5);
      clippingPolygon.addVertex(1.0 + 0.5, -0.5);
      clippingPolygon.addVertex(1.0 + -0.5, -0.5);
      clippingPolygon.update();

      assertTrue(polygonToClip.isPointInside(clippingPolygon.getVertex(0)));
      assertFalse(polygonToClip.isPointInside(clippingPolygon.getVertex(1)));
      assertFalse(polygonToClip.isPointInside(clippingPolygon.getVertex(2)));
      assertTrue(polygonToClip.isPointInside(clippingPolygon.getVertex(3)));

      ConcavePolygon2D polygonToClipUnmodified = new ConcavePolygon2D(polygonToClip);
      ConcavePolygon2D clippingPolygonUnmodified = new ConcavePolygon2D(clippingPolygon);

      assertTrue(GeometryPolygonTools.doPolygonsIntersect(polygonToClip, clippingPolygon));
      assertTrue(polygonToClip.epsilonEquals(polygonToClipUnmodified, 1e-8));
      assertTrue(clippingPolygon.epsilonEquals(clippingPolygonUnmodified, 1e-8));
   }

   @Test
   public void testIsPoint2DInsideSimplePolygon2D()
   {
      ConcavePolygon2D polygon = new ConcavePolygon2D();
      polygon.addVertex(1.5, 1.5);
      polygon.addVertex(0.0, 0.0);
      polygon.addVertex(-1.5, 1.5);
      polygon.update();

      assertTrue(GeometryPolygonTools.isPoint2DInsideSimplePolygon2D(1.0, 1.0, polygon.getVertexBufferView(), 3));
      assertTrue(GeometryPolygonTools.isPoint2DInsideSimplePolygon2D(-1.0, 1.0, polygon.getVertexBufferView(), 3));

      ConcavePolygon2D polygon1 = new ConcavePolygon2D();
      polygon1.addVertex(-0.1, 1.0);
      polygon1.addVertex(0.1, 1.0);
      polygon1.addVertex(0.1, -1.0);
      polygon1.addVertex(-0.1, -1.0);
      polygon1.update();

      ConcavePolygon2D polygon2 = new ConcavePolygon2D();
      polygon2.addVertex(0.9, 1.0);
      polygon2.addVertex(1.1, 1.0);
      polygon2.addVertex(1.1, -1.0);
      polygon2.addVertex(0.9, -1.0);
      polygon2.update();

      for (int i = 0; i < 4; i++)
      {
         assertFalse(GeometryPolygonTools.isPoint2DInsideSimplePolygon2D(polygon1.getVertex(i), polygon2.getVertexBufferView(), 4));
         assertFalse(GeometryPolygonTools.isPoint2DInsideSimplePolygon2D(polygon2.getVertex(i), polygon1.getVertexBufferView(), 4));
      }
   }

   @Test
   public void testNastyPointInsideBug()
   {
      ConcavePolygon2D polygon1 = new ConcavePolygon2D();
      polygon1.addVertex(-0.1, 1.0);
      polygon1.addVertex(0.1, 1.0);
      polygon1.addVertex(0.1, -1.0);
      polygon1.addVertex(-0.1, -1.0);
      polygon1.update();

      assertFalse(GeometryPolygonTools.isPoint2DInsideSimplePolygon2D(-0.3, 1.0, polygon1.getVertexBufferView(), 4));
   }

   @Test
   public void testPointInside()
   {
      ConcavePolygon2D polygon2 = new ConcavePolygon2D();
      polygon2.addVertex(-1.0, 1.5);
      polygon2.addVertex(1.0, 1.5);
      polygon2.addVertex(0.0, 0.5);
      polygon2.update();

      ConvexPolygon2D convexPolygon2D = new ConvexPolygon2D(polygon2);

      assertTrue(convexPolygon2D.isPointInside(0.499, 1.0));
      assertTrue(polygon2.isPointInside(0.499, 1.0));
      assertFalse(convexPolygon2D.isPointInside(0.501, 1.0));
      assertFalse(polygon2.isPointInside(0.501, 1.0));
   }
}
