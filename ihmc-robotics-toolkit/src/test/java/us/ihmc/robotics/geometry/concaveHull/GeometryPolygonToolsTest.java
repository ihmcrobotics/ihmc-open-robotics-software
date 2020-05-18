package us.ihmc.robotics.geometry.concaveHull;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;
import us.ihmc.robotics.geometry.concavePolygon2D.GeometryPolygonTools;

import java.util.ArrayList;
import java.util.List;

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
   }
}
