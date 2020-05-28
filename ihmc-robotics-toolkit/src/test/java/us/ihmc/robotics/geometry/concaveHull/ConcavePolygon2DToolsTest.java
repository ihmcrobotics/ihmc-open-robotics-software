package us.ihmc.robotics.geometry.concaveHull;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.Assert.*;

public class ConcavePolygon2DToolsTest
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

      ConcavePolygon2D concaveHull = new ConcavePolygon2D();
      concaveHull.addVertex(-1.0, 1.0);
      concaveHull.addVertex(1.0, 1.0);
      concaveHull.addVertex(1.0, 0.5);
      concaveHull.addVertex(2.0, 0.5);
      concaveHull.addVertex(2.0, -0.5);
      concaveHull.addVertex(1.0, -0.5);
      concaveHull.addVertex(1.0, -1.0);
      concaveHull.addVertex(-1.0, -1.0);
      concaveHull.update();

      double totalArea = polygon1.getArea() + polygon2.getArea();
      Point2D totalCentroid = new Point2D();
      Point2D scaledCentroid1 = new Point2D();
      Point2D scaledCentroid2 = new Point2D();
      scaledCentroid1.set(polygon1.getCentroid());
      scaledCentroid1.scale(polygon1.getArea() / totalArea);
      scaledCentroid2.set(polygon2.getCentroid());
      scaledCentroid2.scale(polygon2.getArea() / totalArea);
      totalCentroid.add(scaledCentroid1, scaledCentroid2);

      assertEquals(totalArea, concaveHull.getArea(), 1e-7);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(totalCentroid, concaveHull.getCentroid(), 1e-7);
   }

   @Test
   public void testInsertVertex()
   {
      ConcavePolygon2D polygon = new ConcavePolygon2D();
      polygon.addVertex(-1.0, 1.0);
      polygon.addVertex(1.0, 1.0);
      polygon.addVertex(1.0, -1.0);
      polygon.addVertex(-1.0, -1.0);
      polygon.update();

      double area = 4.0;
      assertEquals(4, polygon.getNumberOfVertices());
      assertEquals(area, polygon.getArea(), 1e-7);

      polygon.insertVertex(1, 0.0, 0.9);
      assertFalse(polygon.isUpToDate());

      polygon.update();

      area = area - 1.0 * 0.1;
      assertTrue(polygon.isUpToDate());
      assertEquals(5, polygon.getNumberOfVertices());
      assertEquals(area, polygon.getArea(), 1e-7);

      polygon.removeVertex(1);
      assertFalse(polygon.isUpToDate());

      polygon.update();

      area = 4.0;
      assertTrue(polygon.isUpToDate());
      assertEquals(4, polygon.getNumberOfVertices());
      assertEquals(area, polygon.getArea(), 1e-7);

      polygon.insertVertex(1, 0.0, 0.0);
      assertFalse(polygon.isUpToDate());

      polygon.update();

      assertTrue(polygon.isUpToDate());
      assertEquals(5, polygon.getNumberOfVertices());
      assertEquals(3.0, polygon.getArea(), 1e-7);
   }

   @Test
   public void testEpsilonEquals()
   {
      ConcavePolygon2D polygonA = new ConcavePolygon2D();
      polygonA.addVertex(1.0, 1.0);
      polygonA.addVertex(1.0, -1.0);
      polygonA.addVertex(-1.0, -1.0);
      polygonA.update();

      ConcavePolygon2D polygonB = new ConcavePolygon2D();
      polygonB.addVertex(1.0, 1.0);
      polygonB.addVertex(1.0, -1.0);
      polygonB.addVertex(-1.0, -1.0);
      polygonB.update();

      assertTrue(polygonA.epsilonEquals(polygonB, 1e-7));

      polygonB.clear();
      polygonB.addVertex(0.5, 0.5);
      polygonB.addVertex(0.5, -0.5);
      polygonB.addVertex(-0.5, -0.5);
      polygonB.update();

      assertFalse(polygonA.epsilonEquals(polygonB, 1e-7));
   }

}
