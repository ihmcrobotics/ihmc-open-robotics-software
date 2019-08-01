package us.ihmc.robotics.geometry;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.log.LogTools;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

public class ConcavePolygonToolsTest
{
   @Test
   public void testCutSimpleConvexPolygonAbove()
   {
      // create simple convex polygon
      ConvexPolygon2D size2square0center = new ConvexPolygon2D();
      size2square0center.addVertex(1.0, 1.0);
      size2square0center.addVertex(1.0, -1.0);
      size2square0center.addVertex(-1.0, -1.0);
      size2square0center.addVertex(-1.0, 1.0);
      size2square0center.update();

      LogTools.info("{}", size2square0center.getVertex(0));

      // create line and up direction
      Line2D yAxis = new Line2D(0.0, 0.0, 0.0, 1.0);
      Vector2D xDirection = new Vector2D(1.0, 0.0);

      // cut it above a line
      List<ConvexPolygon2D> result = ConcavePolygonTools.cropPolygonToAboveLine(size2square0center, yAxis, xDirection);

      assertEquals(1, result.size(), "supposed to cut");

      // create the ideal result
      ConvexPolygon2D aboveYAxisRectangle = new ConvexPolygon2D();
      aboveYAxisRectangle.addVertex(1.0, 1.0);
      aboveYAxisRectangle.addVertex(1.0, -1.0);
      aboveYAxisRectangle.addVertex(0.0, -1.0);
      aboveYAxisRectangle.addVertex(0.0, 1.0);
      aboveYAxisRectangle.update();

      // assert equal
      assertTrue(result.get(0).epsilonEquals(aboveYAxisRectangle, 1e-7));
   }

   @Test
   public void testCutSimpleConvexPolygonBelow()
   {
      // create simple convex polygon
      ConvexPolygon2D size2square0center = new ConvexPolygon2D();
      size2square0center.addVertex(1.0, 1.0);
      size2square0center.addVertex(1.0, -1.0);
      size2square0center.addVertex(-1.0, -1.0);
      size2square0center.addVertex(-1.0, 1.0);
      size2square0center.update();

      // create line and up direction
      Line2D yAxis = new Line2D(0.0, 0.0, 0.0, 1.0);
      Vector2D negativeXDirection = new Vector2D(-1.0, 0.0);

      // cut it above a line
      List<ConvexPolygon2D> result = ConcavePolygonTools.cropPolygonToAboveLine(size2square0center, yAxis, negativeXDirection);

      assertEquals(1, result.size(), "supposed to cut");

      // create the ideal result
      ConvexPolygon2D expected = new ConvexPolygon2D();
      expected.addVertex(-1.0, 1.0);
      expected.addVertex(0.0, 1.0);
      expected.addVertex(0.0, -1.0);
      expected.addVertex(-1.0, -1.0);
      expected.update();

      // assert equal
      assertTrue(result.get(0).epsilonEquals(expected, 1e-7));
   }

   @Test
   public void testKeepSideSimpleConvexPolygonAbove1()
   {
      // create simple convex polygon
      ConvexPolygon2D size2square0center = new ConvexPolygon2D();
      size2square0center.addVertex(1.0, 1.0);
      size2square0center.addVertex(1.0, -1.0);
      size2square0center.addVertex(-1.0, -1.0);
      size2square0center.addVertex(-1.0, 1.0);
      size2square0center.update();

      LogTools.info("{}", size2square0center.getVertex(0));

      // create line and up direction
      Line2D yAxis = new Line2D(1.0, 0.0, 0.0, 1.0);
      Vector2D xDirection = new Vector2D(1.0, 0.0);

      // cut it above a line
      List<ConvexPolygon2D> result = ConcavePolygonTools.cropPolygonToAboveLine(size2square0center, yAxis, xDirection);

      assertEquals(1, result.size(), "supposed to cut");
      LogTools.debug("{}", result.get(0));

      // create the ideal result
      ConvexPolygon2D expected = new ConvexPolygon2D();
      expected.addVertex(1.0, 1.0);
      expected.addVertex(1.0, -1.0);
      expected.update();

      // assert equal
      assertTrue(result.get(0).epsilonEquals(expected, 1e-7));
   }


   @Test
   public void testRemoveAllSimpleConvexPolygonAbove2()
   {
      // create simple convex polygon
      ConvexPolygon2D size2square0center = new ConvexPolygon2D();
      size2square0center.addVertex(1.0, 1.0);
      size2square0center.addVertex(1.0, -1.0);
      size2square0center.addVertex(-1.0, -1.0);
      size2square0center.addVertex(-1.0, 1.0);
      size2square0center.update();

      LogTools.info("{}", size2square0center.getVertex(0));

      // create line and up direction
      Line2D yAxis = new Line2D(2.0, 0.0, 0.0, 1.0);
      Vector2D xDirection = new Vector2D(1.0, 0.0);

      // cut it above a line
      List<ConvexPolygon2D> result = ConcavePolygonTools.cropPolygonToAboveLine(size2square0center, yAxis, xDirection);

      assertEquals(0, result.size(), "supposed to cut");
   }

   @Test
   public void testKeepAllSimpleConvexPolygonAbove1()
   {
      // create simple convex polygon
      ConvexPolygon2D size2square0center = new ConvexPolygon2D();
      size2square0center.addVertex(1.0, 1.0);
      size2square0center.addVertex(1.0, -1.0);
      size2square0center.addVertex(-1.0, -1.0);
      size2square0center.addVertex(-1.0, 1.0);
      size2square0center.update();

      LogTools.info("{}", size2square0center.getVertex(0));

      // create line and up direction
      Line2D yAxis = new Line2D(1.0, 0.0, 0.0, 1.0);
      Vector2D xDirection = new Vector2D(-1.0, 0.0);

      // cut it above a line
      List<ConvexPolygon2D> result = ConcavePolygonTools.cropPolygonToAboveLine(size2square0center, yAxis, xDirection);

      assertEquals(1, result.size(), "supposed to cut");
      LogTools.debug("{}", result.get(0));

      // create the ideal result
      ConvexPolygon2D expected = new ConvexPolygon2D();
      expected.addVertex(1.0, 1.0);
      expected.addVertex(1.0, -1.0);
      expected.addVertex(-1.0, -1.0);
      expected.addVertex(-1.0, 1.0);
      expected.update();

      // assert equal
      assertTrue(result.get(0).epsilonEquals(expected, 1e-7));
   }

   @Test
   public void testKeepAllSimpleConvexPolygonAbove2()
   {
      // create simple convex polygon
      ConvexPolygon2D size2square0center = new ConvexPolygon2D();
      size2square0center.addVertex(1.0, 1.0);
      size2square0center.addVertex(1.0, -1.0);
      size2square0center.addVertex(-1.0, -1.0);
      size2square0center.addVertex(-1.0, 1.0);
      size2square0center.update();

      LogTools.info("{}", size2square0center.getVertex(0));

      // create line and up direction
      Line2D yAxis = new Line2D(2.0, 0.0, 0.0, 1.0);
      Vector2D xDirection = new Vector2D(-1.0, 0.0);

      // cut it above a line
      List<ConvexPolygon2D> result = ConcavePolygonTools.cropPolygonToAboveLine(size2square0center, yAxis, xDirection);

      assertEquals(1, result.size(), "supposed to cut");

      // create the ideal result
      ConvexPolygon2D expected = new ConvexPolygon2D();
      expected.addVertex(1.0, 1.0);
      expected.addVertex(1.0, -1.0);
      expected.addVertex(-1.0, -1.0);
      expected.addVertex(-1.0, 1.0);
      expected.update();

      // assert equal
      assertTrue(result.get(0).epsilonEquals(expected, 1e-7));
   }
}
