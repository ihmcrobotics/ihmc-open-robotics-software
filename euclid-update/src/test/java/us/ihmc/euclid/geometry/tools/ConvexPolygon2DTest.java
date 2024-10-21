package us.ihmc.euclid.geometry.tools;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

import static org.junit.jupiter.api.Assertions.*;

public class ConvexPolygon2DTest
{
   private static final double epsilon = 1e-7;


   @Test
   public void testTranslatePolygon1()
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.addVertex(new Point2D(10.0, 0.0));
      polygon.addVertex(new Point2D(0.0, 10.0));
      polygon.update();

      Vector2D translation1 = new Vector2D(0.0, 0.0);
      ConvexPolygon2DBasics polygon1 = polygon.translateCopy(translation1);
      assertTrue(polygon1.epsilonEquals(polygon, epsilon));

      Vector2D translation2 = new Vector2D(1.0, 0.5);
      ConvexPolygon2DBasics polygon2 = polygon.translateCopy(translation2);
      assertTrue(polygon2.getVertex(2).epsilonEquals(new Point2D(1.0, 0.5), epsilon));
      assertTrue(polygon2.getVertex(1).epsilonEquals(new Point2D(11.0, 0.5), epsilon));
      assertTrue(polygon2.getVertex(0).epsilonEquals(new Point2D(1.0, 10.5), epsilon));
   }
}
