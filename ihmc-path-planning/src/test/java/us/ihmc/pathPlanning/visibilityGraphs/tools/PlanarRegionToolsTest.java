package us.ihmc.pathPlanning.visibilityGraphs.tools;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextDouble;

import java.util.Collections;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class PlanarRegionToolsTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testIsInsidePolygon() throws Exception
   {
      Random random = new Random(324534L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with convex polygon
         List<? extends Point2DReadOnly> convexPolygon2D = EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random, 10.0, 10.0, 100);
         int hullSize = EuclidGeometryPolygonTools.inPlaceGiftWrapConvexHull2D(convexPolygon2D);
         boolean clockwiseOrdered = random.nextBoolean();
         if (!clockwiseOrdered)
            Collections.reverse(convexPolygon2D.subList(0, hullSize));

         Point2D[] convexPolygon2DArray = convexPolygon2D.subList(0, hullSize).toArray(new Point2D[convexPolygon2D.size()]);

         Point2D centroid = new Point2D();
         EuclidGeometryPolygonTools.computeConvexPolyong2DArea(convexPolygon2D, hullSize, clockwiseOrdered, centroid);
         int vertexIndex = random.nextInt(hullSize);
         int nextVertexIndex = EuclidGeometryPolygonTools.next(vertexIndex, hullSize);
         Point2DReadOnly vertex = convexPolygon2D.get(vertexIndex);
         Point2DReadOnly nextVertex = convexPolygon2D.get(nextVertexIndex);


         Point2D pointOnEdge = new Point2D();
         pointOnEdge.interpolate(vertex, nextVertex, random.nextDouble());

         double alphaOutside = nextDouble(random, 1.0, 3.0);
         Point2D outsidePoint = new Point2D();
         outsidePoint.interpolate(centroid, pointOnEdge, alphaOutside);

         Vector2D directionToCentroid = new Vector2D(centroid.getX() - outsidePoint.getX(), centroid.getY() - outsidePoint.getY());
         directionToCentroid.normalize();
         directionToCentroid.scale(10);
         Point2D endPoint = new Point2D(outsidePoint.getX() + directionToCentroid.getX(), outsidePoint.getY() + directionToCentroid.getY());

         assertFalse(PlanarRegionTools.isPointInsidePolygon(convexPolygon2DArray, outsidePoint, endPoint));

         double alphaInside = nextDouble(random, 0.0, 1.0);
         Point2D insidePoint = new Point2D();
         insidePoint.interpolate(centroid, pointOnEdge, alphaInside);

         directionToCentroid = new Vector2D(centroid.getX() - outsidePoint.getX(), centroid.getY() - outsidePoint.getY());
         directionToCentroid.normalize();
         directionToCentroid.scale(10);
         endPoint = new Point2D(outsidePoint.getX() + directionToCentroid.getX(), outsidePoint.getY() + directionToCentroid.getY());

         assertTrue(PlanarRegionTools.isPointInsidePolygon(convexPolygon2DArray, insidePoint, endPoint));
      }
   }
}