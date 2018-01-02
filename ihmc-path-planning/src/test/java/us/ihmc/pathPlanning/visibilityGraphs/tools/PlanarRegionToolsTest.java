package us.ihmc.pathPlanning.visibilityGraphs.tools;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextDouble;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.PlanarRegion;

public class PlanarRegionToolsTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;

   @Test(timeout = 10000)
   public void testTruncatePlanarRegionIfIntersectingWithPlane() throws Exception
   {
      Point3D groundOrigin = new Point3D();
      Vector3D groundNormal = new Vector3D(0.0, 0.0, 1.0);

      Point3D squareOrigin = new Point3D(0.0, 0.0, -0.001);
      Vector3D squareNormal = new Vector3D(0.0, -1.0, 0.0);
      AxisAngle squareOrientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(squareNormal);
      RigidBodyTransform squarePose = new RigidBodyTransform(squareOrientation, squareOrigin);

      double squareSide = 4.0;

      Point2D[] concaveHullVertices = {new Point2D(0.0, 0.0), new Point2D(0.0, squareSide), new Point2D(squareSide, squareSide), new Point2D(squareSide, 0.0)};
      List<ConvexPolygon2D> convexPolygons = new ArrayList<>();
      convexPolygons.add(new ConvexPolygon2D(concaveHullVertices));
      PlanarRegion verticalSquare = new PlanarRegion(squarePose, concaveHullVertices, convexPolygons);

      Point3D[] expectedVerticesInWorld = Arrays.stream(concaveHullVertices).map(p -> toWorld(p, squarePose)).toArray(Point3D[]::new);
      expectedVerticesInWorld[0].addZ(0.001);
      expectedVerticesInWorld[3].addZ(0.001);

      PlanarRegion truncatedSquare = PlanarRegionTools.truncatePlanarRegionIfIntersectingWithPlane(groundOrigin, groundNormal, verticalSquare, 0.05, null);
      RigidBodyTransform truncatedTransform = new RigidBodyTransform();
      truncatedSquare.getTransformToWorld(truncatedTransform);
      EuclidCoreTestTools.assertRigidBodyTransformGeometricallyEquals(squarePose, truncatedTransform, EPSILON);

      Point3D[] actualVerticesInWorld = Arrays.stream(truncatedSquare.getConcaveHull()).map(p -> toWorld(p, squarePose)).toArray(Point3D[]::new);

      assertEquals(expectedVerticesInWorld.length, actualVerticesInWorld.length);

      for (int i = 0; i < expectedVerticesInWorld.length; i++)
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedVerticesInWorld[i], actualVerticesInWorld[i], EPSILON);
   }

   public static Point3D toWorld(Point2D point2D, Transform transformToWorld)
   {
      Point3D inWorld = new Point3D(point2D);
      transformToWorld.transform(inWorld);
      return inWorld;
   }

   @Test(timeout = 10000)
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

         Point2DReadOnly[] convexPolygon2DArray = convexPolygon2D.subList(0, hullSize).toArray(new Point2DReadOnly[convexPolygon2D.size()]);

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
         assertFalse(PlanarRegionTools.isPointInsidePolygon(convexPolygon2DArray, outsidePoint));

         double alphaInside = nextDouble(random, 0.0, 1.0);
         Point2D insidePoint = new Point2D();
         insidePoint.interpolate(centroid, pointOnEdge, alphaInside);
         assertTrue(PlanarRegionTools.isPointInsidePolygon(convexPolygon2DArray, insidePoint));
      }
   }

   @Test(timeout = 10000)
   public void testIsInsidePolygonBug1() throws Exception
   {
      Point2D[] polygon = {new Point2D(-0.3, 0.5), new Point2D(0.3, 0.5), new Point2D(0.3, -0.5), new Point2D(-0.3, -0.5)};
      Point2D pointToCheck = new Point2D(-2.0, 0.5);

      assertFalse(PlanarRegionTools.isPointInsidePolygon(polygon, pointToCheck));
   }
}