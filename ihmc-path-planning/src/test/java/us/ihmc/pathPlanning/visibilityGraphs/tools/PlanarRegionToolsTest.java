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
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
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

      PlanarRegion truncatedSquare = PlanarRegionTools.truncatePlanarRegionIfIntersectingWithPlane(groundOrigin, groundNormal, verticalSquare, 0.05, 0, 0);
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

   @Test(timeout = 10000)
   public void testDoRay2DAndLineSegment2DIntersect() throws Exception
   {
      Random random = new Random(116L);

      // Trivial test by positioning the intersection on the ray
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D rayOrigin = EuclidCoreRandomTools.nextPoint2D(random);
         rayOrigin.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector2D rayDirection = EuclidCoreRandomTools.nextVector2D(random);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D pointOnRay = new Point2D();
         pointOnRay.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), rayDirection, rayOrigin);

         Vector2D lineSegmentDirection = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);

         Point2D lineSegmentStart = new Point2D();
         Point2D lineSegmentEnd = new Point2D();

         double alphaStart, alphaEnd;

         // Expecting intersection
         alphaStart = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         alphaEnd = EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0);
         lineSegmentStart.scaleAdd(alphaStart, lineSegmentDirection, pointOnRay);
         lineSegmentEnd.scaleAdd(alphaEnd, lineSegmentDirection, pointOnRay);
         assertTrue("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertTrue("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));

         // Not expecting intersection
         alphaStart = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         alphaEnd = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         lineSegmentStart.scaleAdd(alphaStart, lineSegmentDirection, pointOnRay);
         lineSegmentEnd.scaleAdd(alphaEnd, lineSegmentDirection, pointOnRay);
         assertFalse("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertFalse("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
      }

      // Trivial test by positioning the intersection on the line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random);

         Point2D intersection = new Point2D();
         intersection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         Vector2D rayDirection = EuclidCoreRandomTools.nextVector2D(random);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D rayOrigin = new Point2D();

         // Expecting intersection
         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), rayDirection, intersection);
         assertTrue("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertTrue("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));

         // Not expecting intersection
         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), rayDirection, intersection);
         assertFalse("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertFalse("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
      }

      // Test intersection at the ray origin
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D rayOrigin = EuclidCoreRandomTools.nextPoint2D(random);
         rayOrigin.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector2D rayDirection = EuclidCoreRandomTools.nextVector2D(random);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Vector2D lineSegmentDirection = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);

         Point2D lineSegmentStart = new Point2D();
         Point2D lineSegmentEnd = new Point2D();

         // Expecting intersection
         lineSegmentStart.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineSegmentDirection, rayOrigin);
         lineSegmentEnd.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineSegmentDirection, rayOrigin);
         assertTrue("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertTrue("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
      }

      // Test intersection at an endpoint of the line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random);

         Point2D intersection = new Point2D(lineSegmentStart);

         Vector2D rayDirection = EuclidCoreRandomTools.nextVector2D(random);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D rayOrigin = new Point2D();

         // Expecting intersection
         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), rayDirection, intersection);
         assertTrue("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertTrue("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));

         // Not expecting intersection
         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0), rayDirection, intersection);
         assertFalse("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertFalse("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
      }

      // Test with parallel/collinear ray and line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D rayOrigin = EuclidCoreRandomTools.nextPoint2D(random);
         rayOrigin.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector2D rayDirection = EuclidCoreRandomTools.nextVector2D(random);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D lineSegmentStart = new Point2D();
         Point2D lineSegmentEnd = new Point2D();

         double alpha1 = EuclidCoreRandomTools.nextDouble(random, 2.0);
         double alpha2 = EuclidCoreRandomTools.nextDouble(random, 2.0);

         // Make the second line segment collinear to the ray
         lineSegmentStart.scaleAdd(alpha1, rayDirection, rayOrigin);
         lineSegmentEnd.scaleAdd(alpha2, rayDirection, rayOrigin);
         Vector2D lineSegmentDirection = new Vector2D();
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);

         if (0.0 < alpha1 || 0.0 < alpha2 || alpha1 * alpha2 < 0.0)
         {
            assertTrue("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
            assertTrue("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
         }
         else
         {
            assertFalse("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
            assertFalse("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
         }

         // Shift the second line segment such that it becomes only parallel to the ray.
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(rayDirection, rayOrigin);
         orthogonal.set(-orthogonal.getY(), orthogonal.getX());
         orthogonal.normalize();

         double distance = EuclidCoreRandomTools.nextDouble(random, 1.0e-10, 10.0);
         lineSegmentStart.scaleAdd(distance, orthogonal, lineSegmentStart);
         lineSegmentEnd.scaleAdd(distance, orthogonal, lineSegmentEnd);
         assertFalse("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertFalse("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
      }

      // Test with vertical parallel/collinear ray and line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         double x = EuclidCoreRandomTools.nextDouble(random, 10.0);
         Point2D rayOrigin = new Point2D(x, EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector2D rayDirection = new Vector2D(x, EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D lineSegmentStart = new Point2D();
         Point2D lineSegmentEnd = new Point2D();

         double alpha1 = EuclidCoreRandomTools.nextDouble(random, 2.0);
         double alpha2 = EuclidCoreRandomTools.nextDouble(random, 2.0);

         // Make the second line segment collinear to the ray
         lineSegmentStart.scaleAdd(alpha1, rayDirection, rayOrigin);
         lineSegmentEnd.scaleAdd(alpha2, rayDirection, rayOrigin);

         if (0.0 < alpha1 || 0.0 < alpha2 || alpha1 * alpha2 < 0.0)
         {
            assertTrue("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
            assertTrue("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
         }
         else
         {
            assertFalse("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
            assertFalse("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
         }

         // Shift the second line segment such that it becomes only parallel to the ray.
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(rayDirection, rayOrigin);
         orthogonal.set(-orthogonal.getY(), orthogonal.getX());
         orthogonal.normalize();

         double distance = EuclidCoreRandomTools.nextDouble(random, 1.0e-10, 10.0);
         lineSegmentStart.scaleAdd(distance, orthogonal, lineSegmentStart);
         lineSegmentEnd.scaleAdd(distance, orthogonal, lineSegmentEnd);
         assertFalse("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertFalse("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
      }

      // Test with horizontal parallel/collinear line segments
      for (int i = 0; i < ITERATIONS; i++)
      {
         double y = EuclidCoreRandomTools.nextDouble(random, 10.0);
         Point2D rayOrigin = new Point2D(EuclidCoreRandomTools.nextDouble(random, 10.0), y);
         Vector2D rayDirection = new Vector2D(EuclidCoreRandomTools.nextDouble(random, 10.0), y);

         Point2D lineSegmentStart = new Point2D();
         Point2D lineSegmentEnd = new Point2D();

         double alpha1 = EuclidCoreRandomTools.nextDouble(random, 2.0);
         double alpha2 = EuclidCoreRandomTools.nextDouble(random, 2.0);

         // Make the second line segment collinear to the ray
         lineSegmentStart.scaleAdd(alpha1, rayDirection, rayOrigin);
         lineSegmentEnd.scaleAdd(alpha2, rayDirection, rayOrigin);

         if (0.0 < alpha1 || 0.0 < alpha2 || alpha1 * alpha2 < 0.0)
         {
            assertTrue("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
            assertTrue("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
         }
         else
         {
            assertFalse("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
            assertFalse("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
         }

         // Shift the second line segment such that it becomes only parallel to the ray.
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(rayDirection, rayOrigin);
         orthogonal.set(-orthogonal.getY(), orthogonal.getX());
         orthogonal.normalize();

         double distance = EuclidCoreRandomTools.nextDouble(random, 1.0e-10, 10.0);
         lineSegmentStart.scaleAdd(distance, orthogonal, lineSegmentStart);
         lineSegmentEnd.scaleAdd(distance, orthogonal, lineSegmentEnd);
         assertFalse("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertFalse("Iteration: " + i, PlanarRegionTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
      }
   }

   @Test(timeout = 10000)
   public void testIntersectionBetweenRay2DAndLineSegment2D() throws Exception
   {
      Random random = new Random(3242L);

      // Trivial test by positioning the intersection on the ray
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D rayOrigin = EuclidCoreRandomTools.nextPoint2D(random);
         rayOrigin.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector2D rayDirection = EuclidCoreRandomTools.nextVector2D(random);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), rayDirection, rayOrigin);

         Vector2D lineSegmentDirection = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);

         Point2D lineSegmentStart = new Point2D();
         Point2D lineSegmentEnd = new Point2D();

         double alphaStart, alphaEnd;

         // Expecting intersection
         alphaStart = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         alphaEnd = EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0);
         lineSegmentStart.scaleAdd(alphaStart, lineSegmentDirection, expectedIntersection);
         lineSegmentEnd.scaleAdd(alphaEnd, lineSegmentDirection, expectedIntersection);
         assertAllCombinationsOfTwoLineSegmentsIntersection(expectedIntersection, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);

         // Not expecting intersection
         alphaStart = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         alphaEnd = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         lineSegmentStart.scaleAdd(alphaStart, lineSegmentDirection, expectedIntersection);
         lineSegmentEnd.scaleAdd(alphaEnd, lineSegmentDirection, expectedIntersection);
         assertOnlyExistenceOfIntersectionBetweenRay2DAndAllCombinationsOfLineSegment(false, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
      }

      // Trivial test by positioning the intersection on the line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random);

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         Vector2D rayDirection = EuclidCoreRandomTools.nextVector2D(random);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D rayOrigin = new Point2D();

         // Expecting intersection
         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), rayDirection, expectedIntersection);
         assertAllCombinationsOfTwoLineSegmentsIntersection(expectedIntersection, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);

         // Not expecting intersection
         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), rayDirection, expectedIntersection);
         assertOnlyExistenceOfIntersectionBetweenRay2DAndAllCombinationsOfLineSegment(false, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
      }

      // Test intersection at the ray origin
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D rayOrigin = EuclidCoreRandomTools.nextPoint2D(random);
         rayOrigin.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector2D rayDirection = EuclidCoreRandomTools.nextVector2D(random);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Vector2D lineSegmentDirection = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);

         Point2D lineSegmentStart = new Point2D();
         Point2D lineSegmentEnd = new Point2D();

         // Expecting intersection
         lineSegmentStart.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineSegmentDirection, rayOrigin);
         lineSegmentEnd.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineSegmentDirection, rayOrigin);
         assertAllCombinationsOfTwoLineSegmentsIntersection(rayOrigin, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
      }

      // Test intersection at an endpoint of the line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random);

         Point2D expectedIntersection = new Point2D(lineSegmentStart);

         Vector2D rayDirection = EuclidCoreRandomTools.nextVector2D(random);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D rayOrigin = new Point2D();

         // Expecting intersection
         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), rayDirection, expectedIntersection);
         assertAllCombinationsOfTwoLineSegmentsIntersection(expectedIntersection, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);

         // Not expecting intersection
         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0), rayDirection, expectedIntersection);
         assertOnlyExistenceOfIntersectionBetweenRay2DAndAllCombinationsOfLineSegment(false, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
      }

      // Test with parallel/collinear ray and line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D rayOrigin = EuclidCoreRandomTools.nextPoint2D(random);
         rayOrigin.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector2D rayDirection = EuclidCoreRandomTools.nextVector2D(random);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D lineSegmentStart = new Point2D();
         Point2D lineSegmentEnd = new Point2D();

         double alpha1 = EuclidCoreRandomTools.nextDouble(random, 2.0);
         double alpha2 = EuclidCoreRandomTools.nextDouble(random, 2.0);

         // Make the second line segment collinear to the ray
         lineSegmentStart.scaleAdd(alpha1, rayDirection, rayOrigin);
         lineSegmentEnd.scaleAdd(alpha2, rayDirection, rayOrigin);
         Vector2D lineSegmentDirection = new Vector2D();
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);

         if (0.0 < alpha1 || 0.0 < alpha2 || alpha1 * alpha2 < 0.0)
         {
            assertOnlyExistenceOfIntersectionBetweenRay2DAndAllCombinationsOfLineSegment(true, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
         }
         else
         {
            assertOnlyExistenceOfIntersectionBetweenRay2DAndAllCombinationsOfLineSegment(false, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
         }

         // Shift the second line segment such that it becomes only parallel to the ray.
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(rayDirection, rayOrigin);
         orthogonal.set(-orthogonal.getY(), orthogonal.getX());
         orthogonal.normalize();

         double distance = EuclidCoreRandomTools.nextDouble(random, 1.0e-10, 10.0);
         lineSegmentStart.scaleAdd(distance, orthogonal, lineSegmentStart);
         lineSegmentEnd.scaleAdd(distance, orthogonal, lineSegmentEnd);
         assertOnlyExistenceOfIntersectionBetweenRay2DAndAllCombinationsOfLineSegment(false, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
      }

      // Test with vertical parallel/collinear ray and line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         double x = EuclidCoreRandomTools.nextDouble(random, 10.0);
         Point2D rayOrigin = new Point2D(x, EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector2D rayDirection = new Vector2D(x, EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D lineSegmentStart = new Point2D();
         Point2D lineSegmentEnd = new Point2D();

         double alpha1 = EuclidCoreRandomTools.nextDouble(random, 2.0);
         double alpha2 = EuclidCoreRandomTools.nextDouble(random, 2.0);

         // Make the second line segment collinear to the ray
         lineSegmentStart.scaleAdd(alpha1, rayDirection, rayOrigin);
         lineSegmentEnd.scaleAdd(alpha2, rayDirection, rayOrigin);

         if (0.0 < alpha1 || 0.0 < alpha2 || alpha1 * alpha2 < 0.0)
         {
            assertOnlyExistenceOfIntersectionBetweenRay2DAndAllCombinationsOfLineSegment(true, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
         }
         else
         {
            assertOnlyExistenceOfIntersectionBetweenRay2DAndAllCombinationsOfLineSegment(false, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
         }

         // Shift the second line segment such that it becomes only parallel to the ray.
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(rayDirection, rayOrigin);
         orthogonal.set(-orthogonal.getY(), orthogonal.getX());
         orthogonal.normalize();

         double distance = EuclidCoreRandomTools.nextDouble(random, 1.0e-10, 10.0);
         lineSegmentStart.scaleAdd(distance, orthogonal, lineSegmentStart);
         lineSegmentEnd.scaleAdd(distance, orthogonal, lineSegmentEnd);
         assertOnlyExistenceOfIntersectionBetweenRay2DAndAllCombinationsOfLineSegment(false, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
      }

      // Test with horizontal parallel/collinear line segments
      for (int i = 0; i < ITERATIONS; i++)
      {
         double y = EuclidCoreRandomTools.nextDouble(random, 10.0);
         Point2D rayOrigin = new Point2D(EuclidCoreRandomTools.nextDouble(random, 10.0), y);
         Vector2D rayDirection = new Vector2D(EuclidCoreRandomTools.nextDouble(random, 10.0), y);

         Point2D lineSegmentStart = new Point2D();
         Point2D lineSegmentEnd = new Point2D();

         double alpha1 = EuclidCoreRandomTools.nextDouble(random, 2.0);
         double alpha2 = EuclidCoreRandomTools.nextDouble(random, 2.0);

         // Make the second line segment collinear to the ray
         lineSegmentStart.scaleAdd(alpha1, rayDirection, rayOrigin);
         lineSegmentEnd.scaleAdd(alpha2, rayDirection, rayOrigin);

         if (0.0 < alpha1 || 0.0 < alpha2 || alpha1 * alpha2 < 0.0)
         {
            assertOnlyExistenceOfIntersectionBetweenRay2DAndAllCombinationsOfLineSegment(true, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
         }
         else
         {
            assertOnlyExistenceOfIntersectionBetweenRay2DAndAllCombinationsOfLineSegment(false, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
         }

         // Shift the second line segment such that it becomes only parallel to the ray.
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(rayDirection, rayOrigin);
         orthogonal.set(-orthogonal.getY(), orthogonal.getX());
         orthogonal.normalize();

         double distance = EuclidCoreRandomTools.nextDouble(random, 1.0e-10, 10.0);
         lineSegmentStart.scaleAdd(distance, orthogonal, lineSegmentStart);
         lineSegmentEnd.scaleAdd(distance, orthogonal, lineSegmentEnd);
         assertOnlyExistenceOfIntersectionBetweenRay2DAndAllCombinationsOfLineSegment(false, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
      }

      // Test with various overlapping with collinear line segments
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D rayOrigin = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Vector2D rayDirection = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);

         // Making four points ordered on the line
         Point2D front1 = new Point2D();
         Point2D front2 = new Point2D();
         Point2D back1 = new Point2D();
         Point2D back2 = new Point2D();

         front1.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), rayDirection, rayOrigin);
         front2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), rayDirection, rayOrigin);
         back1.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), rayDirection, rayOrigin);
         back2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), rayDirection, rayOrigin);

         boolean success;
         Point2D expectedIntersection = new Point2D();
         Point2D actualIntersection = new Point2D();

         // Line segment fully in front of ray
         expectedIntersection.set(front1);
         success = PlanarRegionTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, front1, front2, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);

         expectedIntersection.set(front2);
         success = PlanarRegionTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, front2, front1, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);

         // Line segment partially in front of ray
         expectedIntersection.set(front1);
         success = PlanarRegionTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, back1, front1, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);

         expectedIntersection.set(front2);
         success = PlanarRegionTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, front2, back1, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);
      }
   }

   private void assertOnlyExistenceOfIntersectionBetweenRay2DAndAllCombinationsOfLineSegment(boolean intersectionExist, Point2D rayOrigin,
                                                                                             Vector2D rayDirection, Point2D lineSegmentStart,
                                                                                             Point2D lineSegmentEnd)
   {
      boolean success;
      Point2D actualIntersection = new Point2D();

      success = PlanarRegionTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd, new Point2D());
      assertTrue(success == intersectionExist);
      actualIntersection = PlanarRegionTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
      assertTrue(actualIntersection != null == intersectionExist);

      success = PlanarRegionTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart, new Point2D());
      assertTrue(success == intersectionExist);
      actualIntersection = PlanarRegionTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart);
      assertTrue(actualIntersection != null == intersectionExist);
   }

   private void assertAllCombinationsOfTwoLineSegmentsIntersection(Point2D expectedIntersection, Point2D rayOrigin, Vector2D rayDirection,
                                                                   Point2D lineSegmentStart, Point2D lineSegmentEnd)
   {
      double epsilon = EuclidGeometryTools.ONE_TRILLIONTH;

      Vector2D direction1 = new Vector2D();
      direction1.sub(rayDirection, rayOrigin);
      Vector2D direction2 = new Vector2D();
      Point2D lss2 = lineSegmentStart;
      Point2D lse2 = lineSegmentEnd;
      direction2.sub(lse2, lss2);

      if (Math.abs(rayDirection.dot(direction2)) > 1.0 - 0.0001)
         epsilon = 1.0e-10;

      boolean success;
      Point2D actualIntersection = new Point2D();

      success = PlanarRegionTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, lss2, lse2, actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      success = PlanarRegionTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, lse2, lss2, actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

      actualIntersection = PlanarRegionTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, lss2, lse2);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      actualIntersection = PlanarRegionTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, lse2, lss2);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
   }
}