package us.ihmc.pathPlanning.visibilityGraphs.tools;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;
import static us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityGraphsGeometryTools.intersectionBetweenRay2DAndCircle2D;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

public class VisibilityGraphsGeometryToolsTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;
   private static final double LARGE_EPSILON = 1.0e-11;

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
         assertTrue("Iteration: " + i,
                    VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertTrue("Iteration: " + i,
                    VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));

         // Not expecting intersection
         alphaStart = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         alphaEnd = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         lineSegmentStart.scaleAdd(alphaStart, lineSegmentDirection, pointOnRay);
         lineSegmentEnd.scaleAdd(alphaEnd, lineSegmentDirection, pointOnRay);
         assertFalse("Iteration: " + i,
                     VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertFalse("Iteration: " + i,
                     VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
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
         assertTrue("Iteration: " + i,
                    VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertTrue("Iteration: " + i,
                    VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));

         // Not expecting intersection
         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), rayDirection, intersection);
         assertFalse("Iteration: " + i,
                     VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertFalse("Iteration: " + i,
                     VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
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
         assertTrue("Iteration: " + i,
                    VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertTrue("Iteration: " + i,
                    VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
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
         assertTrue("Iteration: " + i,
                    VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertTrue("Iteration: " + i,
                    VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));

         // Not expecting intersection
         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0), rayDirection, intersection);
         assertFalse("Iteration: " + i,
                     VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertFalse("Iteration: " + i,
                     VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
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
            assertTrue("Iteration: " + i,
                       VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
            assertTrue("Iteration: " + i,
                       VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
         }
         else
         {
            assertFalse("Iteration: " + i,
                        VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
            assertFalse("Iteration: " + i,
                        VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
         }

         // Shift the second line segment such that it becomes only parallel to the ray.
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(rayDirection, rayOrigin);
         orthogonal.set(-orthogonal.getY(), orthogonal.getX());
         orthogonal.normalize();

         double distance = EuclidCoreRandomTools.nextDouble(random, 1.0e-10, 10.0);
         lineSegmentStart.scaleAdd(distance, orthogonal, lineSegmentStart);
         lineSegmentEnd.scaleAdd(distance, orthogonal, lineSegmentEnd);
         assertFalse("Iteration: " + i,
                     VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertFalse("Iteration: " + i,
                     VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
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
            assertTrue("Iteration: " + i,
                       VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
            assertTrue("Iteration: " + i,
                       VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
         }
         else
         {
            assertFalse("Iteration: " + i,
                        VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
            assertFalse("Iteration: " + i,
                        VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
         }

         // Shift the second line segment such that it becomes only parallel to the ray.
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(rayDirection, rayOrigin);
         orthogonal.set(-orthogonal.getY(), orthogonal.getX());
         orthogonal.normalize();

         double distance = EuclidCoreRandomTools.nextDouble(random, 1.0e-10, 10.0);
         lineSegmentStart.scaleAdd(distance, orthogonal, lineSegmentStart);
         lineSegmentEnd.scaleAdd(distance, orthogonal, lineSegmentEnd);
         assertFalse("Iteration: " + i,
                     VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertFalse("Iteration: " + i,
                     VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
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
            assertTrue("Iteration: " + i,
                       VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
            assertTrue("Iteration: " + i,
                       VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
         }
         else
         {
            assertFalse("Iteration: " + i,
                        VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
            assertFalse("Iteration: " + i,
                        VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
         }

         // Shift the second line segment such that it becomes only parallel to the ray.
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(rayDirection, rayOrigin);
         orthogonal.set(-orthogonal.getY(), orthogonal.getX());
         orthogonal.normalize();

         double distance = EuclidCoreRandomTools.nextDouble(random, 1.0e-10, 10.0);
         lineSegmentStart.scaleAdd(distance, orthogonal, lineSegmentStart);
         lineSegmentEnd.scaleAdd(distance, orthogonal, lineSegmentEnd);
         assertFalse("Iteration: " + i,
                     VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertFalse("Iteration: " + i,
                     VisibilityGraphsGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
      }
   }

   @Test(timeout = 10000)
   public void testIntersectionBetweenRay2DAndCircle2D() throws Exception
   {
      Random random = new Random(456467);

      try
      { // Assert that the method throws an exception for negative radius
         Point2DReadOnly rayOrigin = EuclidCoreRandomTools.nextPoint2D(random);
         Vector2DReadOnly rayDirection = EuclidCoreRandomTools.nextVector2D(random);
         Point2DReadOnly circleCenter = EuclidCoreRandomTools.nextPoint2D(random);
         double circleRadius = -0.001;
         VisibilityGraphsGeometryTools.intersectionBetweenRay2DAndCircle2D(rayOrigin, rayDirection, circleCenter, circleRadius, null, null);
         fail("Should have thrown a " + IllegalArgumentException.class.getSimpleName());
      }
      catch (IllegalArgumentException e)
      {
         // Good
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Non-intersecting examples
         Point2D circleCenter = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         double circleRadius = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);

         Vector2D fromCenterToCircle = new Vector2D(circleRadius, 0.0);
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), fromCenterToCircle, fromCenterToCircle);
         Point2D pointOnCircle = new Point2D();
         pointOnCircle.add(fromCenterToCircle, circleCenter);
         Vector2D tangent = EuclidGeometryTools.perpendicularVector2D(fromCenterToCircle);
         if (random.nextBoolean())
            tangent.negate();

         Point2D rayOrigin = new Point2D();
         Vector2D rayDirection = new Vector2D();
         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), fromCenterToCircle, pointOnCircle);
         rayDirection.interpolate(fromCenterToCircle, tangent, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         Point2D firstIntersection = new Point2D();
         Point2D secondIntersection = new Point2D();

         int expectedNumberOfIntersections = 0;
         int actualNumberOfIntersections = intersectionBetweenRay2DAndCircle2D(rayOrigin, rayDirection, circleCenter, circleRadius, firstIntersection,
                                                                               secondIntersection);
         assertEquals(expectedNumberOfIntersections, actualNumberOfIntersections);
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(firstIntersection);
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(secondIntersection);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Intersecting examples with only one intersection
         Point2D circleCenter = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         double circleRadius = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);

         Vector2D fromCenterToCircle = new Vector2D(circleRadius, 0.0);
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), fromCenterToCircle, fromCenterToCircle);
         Point2D expectedIntersection = new Point2D();
         expectedIntersection.add(fromCenterToCircle, circleCenter);

         Point2D rayOrigin = new Point2D();
         Vector2D rayDirection = new Vector2D();
         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0), fromCenterToCircle, circleCenter);
         rayDirection.setAndScale(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), fromCenterToCircle);

         Point2D actualFirstIntersection = new Point2D();
         Point2D actualSecondIntersection = new Point2D();

         int expectedNumberOfIntersections = 1;
         int actualNumberOfIntersections = intersectionBetweenRay2DAndCircle2D(rayOrigin, rayDirection, circleCenter, circleRadius, actualFirstIntersection,
                                                                               actualSecondIntersection);
         assertEquals(expectedNumberOfIntersections, actualNumberOfIntersections);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualFirstIntersection, EPSILON);
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(actualSecondIntersection);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Intersecting examples with two intersections
         Point2D circleCenter = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         double circleRadius = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);

         Vector2D fromCenterToCircle = new Vector2D(circleRadius, 0.0);

         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), fromCenterToCircle, fromCenterToCircle);
         Point2D expectedFirstIntersection = new Point2D();
         expectedFirstIntersection.add(fromCenterToCircle, circleCenter);

         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), fromCenterToCircle, fromCenterToCircle);
         Point2D expectedSecondIntersection = new Point2D();
         expectedSecondIntersection.add(fromCenterToCircle, circleCenter);

         Point2D rayOrigin = new Point2D();
         Vector2D rayDirection = new Vector2D();
         rayOrigin.interpolate(expectedFirstIntersection, expectedSecondIntersection, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
         rayDirection.sub(expectedSecondIntersection, expectedFirstIntersection);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));

         Point2D actualFirstIntersection = new Point2D();
         Point2D actualSecondIntersection = new Point2D();

         int expectedNumberOfIntersections = 2;
         int actualNumberOfIntersections = intersectionBetweenRay2DAndCircle2D(rayOrigin, rayDirection, circleCenter, circleRadius, actualFirstIntersection,
                                                                               actualSecondIntersection);
         assertEquals("Iteration: " + i, expectedNumberOfIntersections, actualNumberOfIntersections);
         EuclidCoreTestTools.assertTuple2DEquals("Iteration: " + i, expectedFirstIntersection, actualFirstIntersection, LARGE_EPSILON);
         EuclidCoreTestTools.assertTuple2DEquals("Iteration: " + i, expectedSecondIntersection, actualSecondIntersection, LARGE_EPSILON);
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
         success = VisibilityGraphsGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, front1, front2, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);

         expectedIntersection.set(front2);
         success = VisibilityGraphsGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, front2, front1, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);

         // Line segment partially in front of ray
         expectedIntersection.set(front1);
         success = VisibilityGraphsGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, back1, front1, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);

         expectedIntersection.set(front2);
         success = VisibilityGraphsGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, front2, back1, actualIntersection);
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

      success = VisibilityGraphsGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd,
                                                                                       new Point2D());
      assertTrue(success == intersectionExist);
      actualIntersection = VisibilityGraphsGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
      assertTrue(actualIntersection != null == intersectionExist);

      success = VisibilityGraphsGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart,
                                                                                       new Point2D());
      assertTrue(success == intersectionExist);
      actualIntersection = VisibilityGraphsGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart);
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

      success = VisibilityGraphsGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, lss2, lse2, actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      success = VisibilityGraphsGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, lse2, lss2, actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

      actualIntersection = VisibilityGraphsGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, lss2, lse2);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      actualIntersection = VisibilityGraphsGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, lse2, lss2);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
   }

}
