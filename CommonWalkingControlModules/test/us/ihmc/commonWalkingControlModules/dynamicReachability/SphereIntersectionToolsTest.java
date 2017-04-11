package us.ihmc.commonWalkingControlModules.dynamicReachability;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;

import java.util.Random;

import static org.junit.Assert.*;

public class SphereIntersectionToolsTest
{
   private static final double epsilon = 0.000001;
   private static final int iters = 1000;

   @ContinuousIntegrationTest(estimatedDuration = 2.0)
   @Test(timeout = 2000)
   public void testFlatCenterComputation()
   {
      Random random = new Random();

      for (int i = 0; i < iters; i++)
      {
         double radius1 = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 5.0);
         double radius2 = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 5.0);

         double sphereSeparation = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, radius1 + radius2);

         double distanceToIntersection = SphereIntersectionTools.computeDistanceToIntersectingPlane(sphereSeparation, radius1, radius2);
         double distanceToEllipseCenter = SphereIntersectionTools.computeDistanceToCenterOfIntersectionEllipse(sphereSeparation, 0.0, radius1, radius2);
         double distanceToEllipseNearEdge = SphereIntersectionTools.computeDistanceToNearEdgeOfIntersectionEllipse(sphereSeparation, 0.0, radius1, radius2);
         double distanceToEllipseFarEdge = SphereIntersectionTools.computeDistanceToFarEdgeOfIntersectionEllipse(sphereSeparation, 0.0, radius1, radius2);

         assertEquals(distanceToIntersection, distanceToEllipseCenter, epsilon);
         assertEquals(distanceToIntersection, distanceToEllipseNearEdge, epsilon);
         assertEquals(distanceToIntersection, distanceToEllipseFarEdge, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 2.0)
   @Test(timeout = 2000)
   public void testComputation()
   {
      Random random = new Random();

      for (int i = 0; i < iters; i ++)
      {
         double radius1 = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 5.0);
         double radius2 = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 5.0);

         double sphereSeparationZ = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0);
         double sphereSeparationX = EuclidCoreRandomTools
               .generateRandomDouble(random, 0.0, Math.sqrt(Math.pow(radius1 + radius2, 2.0) - Math.pow(sphereSeparationZ, 2.0)));

         double sphereSeparation = Math.sqrt(Math.pow(sphereSeparationX, 2.0) + Math.pow(sphereSeparationZ, 2.0));
         double angleOfSeparation = Math.atan(sphereSeparationZ / sphereSeparationX);

         double distanceToIntersection = SphereIntersectionTools.computeDistanceToIntersectingPlane(sphereSeparation, radius1, radius2);

         double distanceToEllipseCenter = SphereIntersectionTools
               .computeDistanceToCenterOfIntersectionEllipse(sphereSeparationX, sphereSeparationZ, radius1, radius2);
         double distanceShouldBe = Math.cos(angleOfSeparation) * distanceToIntersection;

         assertEquals(distanceShouldBe, distanceToEllipseCenter, epsilon);

         double majorAxisCalculated = Math.sqrt(Math.pow(radius1, 2.0) - Math.pow(distanceToIntersection, 2.0));
         double majorAxisCalculatedAlt = Math.sqrt(Math.pow(radius2, 2.0) - Math.pow((sphereSeparation - distanceToIntersection), 2.0));
         assertEquals(majorAxisCalculated, majorAxisCalculatedAlt, epsilon);

         double majorAxis = SphereIntersectionTools.computeRadiusOfIntersectingPlane(sphereSeparation, radius1, radius2);
         assertEquals(majorAxisCalculated, majorAxis, epsilon);

         double majorAxisAlt = SphereIntersectionTools.computeEllipseMajorAxisRadius(radius1, distanceToIntersection);
         assertEquals(majorAxisAlt, majorAxis, epsilon);

         double minorAxisCalculated = majorAxis * Math.sin(angleOfSeparation);
         double minorAxis = SphereIntersectionTools.computeEllipseMinorAxisRadius(radius1, distanceToIntersection, angleOfSeparation);
         double minorAxisShouldBe = SphereIntersectionTools.computeEllipseMinorAxisRadius(majorAxis, angleOfSeparation);
         assertEquals(minorAxisCalculated, minorAxis, epsilon);
         assertEquals(minorAxis, minorAxisShouldBe, epsilon);

         double distanceToNearEdge = SphereIntersectionTools
               .computeDistanceToNearEdgeOfIntersectionEllipse(sphereSeparationX, sphereSeparationZ, radius1, radius2);
         double nearEdgeShouldBe = distanceToEllipseCenter - minorAxis;
         assertEquals(distanceToNearEdge, nearEdgeShouldBe, epsilon);

         double distanceToFarEdge = SphereIntersectionTools.computeDistanceToFarEdgeOfIntersectionEllipse(sphereSeparationX, sphereSeparationZ, radius1, radius2);
         double farEdgeShouldBe = distanceToEllipseCenter + minorAxis;
         assertEquals(distanceToFarEdge, farEdgeShouldBe, epsilon);
         assertEquals(nearEdgeShouldBe + 2 * minorAxis, farEdgeShouldBe, epsilon);
         assertEquals(distanceToNearEdge + 2 * minorAxis, distanceToFarEdge, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 2.0)
   @Test(timeout = 2000)
   public void testRadiusOfIntersection()
   {
      Random random = new Random();

      for (int i = 0; i < iters; i++)
      {
         double radius1 = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 5.0);
         double radius2 = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 5.0);

         double sphereSeparation = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, radius1 + radius2);

         double distanceToIntersection = SphereIntersectionTools.computeDistanceToIntersectingPlane(sphereSeparation, radius1, radius2);

         double ellipseMajorAxis = SphereIntersectionTools.computeEllipseMajorAxisRadius(radius1, distanceToIntersection);
         double planeRadius = SphereIntersectionTools.computeRadiusOfIntersectingPlane(sphereSeparation, radius1, radius2);

         assertEquals(ellipseMajorAxis, planeRadius, epsilon);
      }
   }
}