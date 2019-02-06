package us.ihmc.pathPlanning.visibilityGraphs.postProcessing;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class PointWigglerTest
{
   private static final long timeout = 30000 * 100;
   private static final double epsilon = 1e-10;
   private static final int iters = 500;

   @Test
   public void testAverageDistanceVectorCalculationEasy()
   {
      Point2D pointAtOrigin = new Point2D();

      // evenly spaced at 90 degrees
      Point2D topLeft = new Point2D(0.5, 0.5);
      Point2D topRight = new Point2D(0.5, -0.5);
      Point2D bottomLeft = new Point2D(-0.5, 0.5);
      Point2D bottomRight = new Point2D(-0.5, -0.5);

      List<Point2DReadOnly> pointsToAvoidByDistance = new ArrayList<>();
      pointsToAvoidByDistance.add(topLeft);
      pointsToAvoidByDistance.add(topRight);
      pointsToAvoidByDistance.add(bottomLeft);
      pointsToAvoidByDistance.add(bottomRight);

      Vector2DReadOnly calculatedVector = PointWiggler
            .computeBestShiftVectorToAvoidPoints(pointAtOrigin, pointsToAvoidByDistance, 1.0, 0.0);
      Vector2DReadOnly expectedVector = new Vector2D();

      EuclidCoreTestTools.assertVector2DGeometricallyEquals(expectedVector, calculatedVector, epsilon);

      // evenly spaced at
      pointsToAvoidByDistance.clear();
      pointsToAvoidByDistance.add(new Point2D(0.0, 0.5));
      pointsToAvoidByDistance.add(new Point2D(0.5 * Math.sin(Math.PI / 3.0), -0.5 * Math.cos(Math.PI / 3.0)));
      pointsToAvoidByDistance.add(new Point2D(-0.5 * Math.sin(Math.PI / 3.0), -0.5 * Math.cos(Math.PI / 3.0)));

      calculatedVector = PointWiggler.computeBestShiftVectorToAvoidPoints(pointAtOrigin, pointsToAvoidByDistance, 1.0, 0.0);
      expectedVector = new Vector2D();

      EuclidCoreTestTools.assertVector2DGeometricallyEquals(expectedVector, calculatedVector, epsilon);
   }

   @Test
   public void testAverageDistanceVectorCalculationEasy2()
   {
      Point2DReadOnly pointToShift = new Point2D(0.33, -0.33);
      double desiredDistance = 0.5;

      Point2D topBound = new Point2D(0.53, -0.33);
      Point2D rightBound = new Point2D(0.33, -0.53);
      Point2D leftBound = new Point2D(0.33, -0.13);
      Point2D bottomBound = new Point2D(0.13, -0.33);

      List<Point2DReadOnly> pointsToAvoidByDistance = new ArrayList<>();
      pointsToAvoidByDistance.add(topBound);
      pointsToAvoidByDistance.add(rightBound);
      pointsToAvoidByDistance.add(leftBound);
      pointsToAvoidByDistance.add(bottomBound);

      Vector2DReadOnly expectedVector = computeShiftVectorAssumingNoLimits(pointsToAvoidByDistance, pointToShift, desiredDistance);
      Vector2DReadOnly calculatedVector = PointWiggler
            .computeBestShiftVectorToAvoidPoints(pointToShift, pointsToAvoidByDistance, desiredDistance, 0.0);

      EuclidCoreTestTools.assertVector2DGeometricallyEquals(expectedVector, calculatedVector, epsilon);

      Point2D shiftedPointExpected = new Point2D(pointToShift);
      shiftedPointExpected.add(expectedVector);

      Point2D shiftedPoint = new Point2D(pointToShift);
      shiftedPoint.add(calculatedVector);

      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(shiftedPointExpected, shiftedPoint, epsilon);

      pointToShift = new Point2D(0.33, -0.33);

      topBound = new Point2D(0.43, -0.23);
      rightBound = new Point2D(0.33, -0.53);
      leftBound = new Point2D(0.38, 0.13);
      bottomBound = new Point2D(0.18, -0.28);

      pointsToAvoidByDistance = new ArrayList<>();
      pointsToAvoidByDistance.add(topBound);
      pointsToAvoidByDistance.add(rightBound);
      pointsToAvoidByDistance.add(leftBound);
      pointsToAvoidByDistance.add(bottomBound);

      expectedVector = computeShiftVectorAssumingNoLimits(pointsToAvoidByDistance, pointToShift, desiredDistance);

      ConvexPolygon2D polygon2D = new ConvexPolygon2D();
      pointsToAvoidByDistance.forEach(polygon2D::addVertex);
      polygon2D.update();

      assertTrue(polygon2D.isPointInside(pointToShift));

      calculatedVector = PointWiggler.computeBestShiftVectorToAvoidPoints(pointToShift, pointsToAvoidByDistance, desiredDistance, 0.0);

      shiftedPoint.add(calculatedVector, pointToShift);
      shiftedPointExpected.add(expectedVector, pointToShift);

      EuclidCoreTestTools.assertVector2DGeometricallyEquals(expectedVector, calculatedVector, epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(shiftedPointExpected, shiftedPoint, epsilon);
   }

   @Test
   public void testWithinBoundsOfOneButNotTheOther()
   {
      Point2DReadOnly pointToShift = new Point2D(0.15, 0.25);
      double desiredDistance = 0.5;

      Point2D leftBound = new Point2D(0.15, 0.3);
      Point2D rightBound = new Point2D(0.15, -0.6);

      Point2D codedPointExpected = new Point2D(0.15, -0.15);
      Vector2D codedShiftExpected = new Vector2D(0.0, -0.3);


      List<Point2DReadOnly> pointsToAvoidByDistance = new ArrayList<>();
      pointsToAvoidByDistance.add(rightBound);
      pointsToAvoidByDistance.add(leftBound);

      Vector2DReadOnly expectedVector = computeShiftVectorAssumingNoLimits(pointsToAvoidByDistance, pointToShift, desiredDistance);
      Vector2DReadOnly calculatedVector = PointWiggler
            .computeBestShiftVectorToAvoidPoints(pointToShift, pointsToAvoidByDistance, desiredDistance, 0.0);

      EuclidCoreTestTools.assertVector2DGeometricallyEquals(expectedVector, calculatedVector, epsilon);

      Point2D shiftedPointExpected = new Point2D(pointToShift);
      shiftedPointExpected.add(expectedVector);

      Point2D shiftedPoint = new Point2D(pointToShift);
      shiftedPoint.add(calculatedVector);



      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(shiftedPointExpected, shiftedPoint, epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(codedPointExpected, shiftedPoint, epsilon);
   }

   @Test
   public void testAverageDistanceVectorWithRandomlyGeneratedNearbyPoints()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         int numberOfPoints = RandomNumbers.nextInt(random, 2, 50);

         Point2DReadOnly pointToShift = EuclidCoreRandomTools.nextPoint2D(random, 100.0);

         double minDistanceAway = 0.5;
         double maxDistanceAway = 1.5;
         double desiredDistanceAway = 1.6;

         List<Point2DReadOnly> pointsToAvoid = new ArrayList<>();
         for (int i = 0; i < numberOfPoints; i++)
         {
            Point2D pointToAvoid = new Point2D(pointToShift);
            double distanceAway = RandomNumbers.nextDouble(random, minDistanceAway, maxDistanceAway);

            Vector2D shiftVector = EuclidCoreRandomTools.nextVector2D(random);
            shiftVector.normalize();
            shiftVector.scale(distanceAway);
            pointToAvoid.add(shiftVector);
            pointsToAvoid.add(pointToAvoid);
         }

         Vector2DReadOnly calculatedVector = PointWiggler
               .computeBestShiftVectorToAvoidPoints(pointToShift, pointsToAvoid, desiredDistanceAway, 0.0);

         // it should just be the average of all these points
         Vector2D expectedVector = new Vector2D();
         for (int i = 0; i < numberOfPoints; i++)
         {
            Vector2D vectorToPoint = new Vector2D();
            vectorToPoint.sub(pointToShift, pointsToAvoid.get(i));
            double distanceAway = vectorToPoint.length();
            double distanceToShift = desiredDistanceAway - distanceAway;

            vectorToPoint.normalize();
            vectorToPoint.scale(distanceToShift);

            expectedVector.add(vectorToPoint);
         }

         expectedVector.scale(1.0 / numberOfPoints);

         EuclidCoreTestTools.assertVector2DGeometricallyEquals(expectedVector, calculatedVector, epsilon);
      }
   }

   @Test
   public void testFancyAverageDistanceVectorWithRandomlyGeneratedNearbyPoints()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         int numberOfPoints = RandomNumbers.nextInt(random, 2, 50);

         Point2DReadOnly pointToShift = EuclidCoreRandomTools.nextPoint2D(random, 100.0);

         double minDistanceAway = 0.5;
         double maxDistanceAway = 1.5;
         double desiredDistanceAway = 1.6;

         List<Point2DReadOnly> pointsToAvoid = new ArrayList<>();
         for (int i = 0; i < numberOfPoints; i++)
         {
            Point2D pointToAvoid = new Point2D(pointToShift);
            double distanceAway = RandomNumbers.nextDouble(random, minDistanceAway, maxDistanceAway);

            Vector2D shiftVector = EuclidCoreRandomTools.nextVector2D(random);
            shiftVector.normalize();
            shiftVector.scale(distanceAway);
            pointToAvoid.add(shiftVector);
            pointsToAvoid.add(pointToAvoid);
         }

         Vector2DReadOnly calculatedVector = PointWiggler
               .computeBestShiftVectorToAvoidPoints(pointToShift, pointsToAvoid, desiredDistanceAway, 0.0);

         // it should just be the average of all these points
         Vector2D expectedVector = computeShiftVectorAssumingNoLimits(pointsToAvoid, pointToShift, desiredDistanceAway);

         EuclidCoreTestTools.assertVector2DGeometricallyEquals(expectedVector, calculatedVector, epsilon);
      }
   }

   private Vector2D computeShiftVectorAssumingNoLimits(List<Point2DReadOnly> pointsToAvoidByDistance, Point2DReadOnly pointToShift, double desiredDistance)
   {
      Vector2D expectedVector = new Vector2D();

      for (int i = 0; i < pointsToAvoidByDistance.size(); i++)
      {
         Point2DReadOnly pointToAvoid = pointsToAvoidByDistance.get(i);
         Vector2D desiredShift = new Vector2D();
         desiredShift.sub(pointToShift, pointToAvoid);
         desiredShift.scale(desiredDistance / desiredShift.length());

         expectedVector.add(pointToAvoid);
         expectedVector.add(desiredShift);
      }

      expectedVector.scale(1.0 / pointsToAvoidByDistance.size());
      expectedVector.sub(pointToShift);

      return expectedVector;
   }
}
