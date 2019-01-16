package us.ihmc.pathPlanning.visibilityGraphs.postProcessing;

import org.junit.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class ObstacleAvoidanceProcessorTest
{
   private static final long timeout = 30000;
   private static final double epsilon = 1e-10;
   private static final int iters = 500;

   @Test(timeout = timeout)
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

      Vector2DReadOnly calculatedVector = ObstacleAvoidanceProcessor
            .computeVectorToMaximizeAverageDistanceFromPoints(pointAtOrigin, pointsToAvoidByDistance, 1.0);
      Vector2DReadOnly expectedVector = new Vector2D();

      EuclidCoreTestTools.assertVector2DGeometricallyEquals(expectedVector, calculatedVector, epsilon);

      // evenly spaced at
      pointsToAvoidByDistance.clear();
      pointsToAvoidByDistance.add(new Point2D(0.0, 0.5));
      pointsToAvoidByDistance.add(new Point2D(0.5 * Math.sin(Math.PI / 3.0), -0.5 * Math.cos(Math.PI / 3.0)));
      pointsToAvoidByDistance.add(new Point2D(-0.5 * Math.sin(Math.PI / 3.0), -0.5 * Math.cos(Math.PI / 3.0)));

      calculatedVector = ObstacleAvoidanceProcessor
            .computeVectorToMaximizeAverageDistanceFromPoints(pointAtOrigin, pointsToAvoidByDistance, 1.0);
      expectedVector = new Vector2D();

      EuclidCoreTestTools.assertVector2DGeometricallyEquals(expectedVector, calculatedVector, epsilon);
   }

   @Test(timeout = timeout)
   public void testAverageDistanceVectorCalculationWithEvenlyDistributedPoints()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         int numberOfPoints = RandomNumbers.nextInt(random, 2, 50);

         Point2DReadOnly expectedCenterPoint = EuclidCoreRandomTools.nextPoint2D(random, 100.0);
         Point2DReadOnly actualPoint = EuclidCoreRandomTools.nextPoint2D(random, 100.0);
         Vector2D expectedVector = new Vector2D();
         expectedVector.sub(expectedCenterPoint, actualPoint);

         double distanceFromPoint = expectedVector.length() * 1.5; // make it a little bigger

         List<Point2DReadOnly> pointsToAvoidByDistance = createPointsEvenlySpacedAboutPoint(expectedCenterPoint, numberOfPoints, expectedVector.length());

         Vector2DReadOnly calculatedVector = ObstacleAvoidanceProcessor
               .computeVectorToMaximizeAverageDistanceFromPoints(actualPoint, pointsToAvoidByDistance, distanceFromPoint);

         EuclidCoreTestTools.assertVector2DGeometricallyEquals(expectedVector, calculatedVector, epsilon);
      }
   }

   @Test(timeout = timeout)
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

         Vector2DReadOnly calculatedVector = ObstacleAvoidanceProcessor
               .computeVectorToMaximizeAverageDistanceFromPoints(pointToShift, pointsToAvoid, desiredDistanceAway);

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

   private static List<Point2DReadOnly> createPointsEvenlySpacedAboutPoint(Point2DReadOnly centerPoint, int numberOfPoints, double distanceFromPoint)
   {
      double angleIncrement = 2.0 * Math.PI / numberOfPoints;

      List<Point2DReadOnly> pointsToReturn = new ArrayList<>();
      for (int i = 0; i < numberOfPoints; i++)
      {
         double angle = angleIncrement * i;
         Point2D point = new Point2D(Math.sin(angle), Math.cos(angle));
         point.scale(distanceFromPoint);
         point.add(centerPoint);

         pointsToReturn.add(point);
      }

      return pointsToReturn;
   }
}

