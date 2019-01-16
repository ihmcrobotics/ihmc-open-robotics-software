package us.ihmc.pathPlanning.visibilityGraphs.postProcessing;

import org.junit.Ignore;
import org.junit.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.tools.ArrayTools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static junit.framework.TestCase.assertTrue;
import static org.junit.Assert.assertEquals;

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

      calculatedVector = ObstacleAvoidanceProcessor.computeVectorToMaximizeAverageDistanceFromPoints(pointAtOrigin, pointsToAvoidByDistance, 1.0);
      expectedVector = new Vector2D();

      EuclidCoreTestTools.assertVector2DGeometricallyEquals(expectedVector, calculatedVector, epsilon);
   }

   @Ignore
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

   @Ignore
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

   @Test(timeout = timeout)
   public void testRemoveDuplicated3DPointsFromList()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double distanceToFilter = RandomNumbers.nextDouble(random, 1e-4, 1.0);
         int numberOfPointsRemove = RandomNumbers.nextInt(random, 0, 10);
         int numberOfPointsToCreate = RandomNumbers.nextInt(random, numberOfPointsRemove, 50);

         List<Point3D> listOfPoints = new ArrayList<>();
         Point3D startPoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         listOfPoints.add(startPoint);
         int numberOfRemovablePointsMade = 0;
         for (int i = 1; i < numberOfPointsToCreate; i++)
         {
            int numberOfPointsRemainingToMake = numberOfPointsToCreate - listOfPoints.size();
            int numberOfRemovablePointsRemainingToMake = numberOfPointsRemove - numberOfRemovablePointsMade;

            double probabilityPointIsRemovable = (double) numberOfRemovablePointsRemainingToMake / numberOfPointsRemainingToMake;
            boolean nextPointIsRemovable = RandomNumbers.nextBoolean(random, probabilityPointIsRemovable);

            Vector3D displacementVector = EuclidCoreRandomTools.nextVector3D(random, 0.1, 1.0);
            displacementVector.normalize();

            if (nextPointIsRemovable)
            {
               displacementVector.scale(0.75 * distanceToFilter);
               numberOfRemovablePointsMade++;
            }
            else
            {
               displacementVector.scale(1.5 * distanceToFilter);
            }

            Point3D newPoint = new Point3D(listOfPoints.get(i - 1));
            newPoint.add(displacementVector);

            listOfPoints.add(newPoint);
         }

         ObstacleAvoidanceProcessor.removeDuplicated3DPointsFromList(listOfPoints, distanceToFilter);

         for (int i = 0; i < listOfPoints.size(); i++)
         {
            for (int j = i + 1; j < listOfPoints.size(); j++)
            {
               Point3DReadOnly pointA = listOfPoints.get(i);
               Point3DReadOnly pointB = listOfPoints.get(j);

               double distance = pointA.distance(pointB);
               assertTrue("Point " + i + " = " + pointA + " is too close to point " + j + " = " + pointB + ", with a distance of " + distance
                                + ", which should be at least " + distanceToFilter, distance > distanceToFilter);
            }
         }
      }
   }

   @Test(timeout = timeout)
   public void testRemoveDuplicated2DPointsFromList()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double distanceToFilter = RandomNumbers.nextDouble(random, 1e-4, 1.0);
         int numberOfPointsRemove = RandomNumbers.nextInt(random, 0, 10);
         int numberOfPointsToCreate = RandomNumbers.nextInt(random, numberOfPointsRemove, 50);

         List<Point2D> listOfPoints = new ArrayList<>();
         Point2D startPoint = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         listOfPoints.add(startPoint);
         int numberOfRemovablePointsMade = 0;
         for (int i = 1; i < numberOfPointsToCreate; i++)
         {
            int numberOfPointsRemainingToMake = numberOfPointsToCreate - listOfPoints.size();
            int numberOfRemovablePointsRemainingToMake = numberOfPointsRemove - numberOfRemovablePointsMade;

            double probabilityPointIsRemovable = (double) numberOfRemovablePointsRemainingToMake / numberOfPointsRemainingToMake;
            boolean nextPointIsRemovable = RandomNumbers.nextBoolean(random, probabilityPointIsRemovable);

            Vector2D displacementVector = EuclidCoreRandomTools.nextVector2D(random, 0.1, 1.0);
            displacementVector.normalize();

            if (nextPointIsRemovable)
            {
               displacementVector.scale(0.75 * distanceToFilter);
               numberOfRemovablePointsMade++;
            }
            else
            {
               displacementVector.scale(1.5 * distanceToFilter);
            }

            Point2D newPoint = new Point2D(listOfPoints.get(i - 1));
            newPoint.add(displacementVector);

            listOfPoints.add(newPoint);
         }

         ObstacleAvoidanceProcessor.removeDuplicated2DPointsFromList(listOfPoints, distanceToFilter);

         for (int i = 0; i < listOfPoints.size(); i++)
         {
            for (int j = i + 1; j < listOfPoints.size(); j++)
            {
               Point2DReadOnly pointA = listOfPoints.get(i);
               Point2DReadOnly pointB = listOfPoints.get(j);

               double distance = pointA.distance(pointB);
               assertTrue("Point " + i + " = " + pointA + " is too close to point " + j + " = " + pointB + ", with a distance of " + distance
                                + ", which should be at least " + distanceToFilter, distance > distanceToFilter);
            }
         }
      }
   }

   @Test(timeout = timeout)
   public void testRemoveDuplicateStartOrEndPointsFromList()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double distanceToFilter = RandomNumbers.nextDouble(random, 1e-4, 1.0);
         int numberOfPointsRemove = RandomNumbers.nextInt(random, 0, 10);
         int numberOfPointsToCreate = RandomNumbers.nextInt(random, numberOfPointsRemove, 50);

         List<Point3D> listOfPoints = new ArrayList<>();
         Point3D startPoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D endPoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         int numberOfRemovablePointsMade = 0;
         for (int i = 0; i < numberOfPointsToCreate; i++)
         {
            int numberOfPointsRemainingToMake = numberOfPointsToCreate - listOfPoints.size();
            int numberOfRemovablePointsRemainingToMake = numberOfPointsRemove - numberOfRemovablePointsMade;

            double probabilityPointIsRemovable = (double) numberOfRemovablePointsRemainingToMake / numberOfPointsRemainingToMake;
            boolean nextPointIsRemovable = RandomNumbers.nextBoolean(random, probabilityPointIsRemovable);

            Vector3D displacementVector = EuclidCoreRandomTools.nextVector3D(random, -5.0, 5.0);
            displacementVector.normalize();

            if (nextPointIsRemovable)
            {
               displacementVector.scale(0.75 * distanceToFilter);
               numberOfRemovablePointsMade++;
            }
            else
            {
               displacementVector.scale(1.5 * distanceToFilter);
            }

            boolean displacementFromStartPoint = RandomNumbers.nextBoolean(random, 0.5);
            Point3D newPoint = new Point3D();
            if (displacementFromStartPoint)
               newPoint.set(startPoint);
            else
               newPoint.set(endPoint);
            newPoint.add(displacementVector);

            listOfPoints.add(newPoint);
         }

         ObstacleAvoidanceProcessor.removeDuplicateStartOrEndPointsFromList(listOfPoints, startPoint, endPoint, distanceToFilter);

         for (int i = 0; i < listOfPoints.size(); i++)
         {
            Point3DReadOnly point = listOfPoints.get(i);

            double distanceFromStart = point.distance(startPoint);
            double distanceFromEnd = point.distance(endPoint);
            assertTrue("Point " + i + " = " + point + " is too close to start " + startPoint + ", with a distance of " + distanceFromStart
                             + ", which should be at least " + distanceToFilter, distanceFromStart > distanceToFilter);
            assertTrue(
                  "Point " + i + " = " + point + " is too close to end " + endPoint + ", with a distance of " + distanceFromEnd + ", which should be at least "
                        + distanceToFilter, distanceFromEnd > distanceToFilter);
         }
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

