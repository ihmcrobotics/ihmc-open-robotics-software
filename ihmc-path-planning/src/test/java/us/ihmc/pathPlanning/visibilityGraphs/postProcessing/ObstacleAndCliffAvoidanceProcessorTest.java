package us.ihmc.pathPlanning.visibilityGraphs.postProcessing;

import org.junit.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static junit.framework.TestCase.assertTrue;

public class ObstacleAndCliffAvoidanceProcessorTest
{
   private static final long timeout = 30000;
   private static final double epsilon = 1e-10;
   private static final int iters = 500;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
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

         ObstacleAndCliffAvoidanceProcessor.removeDuplicated3DPointsFromList(listOfPoints, distanceToFilter);

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

   @ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
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

         ObstacleAndCliffAvoidanceProcessor.removeDuplicated2DPointsFromList(listOfPoints, distanceToFilter);

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

   @ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
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

         ObstacleAndCliffAvoidanceProcessor.removeDuplicateStartOrEndPointsFromList(listOfPoints, startPoint, endPoint, distanceToFilter);

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
}

