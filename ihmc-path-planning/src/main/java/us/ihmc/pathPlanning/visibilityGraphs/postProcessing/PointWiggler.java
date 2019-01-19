package us.ihmc.pathPlanning.visibilityGraphs.postProcessing;

import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

import java.util.ArrayList;
import java.util.List;

public class PointWiggler
{
   static Vector2DReadOnly computeVectorToMaximizeAverageDistanceFromPoints(Point2DReadOnly pointToShift, List<Point2DReadOnly> pointsToAvoidByDistance,
                                                                            double desiredDistance)
   {
      return computeVectorToMaximizeAverageDistanceFromPoints(pointToShift, pointsToAvoidByDistance, new ArrayList<>(), desiredDistance, 0.0);
   }

   static Vector2DReadOnly computeVectorToMaximizeAverageDistanceFromPoints(Point2DReadOnly pointToShift, List<Point2DReadOnly> pointsToAvoidByDistanceA,
                                                                            List<Point2DReadOnly> pointsToAvoidByDistanceB, double distanceA, double distanceB)
   {
      Vector2D averageShiftVector = new Vector2D();
      int numberOfPointsWithinProximity = 0;

      // sort these by distance
      List<Point2DReadOnly> pointsToAvoid = new ArrayList<>(pointsToAvoidByDistanceA);
      pointsToAvoid.addAll(pointsToAvoidByDistanceB);
      pointsToAvoid.sort((a, b) -> {
         double distanceForPointA = pointsToAvoidByDistanceA.contains(a) ? distanceA : distanceB;
         double distanceForPointB = pointsToAvoidByDistanceA.contains(b) ? distanceA : distanceB;

         double aShiftDistance = pointToShift.distance(a) - distanceForPointA;
         double bShiftDistance = pointToShift.distance(b) - distanceForPointB;

         return Double.compare(aShiftDistance, bShiftDistance);
      });

      Vector2D vectorToPoint = new Vector2D();

      for (Point2DReadOnly pointToShiftFrom : pointsToAvoid)
      {
         vectorToPoint.sub(pointToShift, pointToShiftFrom);
         double distanceToPoint = vectorToPoint.length();
         double distanceAfterShifting = distanceToPoint + averageShiftVector.dot(vectorToPoint);

         double distanceToAchieve = pointsToAvoidByDistanceA.contains(pointToShiftFrom) ? distanceA : distanceB;
         if (distanceAfterShifting < distanceToAchieve)
         {
            double extraDistanceToShift = distanceToAchieve - distanceToPoint;
            vectorToPoint.scale(extraDistanceToShift / distanceToPoint);

            // add this offset into shift vector
            averageShiftVector.scale(numberOfPointsWithinProximity);
            averageShiftVector.add(vectorToPoint);
            numberOfPointsWithinProximity++;
            averageShiftVector.scale(1.0 / numberOfPointsWithinProximity);
         }
      }

      // FIXME this doesn't currently work
      // enforce the minimum distance to a point
      /*
      for (Point2DReadOnly pointToShiftFrom : pointsToAvoid)
      {
         vectorToPoint.sub(pointToShift, pointToShiftFrom);
         vectorToPoint.add(averageShiftVector);
         double distanceToPoint = vectorToPoint.length();
         double distanceAfterShifting = distanceToPoint + averageShiftVector.dot(vectorToPoint);

         double distanceToAchieve = pointsToAvoidByDistanceA.contains(pointToShiftFrom) ? minimumDistanceA : minimumDistanceB;

         if (distanceAfterShifting < distanceToAchieve)
         {
            double extraDistanceToShift = distanceToAchieve - distanceToPoint;
            vectorToPoint.scale(extraDistanceToShift / distanceToPoint);

            averageShiftVector.add(vectorToPoint);
         }
      }
      */

      return averageShiftVector;
   }
}
