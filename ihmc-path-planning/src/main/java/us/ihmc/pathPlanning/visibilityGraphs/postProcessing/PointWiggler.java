package us.ihmc.pathPlanning.visibilityGraphs.postProcessing;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.convexOptimization.quadraticProgram.JavaQuadProgSolver;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

import javax.vecmath.Vector2d;
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

   static Vector2DReadOnly computeVectorToMaximizeAverageDistanceFromPointsFancily(Point2DReadOnly pointToShift, List<Point2DReadOnly> pointsToAvoidByDistance,
                                                                            double desiredDistance)
   {
      return computeVectorToMaximizeAverageDistanceFromPointsFancily(pointToShift, pointsToAvoidByDistance, new ArrayList<>(), desiredDistance, 0.0);
   }

   static Vector2DReadOnly computeVectorToMaximizeAverageDistanceFromPointsFancily(Point2DReadOnly pointToShift, List<Point2DReadOnly> pointsToAvoidByDistanceA,
                                                                            List<Point2DReadOnly> pointsToAvoidByDistanceB, double desiredDistanceA,
                                                                                   double desiredDistanceB)
   {
      Vector2D averageShiftVector = new Vector2D();
      int numberOfPointsWithinProximity = 0;

      List<Point2DReadOnly> pointsToAvoidByDistanceACopy = new ArrayList<>(pointsToAvoidByDistanceA);
      List<Point2DReadOnly> pointsToAvoidByDistanceBCopy = new ArrayList<>(pointsToAvoidByDistanceB);

      // remove them if we know they can't be to close
      double maxShift = Math.max(desiredDistanceA, desiredDistanceB);
      int pointIndex = 0;
      while (pointIndex < pointsToAvoidByDistanceACopy.size())
      {
         if (pointsToAvoidByDistanceACopy.get(pointIndex).distance(pointToShift) > desiredDistanceA + maxShift)
            pointsToAvoidByDistanceACopy.remove(pointIndex);
         else
            pointIndex++;
      }
      pointIndex = 0;
      while (pointIndex < pointsToAvoidByDistanceBCopy.size())
      {
         if (pointsToAvoidByDistanceBCopy.get(pointIndex).distance(pointToShift) > desiredDistanceB + maxShift)
            pointsToAvoidByDistanceBCopy.remove(pointIndex);
         else
            pointIndex++;
      }


      // sort everything in ascending order
      List<Point2DReadOnly> pointsToAvoid = new ArrayList<>(pointsToAvoidByDistanceACopy);
      pointsToAvoid.addAll(pointsToAvoidByDistanceBCopy);
      pointsToAvoid.sort((pointA, pointB) -> {
         double distanceForPointA = pointsToAvoidByDistanceACopy.contains(pointA) ? desiredDistanceA : desiredDistanceB;
         double distanceForPointB = pointsToAvoidByDistanceACopy.contains(pointB) ? desiredDistanceA : desiredDistanceB;

         double aShiftDistance = pointToShift.distance(pointA) - distanceForPointA;
         double bShiftDistance = pointToShift.distance(pointB) - distanceForPointB;

         return Double.compare(aShiftDistance, bShiftDistance);
      });


      int pointToCheckIndex = 0;
      while (pointToCheckIndex < pointsToAvoid.size())
      {
         int otherPointIndex = 0;
         Vector2D currentVector = new Vector2D();
         Point2DReadOnly pointToAvoid = pointsToAvoid.get(pointToCheckIndex);
         currentVector.sub(pointToShift, pointToAvoid);
         double currentDistance = currentVector.length();
         double currentDesiredDistance = pointsToAvoidByDistanceACopy.contains(pointToAvoid) ? desiredDistanceA : desiredDistanceB;

         boolean affectedByOtherShift = currentDistance < currentDesiredDistance;

         while (otherPointIndex < pointsToAvoid.size() && !affectedByOtherShift)
         {
            if (otherPointIndex != pointToCheckIndex)
            {
               Point2DReadOnly otherPointToAvoid = pointsToAvoid.get(otherPointIndex);

               double distance = pointsToAvoidByDistanceACopy.contains(otherPointToAvoid) ? desiredDistanceA : desiredDistanceB;

               Vector2D otherVector = new Vector2D();
               otherVector.sub(pointToShift, otherPointToAvoid);
               otherVector.scale(distance, otherVector.length());

               affectedByOtherShift = currentDistance + otherVector.dot(currentVector) < currentDesiredDistance;
            }
            otherPointIndex++;
         }

         if (!affectedByOtherShift)
            pointsToAvoid.remove(pointToCheckIndex);
         else
            pointToCheckIndex++;
      }


      Vector2D vectorToPoint = new Vector2D();

      DenseMatrix64F A = new DenseMatrix64F(2, 2);
      DenseMatrix64F b = new DenseMatrix64F(2, 1);
      double residualCost = 0.0;


      for (Point2DReadOnly pointToShiftFrom : pointsToAvoid)
      {
         vectorToPoint.sub(pointToShift, pointToShiftFrom);
         double distanceToPoint = vectorToPoint.length();
         double desiredDistance = pointsToAvoidByDistanceACopy.contains(pointToShiftFrom) ? desiredDistanceA : desiredDistanceB;
         double distanceToShift = desiredDistance - distanceToPoint;

         if (distanceToShift > 0)
         {
            vectorToPoint.scale(distanceToShift / distanceToPoint);

            b.add(0, 0, vectorToPoint.getX());
            b.add(1, 0, vectorToPoint.getY());
            residualCost += vectorToPoint.dot(vectorToPoint);

            numberOfPointsWithinProximity++;
         }
      }

      A.set(0, 0, numberOfPointsWithinProximity);
      A.set(1, 1, numberOfPointsWithinProximity);

      JavaQuadProgSolver solver = new JavaQuadProgSolver();
      solver.setQuadraticCostFunction(A, b, residualCost);

      DenseMatrix64F solutionVector = new DenseMatrix64F(2, 1);
      solver.solve(solutionVector);

      averageShiftVector.set(solutionVector);

      return averageShiftVector;
   }



}
