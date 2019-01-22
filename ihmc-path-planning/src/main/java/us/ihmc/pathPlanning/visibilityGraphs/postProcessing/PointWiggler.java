package us.ihmc.pathPlanning.visibilityGraphs.postProcessing;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.convexOptimization.quadraticProgram.JavaQuadProgSolver;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

import java.util.ArrayList;
import java.util.List;

public class PointWiggler
{
   static Vector2DReadOnly computeVectorToMaximizeAverageDistanceFromPoints(Point2DReadOnly pointToShift, List<Point2DReadOnly> pointsToAvoidByDistance,
                                                                            double desiredDistance, double minimumDistance)
   {
      return computeVectorToMaximizeAverageDistanceFromPoints(pointToShift, pointsToAvoidByDistance, new ArrayList<>(), desiredDistance, 0.0, minimumDistance,
                                                              0.0);
   }

   static Vector2DReadOnly computeVectorToMaximizeAverageDistanceFromPoints(Point2DReadOnly pointToShift, List<Point2DReadOnly> pointsToAvoidByDistanceA,
                                                                            List<Point2DReadOnly> pointsToAvoidByDistanceB, double desiredDistanceA,
                                                                            double desiredDistanceB, double minimumDistanceA, double minimumDistanceB)
   {

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

      int numberOfPointsWithinProximity = pointsToAvoid.size();

      DenseMatrix64F A = new DenseMatrix64F(2, 2);
      DenseMatrix64F b = new DenseMatrix64F(2, 1);
      DenseMatrix64F CI = new DenseMatrix64F(numberOfPointsWithinProximity, 2);
      DenseMatrix64F ci = new DenseMatrix64F(numberOfPointsWithinProximity, 1);

      int numberOfPointsAdded = 0;
      Vector2D vectorToPoint = new Vector2D();

      for (Point2DReadOnly pointToShiftFrom : pointsToAvoid)
      {
         vectorToPoint.sub(pointToShift, pointToShiftFrom);

         double distanceToPoint = vectorToPoint.length();
         double desiredDistance = pointsToAvoidByDistanceACopy.contains(pointToShiftFrom) ? desiredDistanceA : desiredDistanceB;
         double minimumDistance = pointsToAvoidByDistanceACopy.contains(pointToShiftFrom) ? minimumDistanceA : minimumDistanceB;
         double distanceToShift = desiredDistance - distanceToPoint;

         if (distanceToShift > 0)
         {
            vectorToPoint.scale(desiredDistance / distanceToPoint);

            b.add(0, 0, -2.0 * (pointToShiftFrom.getX() + vectorToPoint.getX()));
            b.add(1, 0, -2.0 * (pointToShiftFrom.getY() + vectorToPoint.getY()));

            CI.set(numberOfPointsAdded, 0, -vectorToPoint.getX());
            CI.set(numberOfPointsAdded, 1, -vectorToPoint.getY());
            ci.set(numberOfPointsAdded,
                   -minimumDistance - vectorToPoint.getX() * pointToShiftFrom.getX() - vectorToPoint.getY() * pointToShiftFrom.getY());

            numberOfPointsAdded++;
         }
      }

      // remove unused constraints
      while (CI.getNumRows() > numberOfPointsAdded)
      {
         MatrixTools.removeRow(CI, numberOfPointsAdded);
         MatrixTools.removeRow(ci, numberOfPointsAdded);
      }

      A.set(0, 0, numberOfPointsAdded);
      A.set(1, 1, numberOfPointsAdded);

      // done because expectation is 0.5 x^T A x + b
      CommonOps.scale(2.0, A);

      JavaQuadProgSolver solver = new JavaQuadProgSolver();
      solver.setQuadraticCostFunction(A, b, 0.0);
      solver.setLinearInequalityConstraints(CI, ci);

      DenseMatrix64F shiftedPointSolutionVector = new DenseMatrix64F(2, 1);
      solver.solve(shiftedPointSolutionVector);

      Point2D shiftedPoint = new Point2D();
      shiftedPoint.set(shiftedPointSolutionVector);

      Vector2D shiftVector = new Vector2D();
      shiftVector.sub(shiftedPoint, pointToShift);

      return shiftVector;
   }
}
