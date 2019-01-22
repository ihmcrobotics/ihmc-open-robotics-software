package us.ihmc.pathPlanning.visibilityGraphs.postProcessing;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.convexOptimization.quadraticProgram.JavaQuadProgSolver;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.tools.lists.PairList;

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
      double maxShift = Math.max(desiredDistanceA, desiredDistanceB);
      List<PointInfo> pointsInfoToAvoid = new ArrayList<>();
      for (Point2DReadOnly pointToAvoidByDistanceA : pointsToAvoidByDistanceA)
      {
         double distance = pointToAvoidByDistanceA.distance(pointToShift);
         if (distance < desiredDistanceA + maxShift)
            pointsInfoToAvoid.add(new PointInfo(pointToAvoidByDistanceA, desiredDistanceA, minimumDistanceA, distance));
      }
      for (Point2DReadOnly pointToAvoidByDistanceB : pointsToAvoidByDistanceB)
      {
         double distance = pointToAvoidByDistanceB.distance(pointToShift);
         if (distance < desiredDistanceB + maxShift)
            pointsInfoToAvoid.add(new PointInfo(pointToAvoidByDistanceB, desiredDistanceB, minimumDistanceB, pointToAvoidByDistanceB.distance(pointToShift)));
      }



      // sort everything in ascending distance order
      pointsInfoToAvoid.sort((pointAInfo, pointBInfo) -> {
         double aShiftDistance = pointAInfo.distanceToPoint - pointAInfo.desiredDistanceToPoint;
         double bShiftDistance = pointBInfo.distanceToPoint - pointBInfo.desiredDistanceToPoint;

         return Double.compare(aShiftDistance, bShiftDistance);
      });

      int pointToCheckIndex = 0;
      while (pointToCheckIndex < pointsInfoToAvoid.size())
      {
         int otherPointIndex = 0;
         Vector2D currentVector = new Vector2D();
         PointInfo pointToAvoidInfo = pointsInfoToAvoid.get(pointToCheckIndex);
         currentVector.sub(pointToShift, pointToAvoidInfo.pointToAvoid);
         double currentDistance = pointToAvoidInfo.distanceToPoint;
         double currentDesiredDistance = pointToAvoidInfo.desiredDistanceToPoint;

         boolean affectedByOtherShift = currentDistance < currentDesiredDistance;

         while (otherPointIndex < pointsInfoToAvoid.size() && !affectedByOtherShift)
         {
            if (otherPointIndex != pointToCheckIndex)
            {
               PointInfo otherPointToAvoidInfo = pointsInfoToAvoid.get(otherPointIndex);

               double desiredDistance = otherPointToAvoidInfo.desiredDistanceToPoint;

               Vector2D otherVector = new Vector2D();
               otherVector.sub(pointToShift, pointToAvoidInfo.pointToAvoid);
               otherVector.scale(desiredDistance, otherPointToAvoidInfo.distanceToPoint);

               affectedByOtherShift = currentDistance + otherVector.dot(currentVector) < currentDesiredDistance;
            }
            otherPointIndex++;
         }

         if (!affectedByOtherShift)
         {
            pointsInfoToAvoid.remove(pointToCheckIndex);
         }
         else
            pointToCheckIndex++;
      }

      int numberOfPointsWithinProximity = pointsInfoToAvoid.size();

      DenseMatrix64F A = new DenseMatrix64F(2, 2);
      DenseMatrix64F b = new DenseMatrix64F(2, 1);
      DenseMatrix64F CI = new DenseMatrix64F(numberOfPointsWithinProximity, 2);
      DenseMatrix64F ci = new DenseMatrix64F(numberOfPointsWithinProximity, 1);

      int numberOfPointsAdded = 0;
      Vector2D vectorToPoint = new Vector2D();

      for (PointInfo pointToShiftFromInfo : pointsInfoToAvoid)
      {
         Point2DReadOnly pointToShiftFrom = pointToShiftFromInfo.pointToAvoid;

         double distanceToPoint = pointToShiftFromInfo.distanceToPoint;
         double desiredDistance = pointToShiftFromInfo.desiredDistanceToPoint;
         double minimumDistance = pointToShiftFromInfo.minimumDistanceToPoint;
         double distanceToShift = desiredDistance - distanceToPoint;

         if (distanceToShift > 0)
         {
            vectorToPoint.sub(pointToShift, pointToShiftFrom);
            vectorToPoint.scale(desiredDistance / distanceToPoint);

            b.add(0, 0, -2.0 * (pointToShiftFrom.getX() + vectorToPoint.getX()));
            b.add(1, 0, -2.0 * (pointToShiftFrom.getY() + vectorToPoint.getY()));

            CI.set(numberOfPointsAdded, 0, -vectorToPoint.getX());
            CI.set(numberOfPointsAdded, 1, -vectorToPoint.getY());
            ci.set(numberOfPointsAdded, -minimumDistance - vectorToPoint.getX() * pointToShiftFrom.getX() - vectorToPoint.getY() * pointToShiftFrom.getY());

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

   private static class PointInfo
   {
      private final double distanceToPoint;
      private final double desiredDistanceToPoint;
      private final double minimumDistanceToPoint;
      private final Point2DReadOnly pointToAvoid;

      public PointInfo(Point2DReadOnly pointToAvoid, double desiredDistanceToPoint, double minimumDistanceToPoint, double distanceToPoint)
      {
         this.desiredDistanceToPoint = desiredDistanceToPoint;
         this.distanceToPoint = distanceToPoint;
         this.minimumDistanceToPoint = minimumDistanceToPoint;
         this.pointToAvoid = new Point2D(pointToAvoid);
      }
   }
}
