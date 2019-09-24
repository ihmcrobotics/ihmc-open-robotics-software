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
import java.util.Comparator;
import java.util.List;

public class PointWiggler
{
   static Vector2DReadOnly computeBestShiftVectorToAvoidPoints(Point2DReadOnly pointToShift, List<Point2DReadOnly> pointsToAvoidByDistance,
                                                               double desiredDistance, double minimumDistance)
   {
      return computeBestShiftVectorToAvoidPoints(pointToShift, pointsToAvoidByDistance, new ArrayList<>(), desiredDistance, 0.0, minimumDistance,
                                                 0.0);
   }

   static Vector2DReadOnly computeBestShiftVectorToAvoidPoints(Point2DReadOnly pointToShift, List<Point2DReadOnly> pointsToAvoidByDistanceA,
                                                               List<Point2DReadOnly> pointsToAvoidByDistanceB, double desiredDistanceA,
                                                               double desiredDistanceB, double minimumDistanceA, double minimumDistanceB)
   {
      if (pointsToAvoidByDistanceA.size() == 0 && pointsToAvoidByDistanceB.size() == 0)
         return new Vector2D();

      double maxShift = Math.max(desiredDistanceA, desiredDistanceB);
      List<PointInfo> pointsInfoToAvoid = new ArrayList<>();
      for (Point2DReadOnly pointToAvoidByDistanceA : pointsToAvoidByDistanceA)
      {
         double distance = pointToAvoidByDistanceA.distance(pointToShift);
         if (distance < desiredDistanceA + maxShift)
            pointsInfoToAvoid.add(new PointInfo(pointToAvoidByDistanceA, pointToShift, desiredDistanceA, minimumDistanceA, distance));
      }
      for (Point2DReadOnly pointToAvoidByDistanceB : pointsToAvoidByDistanceB)
      {
         double distance = pointToAvoidByDistanceB.distance(pointToShift);
         if (distance < desiredDistanceB + maxShift)
            pointsInfoToAvoid.add(new PointInfo(pointToAvoidByDistanceB, pointToShift, desiredDistanceB, minimumDistanceB, distance));
      }

      // sort everything in ascending distance order
      pointsInfoToAvoid.sort(Comparator.comparingDouble(pointInfo -> pointInfo.distanceToShift));

      // filter out the points that won't be affected by any shifting
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
               affectedByOtherShift = currentDistance + otherPointToAvoidInfo.desiredVector.dot(currentVector) < currentDesiredDistance;
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

      for (PointInfo pointToShiftFromInfo : pointsInfoToAvoid)
      {
         Point2DReadOnly pointToShiftFrom = pointToShiftFromInfo.pointToAvoid;

         double minimumDistance = pointToShiftFromInfo.minimumDistanceToPoint;

         Vector2DReadOnly desiredVectorToPoint = pointToShiftFromInfo.desiredVector;

         double desiredX = pointToShiftFrom.getX() + desiredVectorToPoint.getX();
         double desiredY = pointToShiftFrom.getY() + desiredVectorToPoint.getY();
         b.add(0, 0, -2.0 * desiredX);
         b.add(1, 0, -2.0 * desiredY);

         CI.set(numberOfPointsAdded, 0, -desiredVectorToPoint.getX());
         CI.set(numberOfPointsAdded, 1, -desiredVectorToPoint.getY());
         ci.set(numberOfPointsAdded,
                -minimumDistance - desiredVectorToPoint.getX() * pointToShiftFrom.getX() - desiredVectorToPoint.getY() * pointToShiftFrom.getY());

         numberOfPointsAdded++;
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
      private final double distanceToShift;
      private final Point2DReadOnly pointToAvoid;
      private final Vector2D desiredVector;

      public PointInfo(Point2DReadOnly pointToAvoid, Point2DReadOnly pointToShift, double desiredDistanceToPoint, double minimumDistanceToPoint,
                       double distanceToPoint)
      {
         this.desiredDistanceToPoint = desiredDistanceToPoint;
         this.minimumDistanceToPoint = minimumDistanceToPoint;
         this.pointToAvoid = new Point2D(pointToAvoid);

         this.distanceToPoint = distanceToPoint;
         distanceToShift = desiredDistanceToPoint - distanceToPoint;

         desiredVector = new Vector2D();
         desiredVector.sub(pointToShift, pointToAvoid);
         desiredVector.scale(desiredDistanceToPoint / distanceToPoint);
      }
   }
}
