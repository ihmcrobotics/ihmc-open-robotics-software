package us.ihmc.pathPlanning.visibilityGraphs.postProcessing;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.convexOptimization.quadraticProgram.JavaQuadProgSolver;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.matrixlib.MatrixTools;

/**
 * Uses QP to wiggle a point away from clusters of other points
 * towards a desired distance and away a minimum distance.
 */
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
         Vector2D currentDirection = new Vector2D();
         PointInfo pointToAvoidInfo = pointsInfoToAvoid.get(pointToCheckIndex);
         currentDirection.sub(pointToShift, pointToAvoidInfo.pointToAvoid);
         currentDirection.normalize();

         double currentDistance = pointToAvoidInfo.distanceToPoint;
         double currentDesiredDistance = pointToAvoidInfo.desiredDistanceToPoint;

         boolean affectedByOtherShift = currentDistance < currentDesiredDistance;

         while (otherPointIndex < pointsInfoToAvoid.size() && !affectedByOtherShift)
         {
            if (otherPointIndex != pointToCheckIndex)
            {
               PointInfo otherPointToAvoidInfo = pointsInfoToAvoid.get(otherPointIndex);
               double projectionOntoCurrent = otherPointToAvoidInfo.desiredVector.dot(currentDirection);
               affectedByOtherShift = currentDistance + projectionOntoCurrent < currentDesiredDistance;
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

      DMatrixRMaj A = new DMatrixRMaj(2, 2);
      DMatrixRMaj b = new DMatrixRMaj(2, 1);
      DMatrixRMaj CI = new DMatrixRMaj(numberOfPointsWithinProximity, 2);
      DMatrixRMaj ci = new DMatrixRMaj(numberOfPointsWithinProximity, 1);

      int numberOfPointsAdded = 0;

      for (PointInfo pointToShiftFromInfo : pointsInfoToAvoid)
      {
         Point2DReadOnly pointToShiftFrom = pointToShiftFromInfo.pointToAvoid;

         double minimumDistance = pointToShiftFromInfo.minimumDistanceToPoint;

         Vector2D desiredVectorToPoint = new Vector2D(pointToShiftFromInfo.desiredVector);

         double desiredX = pointToShiftFrom.getX() + desiredVectorToPoint.getX();
         double desiredY = pointToShiftFrom.getY() + desiredVectorToPoint.getY();
         b.add(0, 0, -2.0 * desiredX);
         b.add(1, 0, -2.0 * desiredY);

         // FIXME this is wrong, didn't normalize
         desiredVectorToPoint.normalize();
         CI.set(numberOfPointsAdded, 0, -desiredVectorToPoint.getX());
         CI.set(numberOfPointsAdded, 1, -desiredVectorToPoint.getY());
         ci.set(numberOfPointsAdded,
                -minimumDistance - desiredVectorToPoint.getX() * pointToShiftFrom.getX() - desiredVectorToPoint.getY() * pointToShiftFrom.getY());

         numberOfPointsAdded++;
      }

      if (numberOfPointsAdded == 0)
         return new Vector2D();

      // remove unused constraints; still need this?
      while (CI.getNumRows() > numberOfPointsAdded)
      {
         MatrixTools.removeRow(CI, numberOfPointsAdded);
         MatrixTools.removeRow(ci, numberOfPointsAdded);
      }

      A.set(0, 0, numberOfPointsAdded);
      A.set(1, 1, numberOfPointsAdded);

      // done because expectation is 0.5 x^T A x + b
      CommonOps_DDRM.scale(2.0, A);

      JavaQuadProgSolver solver = new JavaQuadProgSolver();
      solver.setQuadraticCostFunction(A, b, 0.0);
      solver.setLinearInequalityConstraints(CI, ci);

      DMatrixRMaj shiftedPointSolutionVector = new DMatrixRMaj(2, 1);
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
