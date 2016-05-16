package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.splineOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.MathTools;

import java.util.List;

public class CubicSplineTools
{
   private final DenseMatrix64F coefficientConstraints = new DenseMatrix64F(1,1);
   private final DenseMatrix64F piecewiseTimeCoefficients = new DenseMatrix64F(1,1);

   /**
    * Compute a matrix that multiplies knots of a piece-wise continuous cubic spline to return the piecewise cubic spline coefficients.
    *
    * @param durations List containing the duration of each piece in the piecewise continuous spline
    * @param projectionMatrixToPack DenseMatrix that multiplies each knot to get the polynomial for its segment
    * @param projectionMatrixDotToPack DenseMatrix that multiplies each knot to get the first polynomial derivative for its segment
    * @param projectionMatrixDotDotToPack DenseMatrix that multiplies each knot to get the second polynomial derivative for its segment
    */
   public void computeKnotProjectionMatrix(List<Double> durations, DenseMatrix64F projectionMatrixToPack, DenseMatrix64F projectionMatrixDotToPack,
         DenseMatrix64F projectionMatrixDotDotToPack)
   {
      int numberOfKnots = durations.size() + 1;

      coefficientConstraints.reshape(4 * (numberOfKnots - 1), numberOfKnots);
      piecewiseTimeCoefficients.reshape(4, numberOfKnots);

      if (numberOfKnots == 2)
      {
         double duration = durations.get(0);

         piecewiseTimeCoefficients.set(0, 0, 1);
         piecewiseTimeCoefficients.set(1, 1, 1);

         piecewiseTimeCoefficients.set(2, 0, 1);
         piecewiseTimeCoefficients.set(2, 1, duration);
         piecewiseTimeCoefficients.set(2, 2, MathTools.powWithInteger(duration, 2));
         piecewiseTimeCoefficients.set(2, 3, MathTools.powWithInteger(duration, 3));

         piecewiseTimeCoefficients.set(3, 0, 0);
         piecewiseTimeCoefficients.set(3, 1, 1);
         piecewiseTimeCoefficients.set(3, 2, 2 * duration);
         piecewiseTimeCoefficients.set(3, 2, 3 * MathTools.powWithInteger(duration, 2));

         coefficientConstraints.set(0, 0, 1);
         coefficientConstraints.set(2, 1, 1);
      }
      else
      {

         double duration = durations.get(0);
         piecewiseTimeCoefficients.set(0, 0, 1);
         piecewiseTimeCoefficients.set(1, 1, 1);

         piecewiseTimeCoefficients.set(2, 0, 1);
         piecewiseTimeCoefficients.set(2, 1, duration);
         piecewiseTimeCoefficients.set(2, 2, MathTools.powWithInteger(duration, 2));
         piecewiseTimeCoefficients.set(2, 3, MathTools.powWithInteger(duration, 3));

         piecewiseTimeCoefficients.set(3, 1, 1);
         piecewiseTimeCoefficients.set(3, 2, 2 * duration);
         piecewiseTimeCoefficients.set(3, 3, 3 * MathTools.powWithInteger(duration, 2));
         piecewiseTimeCoefficients.set(3, 5, -1);

         piecewiseTimeCoefficients.set(4, 2, 2);
         piecewiseTimeCoefficients.set(4, 3, 6 * duration);
         piecewiseTimeCoefficients.set(4, 6, -1);

         coefficientConstraints.set(0, 0, 1);
         coefficientConstraints.set(2, 1, 1);

         int rowIndex = 4;
         int colIndex = 3;

         for(int i = 1; i < numberOfKnots - 2; i++)
         {
            duration = durations.get(i);

            piecewiseTimeCoefficients.set(rowIndex + 1, colIndex + 1, 1);

            piecewiseTimeCoefficients.set(rowIndex + 2, colIndex + 1, 1);
            piecewiseTimeCoefficients.set(rowIndex + 2, colIndex + 2, duration);
            piecewiseTimeCoefficients.set(rowIndex + 2, colIndex + 3, MathTools.powWithInteger(duration, 2));
            piecewiseTimeCoefficients.set(rowIndex + 2, colIndex + 4, MathTools.powWithInteger(duration, 3));

            piecewiseTimeCoefficients.set(rowIndex + 3, colIndex + 2, 1);
            piecewiseTimeCoefficients.set(rowIndex + 3, colIndex + 3, 2 * duration);
            piecewiseTimeCoefficients.set(rowIndex + 3, colIndex + 4, 3 * MathTools.powWithInteger(duration, 2));
            piecewiseTimeCoefficients.set(rowIndex + 3, colIndex + 6, -1);

            piecewiseTimeCoefficients.set(rowIndex + 4, colIndex + 3, 2);
            piecewiseTimeCoefficients.set(rowIndex + 4, colIndex + 4, 6 * duration);
            piecewiseTimeCoefficients.set(rowIndex + 4, colIndex + 7, -1);

            coefficientConstraints.set(rowIndex + 1, i, 1);
            coefficientConstraints.set(rowIndex + 2, i + 1, 1);

            rowIndex += 4;
            colIndex += 4;
         }

         duration = durations.get(numberOfKnots - 2);

         piecewiseTimeCoefficients.set(rowIndex + 1, colIndex + 1, 1);

         piecewiseTimeCoefficients.set(rowIndex + 2, colIndex + 1, 1);
         piecewiseTimeCoefficients.set(rowIndex + 2, colIndex + 2, duration);
         piecewiseTimeCoefficients.set(rowIndex + 2, colIndex + 3, MathTools.powWithInteger(duration, 2));
         piecewiseTimeCoefficients.set(rowIndex + 2, colIndex + 4, MathTools.powWithInteger(duration, 3));

         piecewiseTimeCoefficients.set(rowIndex + 3, colIndex + 2, 1);
         piecewiseTimeCoefficients.set(rowIndex + 3, colIndex + 3, 2 * duration);
         piecewiseTimeCoefficients.set(rowIndex + 3, colIndex + 4, 3 * MathTools.powWithInteger(duration, 2));

         coefficientConstraints.set(rowIndex + 1, numberOfKnots - 2, 1);
         coefficientConstraints.set(rowIndex + 2, numberOfKnots - 1, 1);
      }

      CommonOps.solve(piecewiseTimeCoefficients, coefficientConstraints, projectionMatrixToPack);
      projectionMatrixToPack.reshape(numberOfKnots-1, 4, true);

      projectionMatrixDotToPack.reshape(numberOfKnots-1, 3);
      projectionMatrixDotDotToPack.reshape(numberOfKnots-1, 2);

      for (int i = 0; i < numberOfKnots - 1; i++)
      {
         projectionMatrixDotToPack.set(i, 0, projectionMatrixToPack.get(i, 1));
         projectionMatrixDotToPack.set(i, 1, 2 * projectionMatrixToPack.get(i, 2));
         projectionMatrixDotToPack.set(i, 2, 3 * projectionMatrixToPack.get(i, 3));

         projectionMatrixDotDotToPack.set(i, 0, projectionMatrixDotToPack.get(i, 1));
         projectionMatrixDotDotToPack.set(i, 1, 2 * projectionMatrixDotToPack.get(i, 2));
      }
   }
}
