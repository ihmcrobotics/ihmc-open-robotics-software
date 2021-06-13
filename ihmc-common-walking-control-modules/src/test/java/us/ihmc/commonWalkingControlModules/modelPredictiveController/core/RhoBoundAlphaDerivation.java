package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.log.LogTools;

import java.util.Random;

public class RhoBoundAlphaDerivation
{
   private static final double omega = 3.0;
   private static final double dt = 1e-4;
   private static final double epsilon = 1e-7;
   private static final int iters = 1000;

   private static void computeNecessaryAlpha()
   {
      Random random = new Random(1738L);
      double minAlpha = Double.POSITIVE_INFINITY;
      double maxAlpha = 0.0;

      for (int iter = 0; iter < iters; iter++)
      {
         double startPosition = RandomNumbers.nextDouble(random, 10.0);
         double endPosition = RandomNumbers.nextDouble(random, 10.0);
         double startVelocity = RandomNumbers.nextDouble(random, 100.0);
         double endVelocity = RandomNumbers.nextDouble(random, 100.0);

         double duration = RandomNumbers.nextDouble(random, 0.01, 2.0);

         DMatrixRMaj coefficients = solveForCoefficients(startPosition, endPosition, startVelocity, endVelocity, omega, duration);
         double[] minMax = computeMinAndMaxOfFunction(coefficients, omega, duration);

         double neededAlpha = solveForAlphaForControlPointsToBound(minMax, startPosition, endPosition, startVelocity, endVelocity);

         if (!Double.isNaN(neededAlpha))
         {
            minAlpha = Math.min(neededAlpha, minAlpha);
            maxAlpha = Math.max(neededAlpha, maxAlpha);
         }
      }

      LogTools.info("Alpha bounds are: min = " + minAlpha + ", max = " + maxAlpha);
   }

   private static DMatrixRMaj solveForCoefficients(double startPosition,
                                                   double endPosition,
                                                   double startVelocity,
                                                   double endVelocity,
                                                   double omega,
                                                   double duration)
   {
      DMatrixRMaj constraintMatrix = new DMatrixRMaj(4, 4);
      DMatrixRMaj objectiveMatrix = new DMatrixRMaj(4, 1);

      // constrain initial position
      constraintMatrix.set(0, 0, omega * omega);
      constraintMatrix.set(0, 1, omega * omega);
      constraintMatrix.set(0, 3, 2.0);

      objectiveMatrix.set(0, 0, startPosition);

      // constrain initial rate
      constraintMatrix.set(1, 0, omega * omega * omega);
      constraintMatrix.set(1, 1, -omega * omega * omega);
      constraintMatrix.set(1, 2, 6.0);

      objectiveMatrix.set(1, 0, startVelocity);

      // constrain final position
      constraintMatrix.set(2, 0, omega * omega * Math.exp(omega * duration));
      constraintMatrix.set(2, 1, omega * omega * Math.exp(-omega * duration));
      constraintMatrix.set(2, 2, 6 * duration);
      constraintMatrix.set(2, 3, 2.0);

      objectiveMatrix.set(2, 0, endPosition);

      // constrain final velocity
      constraintMatrix.set(3, 0, omega * omega * omega * Math.exp(omega * duration));
      constraintMatrix.set(3, 1, -omega * omega * omega * Math.exp(-omega * duration));
      constraintMatrix.set(3, 2, 6.0);

      objectiveMatrix.set(3, 0, endVelocity);

      DMatrixRMaj coefficients = new DMatrixRMaj(4, 1);
      CommonOps_DDRM.solve(constraintMatrix, objectiveMatrix, coefficients);

      return coefficients;
   }

   private static double[] computeMinAndMaxOfFunction(DMatrixRMaj coefficients, double omega, double duration)
   {
      double maxValue = Double.NEGATIVE_INFINITY;
      double minValue = Double.POSITIVE_INFINITY;
      for (double time = 0.0; time <= duration; time += dt)
      {
         double value = omega * omega * (Math.exp(omega * time) * coefficients.get(0) + Math.exp(-omega * time) * coefficients.get(1));
         value += 6.0 * time + 2.0;

         maxValue = Math.max(value, maxValue);
         minValue = Math.min(value, minValue);
      }

      return new double[] {minValue, maxValue};
   }

   private static double solveForAlphaForControlPointsToBound(double[] minMax, double startPosition, double startVelocity, double endPosition, double endVelocity)
   {
      boolean minIsBound = MathTools.epsilonEquals(minMax[0], Math.min(startPosition, endPosition), epsilon);
      boolean maxIsBound = MathTools.epsilonEquals(minMax[1], Math.max(startPosition, endPosition), epsilon);

      if (minIsBound && maxIsBound)
         return Double.NaN;

      double alphaForPoint2;
      double alphaForPoint3;

      if (startVelocity > 0.0)
      { // point 2 could be a max
         alphaForPoint2 = startVelocity / (minMax[1] - startPosition);
      }
      else
      { // point 2 could be a min
         alphaForPoint2 = startVelocity / (minMax[0] - startPosition);
      }

      if (endVelocity < 0.0)
      { // point 3 could be a max
         alphaForPoint3 = endVelocity / (endPosition - minMax[1]);
      }
      else
      { // point 3 could be a min
         alphaForPoint3 = endVelocity / (endPosition - minMax[0]);
      }

      double candidateAlpha = Math.max(alphaForPoint2, alphaForPoint3);

      double point1 = startPosition;
      double point2 = startPosition + 1.0 / candidateAlpha * startVelocity;
      double point3 = endVelocity - 1.0 / candidateAlpha * endVelocity;
      double point4 = endPosition;

      double minPoint = MathTools.min(new double[]{point1, point2, point3, point4});
      double maxPoint = MathTools.max(new double[]{point1, point2, point3, point4});

      if (minPoint > minMax[0] + epsilon)
         throw new IllegalArgumentException("Invalid alpha.");
      if (maxPoint < minMax[1] - epsilon)
         throw new IllegalArgumentException("Invalid alpha.");

      return candidateAlpha;
   }

   public static void main(String[] args)
   {
      computeNecessaryAlpha();
   }
}
