package us.ihmc.quadrupedRobotics.planning.trajectory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;

public class ExponentialAndCubicTrajectory
{
   private static final int numberOfCoefficients = 6;

   private final double[] coefficients;
   private final DenseMatrix64F constraintMatrix;
   private final DenseMatrix64F constraintVector;
   private final DenseMatrix64F coefficientVector;
   private final LinearSolver<DenseMatrix64F> solver;

   private double timeConstant;
   private double tInitial;
   private double tFinal;
   private double f, df, ddf;
   private final double[] xPowers;
   private final double[] xDotPowers;
   private final double[] xDDotPowers;

   public ExponentialAndCubicTrajectory()
   {
      this.coefficients = new double[numberOfCoefficients];
      this.constraintMatrix = new DenseMatrix64F(numberOfCoefficients, numberOfCoefficients);
      this.constraintVector = new DenseMatrix64F(numberOfCoefficients, 1);
      this.coefficientVector = new DenseMatrix64F(numberOfCoefficients, 1);
      this.solver = LinearSolverFactory.general(numberOfCoefficients, numberOfCoefficients);
      this.xPowers = new double[numberOfCoefficients];
      this.xDotPowers = new double[numberOfCoefficients];
      this.xDDotPowers = new double[numberOfCoefficients];
      reset();
   }

   public void reset()
   {
      timeConstant = Double.NaN;
      tInitial = Double.NaN;
      tFinal = Double.NaN;
      for (int i = 0; i < numberOfCoefficients; i++)
      {
         //xPowers[i] = Double.NaN;
         coefficients[i] = Double.NaN;
      }
      //xPowersDerivativeVector.zero();
   }


   public void compute(double time)
   {
      setXPowers(time);
      ddf = df = f = 0.0;
      for (int i = 0; i < numberOfCoefficients; i++)
         f += coefficients[i] * xPowers[i];
      for (int i = 1; i < numberOfCoefficients; i++)
         df += coefficients[i] * xDotPowers[i];
      for (int i = 2; i < numberOfCoefficients; i++)
         ddf += coefficients[i] * xDDotPowers[i];
   }

   /**
    * Sets the given array to be:
    * <br> [1, x, x<sup>2</sup>, ..., x<sup>N</sup>]
    * <br> where N+1 is the length of the given array
    *
    * @param time base of the power series
    */
   private void setXPowers(double time)
   {
      xPowers[0] = 1.0;
      xPowers[1] = time;
      xPowers[2] = time * time;
      xPowers[3] = xPowers[2] * time;

      xDotPowers[0] = 0.0;
      xDotPowers[1] = 1.0;
      xDotPowers[2] = time;
      xDotPowers[3] = time * time;

      xDDotPowers[0] = 0.0;
      xDDotPowers[1] = 0.0;
      xDDotPowers[2] = 1.0;
      xDDotPowers[3] = time;

      xPowers[4] = Math.exp(timeConstant * time);
      xPowers[5] = Math.exp(-timeConstant * time);

      xDotPowers[4] = timeConstant * xPowers[4];
      xDotPowers[5] = -timeConstant * xPowers[5];

      xDDotPowers[4] = timeConstant * xDotPowers[4];
      xDDotPowers[5] = -timeConstant * xDotPowers[5];
   }



   public double getPosition()
   {
      return f;
   }

   public double getVelocity()
   {
      return df;
   }

   public double getAcceleration()
   {
      return ddf;
   }


   public void setTime(double t0, double tFinal)
   {
      setInitialTime(t0);
      setFinalTime(tFinal);
   }

   public void setTimeConstant(double timeConstant)
   {
      this.timeConstant = timeConstant;
   }

   public void setFinalTime(double tFinal)
   {
      this.tFinal = tFinal;
   }

   public void setInitialTime(double t0)
   {
      this.tInitial = t0;
   }

   public double getFinalTime()
   {
      return this.tFinal;
   }

   public double getInitialTime()
   {
      return this.tInitial;
   }

   public double getDuration()
   {
      return tFinal - tInitial;
   }

   public boolean timeIntervalContains(double timeToCheck, double EPSILON)
   {
      return MathTools.intervalContains(timeToCheck, getInitialTime(), getFinalTime(), EPSILON);
   }

   public boolean timeIntervalContains(double timeToCheck)
   {
      return MathTools.intervalContains(timeToCheck, getInitialTime(), getFinalTime(), Epsilons.ONE_MILLIONTH);
   }

   public void setFromBounds(double t0, double tFinal, double timeConstant, double x0, double xd0, double vrp0, double xFinal, double xdFinal, double vrpFinal)
   {
      setTime(t0, tFinal);
      setTimeConstant(timeConstant);
      setPositionRow(0, t0, x0);
      setVelocityRow(1, t0, xd0);
      setVRPRow(2, t0, vrp0);
      setPositionRow(3, tFinal, xFinal);
      setVelocityRow(4, tFinal, xdFinal);
      setVRPRow(4, tFinal, vrpFinal);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void solveForCoefficients()
   {
      solver.setA(constraintMatrix);
      solver.solve(constraintVector, coefficientVector);
   }

   public void setCoefficientVariables()
   {
      for (int row = 0; row < numberOfCoefficients; row++)
         coefficients[row] = coefficientVector.get(row, 0);
   }

   private void setConstraintRow(int row, double time, int derivativeOrderWithPositionBeingZero)
   {
      setConstraintRow(row, 1.0, time, derivativeOrderWithPositionBeingZero);
   }

   private void setConstraintRow(int row, double constantMultiplier, double time, int derivativeOrderWithPositionBeingZero)
   {
      double timePower = 1.0;

      int col = derivativeOrderWithPositionBeingZero;
      for (; col < numberOfCoefficients - 2; col++)
      {
         double columnPower = 1.0;
         for (int i = 0; i < derivativeOrderWithPositionBeingZero; i++)
         {
            columnPower *= (col - i);
         }
         constraintMatrix.set(row, col, constantMultiplier * timePower * columnPower);
         timePower *= time;
      }

      double positiveCoefficientLead = 1.0;
      double negativeCoefficientLead = 1.0;
      for (int order = 0; order <= derivativeOrderWithPositionBeingZero; order++)
      {
         positiveCoefficientLead *= timeConstant;
         negativeCoefficientLead *= -timeConstant;
      }
      constraintMatrix.set(row, col, positiveCoefficientLead);
      constraintMatrix.set(row, col + 1, negativeCoefficientLead);
   }

   public void setConstraintValue(int row, double desiredValue)
   {
      constraintVector.set(row, 0, desiredValue);
   }

   private void setPositionRow(int row, double time, double xAtTime)
   {
      setConstraintRow(row, time, 0);
      setConstraintValue(row, xAtTime);
   }

   private void setVelocityRow(int row, double time, double xDotAtTime)
   {
      setConstraintRow(row, time, 1);
      setConstraintValue(row, xDotAtTime);
   }

   private void setVRPRow(int row, double time, double vrpAtTime)
   {
      setConstraintRow(row, time, 0);
      setConstraintRow(row, -1.0 / timeConstant, time, 1);
      setConstraintValue(row, vrpAtTime);
   }


}
