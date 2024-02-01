package us.ihmc.robotics.math.trajectories.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialBasics;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalBasics;

/**
 * Used to compute the value and 1st and 2nd order derivatives of a polynomial.
 */
public class Polynomial implements PolynomialBasics
{
   private final int maximumNumberOfCoefficients;
   private final double[] coefficients;
   private final double[] coefficientsCopy;
   private final DMatrixRMaj constraintMatrix;
   private final DMatrixRMaj constraintVector;
   private final DMatrixRMaj coefficientVector;
   private final LinearSolverDense<DMatrixRMaj> solver;

   private final TimeIntervalBasics timeInterval = new TimeInterval();
   private double currentTime;
   private int numberOfCoefficients;
   private double f, df, ddf;
   private final double[] xPowers;
   private final DMatrixRMaj xPowersDerivativeVector;

   private boolean isConstraintMatrixUpToDate = false;

   public Polynomial(int maxNumberOfCoefficients)
   {
      if (maxNumberOfCoefficients < 1)
         throw new IllegalArgumentException("You have to make the polynomial to have at least 1 coefficient!");
      this.maximumNumberOfCoefficients = maxNumberOfCoefficients;
      this.coefficients = new double[maxNumberOfCoefficients];
      this.coefficientsCopy = new double[maxNumberOfCoefficients];
      this.constraintMatrix = new DMatrixRMaj(maxNumberOfCoefficients, maxNumberOfCoefficients);
      this.constraintVector = new DMatrixRMaj(maxNumberOfCoefficients, 1);
      this.coefficientVector = new DMatrixRMaj(maxNumberOfCoefficients, 1);
      this.xPowersDerivativeVector = new DMatrixRMaj(1, maxNumberOfCoefficients);
      this.solver = LinearSolverFactory_DDRM.general(maxNumberOfCoefficients, maxNumberOfCoefficients);
      this.xPowers = new double[maxNumberOfCoefficients];
      reset();
   }

   public Polynomial(double constant)
   {
      this(new double[] {constant});
   }

   public Polynomial(double coefficient1, double constant)
   {
      this(new double[] {constant, coefficient1});
   }

   public Polynomial(double coefficient2, double coefficient1, double constant)
   {
      this(new double[] {constant, coefficient1, coefficient2});
   }

   public Polynomial(double coefficient3, double coefficient2, double coefficient1, double constant)
   {
      this(new double[] {constant, coefficient1, coefficient2, coefficient3});
   }

   public Polynomial(double tInitial, double tFinal, double[] coefficients)
   {
      this(tInitial, tFinal, coefficients, true);
   }

   public Polynomial(double[] coefficients)
   {
      this(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, coefficients, true);
   }

   public Polynomial(double[] coefficients, boolean coefficientsAreLowestOrderFirst)
   {
      this(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, coefficients, coefficientsAreLowestOrderFirst);
   }

   public Polynomial(double tInitial, double tFinal, double[] coefficients, boolean coefficientsAreLowestOrderFirst)
   {
      this(coefficients.length);

      setNumberOfCoefficients(coefficients.length);
      setTime(tInitial, tFinal);

      for (int i = 0; i < Math.min(getMaximumNumberOfCoefficients(), getNumberOfCoefficients()); i++)
      {
         if (coefficientsAreLowestOrderFirst)
            setCoefficient(i, coefficients[i]);
         else
            setCoefficient(i, coefficients[coefficients.length - 1 - i]);
      }
   }

   /** Deep copy constructor */
   public Polynomial(Polynomial other)
   {
      maximumNumberOfCoefficients = other.maximumNumberOfCoefficients;
      coefficients = other.coefficients.clone();
      coefficientsCopy = other.coefficientsCopy.clone();
      constraintMatrix = new DMatrixRMaj(other.constraintMatrix);
      constraintVector = new DMatrixRMaj(other.constraintVector);
      coefficientVector = new DMatrixRMaj(other.coefficientVector);
      solver = LinearSolverFactory_DDRM.general(other.maximumNumberOfCoefficients, other.maximumNumberOfCoefficients);
      timeInterval.setInterval(other.timeInterval.getStartTime(), other.timeInterval.getEndTime());
      currentTime = other.currentTime;
      numberOfCoefficients = other.numberOfCoefficients;
      f = other.f;
      df = other.df;
      ddf = other.ddf;
      xPowers = other.xPowers.clone();
      xPowersDerivativeVector = new DMatrixRMaj(other.xPowersDerivativeVector);
      isConstraintMatrixUpToDate = other.isConstraintMatrixUpToDate;
   }

   @Override
   public void compute(double x)
   {
      this.currentTime = x;
      PolynomialTools.setXPowers(xPowers, x);
      ddf = df = f = 0.0;
      for (int i = 0; i < numberOfCoefficients; i++)
         f += coefficients[i] * xPowers[i];
      for (int i = 1; i < numberOfCoefficients; i++)
         df += coefficients[i] * (i) * xPowers[i - 1];
      for (int i = 2; i < numberOfCoefficients; i++)
         ddf += coefficients[i] * (i - 1) * (i) * xPowers[i - 2];
   }


   @Override
   public double getValue()
   {
      return f;
   }

   @Override
   public double getVelocity()
   {
      return df;
   }

   @Override
   public double getAcceleration()
   {
      return ddf;
   }

   @Override
   public double getCoefficient(int i)
   {
      return coefficients[i];
   }

   @Override
   public void setNumberOfCoefficients(int numberOfCoefficients)
   {
      this.numberOfCoefficients = numberOfCoefficients;
   }

   @Override
   public void setCurrentTime(double currentTime)
   {
      this.currentTime = currentTime;
   }

   @Override
   public int getNumberOfCoefficients()
   {
      return numberOfCoefficients;
   }

   @Override
   public double getCurrentTime()
   {
      return currentTime;
   }

   @Override
   public int getMaximumNumberOfCoefficients()
   {
      return maximumNumberOfCoefficients;
   }

   public DMatrixRMaj getCoefficientsVector()
   {
      return coefficientVector;
   }

   public double[] getCoefficients()
   {
      setCoefficientsCopy();
      return coefficientsCopy;
   }

   public void solveForCoefficients()
   {
      solver.setA(constraintMatrix);
      solver.solve(constraintVector, coefficientVector);
   }

   @Override
   public void setIsConstraintMatrixUpToDate(boolean isConstraintMatrixUpToDate)
   {
      this.isConstraintMatrixUpToDate = isConstraintMatrixUpToDate;
   }

   @Override
   public boolean isConstraintMatrixUpToDate()
   {
      return isConstraintMatrixUpToDate;
   }

   public void setDirectly(int power, double coefficient)
   {
      if (power >= getMaximumNumberOfCoefficients())
         throw new RuntimeException("Maximum number of coefficients is: " + maximumNumberOfCoefficients + ", can't set coefficient as it requires: " + power + 1
               + " coefficients");

      if (power >= getNumberOfCoefficients())
      {
         for (int i = getNumberOfCoefficients(); i <= power; i++)
            this.coefficients[i] = 0.0;
         this.coefficientVector.reshape(power + 1, 1);
         this.constraintMatrix.reshape(power + 1, power + 1);
         this.constraintVector.reshape(power + 1, 1);
         this.xPowersDerivativeVector.reshape(1, power + 1);
         numberOfCoefficients = power + 1;
      }
      setCoefficient(power, coefficient);
   }


   public void setConstraintRow(int row, double x, double desiredZDerivative, int derivativeOrderWithPositionBeingZero)
   {
      double xPower = 1.0;

      for (int col = derivativeOrderWithPositionBeingZero; col < numberOfCoefficients; col++)
      {
         double columnPower = 1.0;
         for (int i = 0; i < derivativeOrderWithPositionBeingZero; i++)
         {
            columnPower *= (col - i);
         }
         constraintMatrix.set(row, col, xPower * columnPower);
         xPower *= x;
      }

      constraintVector.set(row, 0, desiredZDerivative);
   }

   private void setCoefficientsCopy()
   {
      int row = 0;
      for (; row < getNumberOfCoefficients(); row++)
         coefficientsCopy[row] = getCoefficient(row);
      for (; row < getMaximumNumberOfCoefficients(); row++)
         coefficientsCopy[row] = Double.NaN;
   }

   public void reshape(int numberOfCoefficientsRequired)
   {
      if (numberOfCoefficientsRequired > getMaximumNumberOfCoefficients())
         throw new RuntimeException("Maximum number of coefficients is: " + getMaximumNumberOfCoefficients() + ", can't build the polynomial as it requires: "
               + numberOfCoefficientsRequired + " coefficients.");

      this.coefficientVector.reshape(numberOfCoefficientsRequired, 1);
      this.constraintMatrix.reshape(numberOfCoefficientsRequired, numberOfCoefficientsRequired);
      this.constraintVector.reshape(numberOfCoefficientsRequired, 1);
      this.xPowersDerivativeVector.reshape(1, numberOfCoefficientsRequired);

      constraintMatrix.zero();
      constraintVector.zero();
      coefficientVector.zero();

      setNumberOfCoefficients(numberOfCoefficientsRequired);

      for (int i = numberOfCoefficientsRequired; i < getMaximumNumberOfCoefficients(); i++)
         setCoefficient(i, Double.NaN);
   }

   @Override
   public void setCoefficientUnsafe(int idx, double value)
   {
      coefficients[idx] = value;
   }

   public String toString()
   {
      double tInitial = getTimeInterval().getStartTime();
      double tFinal = getTimeInterval().getEndTime();
      String inString = "Polynomial: " + getCoefficient(0);
      for (int i = 1; i < getNumberOfCoefficients(); i++)
      {
         inString += " ";
         if (getCoefficient(i) >= 0)
            inString += "+";
         inString += getCoefficient(i) + " x^" + i;
      }
      return inString + " TInitial: " + tInitial + " TFinal: " + tFinal;
   }

   @Override
   public TimeIntervalBasics getTimeInterval()
   {
      return timeInterval;
   }
}