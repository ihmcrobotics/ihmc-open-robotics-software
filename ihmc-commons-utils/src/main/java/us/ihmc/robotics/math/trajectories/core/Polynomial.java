package us.ihmc.robotics.math.trajectories.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialBasics;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialReadOnly;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalBasics;

/**
 * Used to compute the value and 1st and 2nd order derivatives of a polynomial.
 * <p>
 * Note that the coefficients are always stored in ascending order. That is, if thiis a third order polynomial, they are stored as
 * y = c<sub>0</sub> + c<sub>1</sub> x + c<sub>2</sub> x<sup>2</sup>
 * </p>
 */
public class Polynomial implements PolynomialBasics
{
   private final int maximumNumberOfCoefficients;

   private final DMatrixRMaj constraintMatrix;
   private final DMatrixRMaj constraintVector;

   private final DMatrixRMaj coefficientsVector;
   private final DMatrixRMaj derivativeCoefficientsVector;
   private final DMatrixRMaj doubleDerivativeCoefficientsVector;
   private final DMatrixRMaj coefficientVectorCopy;
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

      this.constraintMatrix = new DMatrixRMaj(maxNumberOfCoefficients, maxNumberOfCoefficients);
      this.constraintVector = new DMatrixRMaj(maxNumberOfCoefficients, 1);

      this.coefficientsVector = new DMatrixRMaj(maxNumberOfCoefficients, 1);
      this.coefficientVectorCopy = new DMatrixRMaj(maxNumberOfCoefficients, 1);
      this.derivativeCoefficientsVector = new DMatrixRMaj(Math.max(maxNumberOfCoefficients, 0), 1);
      this.doubleDerivativeCoefficientsVector = new DMatrixRMaj(Math.max(maxNumberOfCoefficients, 0), 1);

      this.xPowersDerivativeVector = new DMatrixRMaj(1, maxNumberOfCoefficients);
      this.solver = LinearSolverFactory_DDRM.general(maxNumberOfCoefficients, maxNumberOfCoefficients);
      this.xPowers = new double[maxNumberOfCoefficients];
      reset();
   }

   public Polynomial(double... coefficients)
   {
      this(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, true, coefficients);
   }

   public Polynomial(boolean coefficientsAreLowestOrderFirst, double... coefficients)
   {
      this(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, coefficientsAreLowestOrderFirst, coefficients);
   }

   public Polynomial(double tInitial, double tFinal, boolean coefficientsAreLowestOrderFirst, double... coefficients)
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
   public Polynomial(PolynomialReadOnly other)
   {
      maximumNumberOfCoefficients = other.getMaximumNumberOfCoefficients();

      constraintMatrix = new DMatrixRMaj(0, maximumNumberOfCoefficients);
      constraintVector = new DMatrixRMaj(0, maximumNumberOfCoefficients);
      isConstraintMatrixUpToDate = false;

      coefficientsVector = new DMatrixRMaj(other.getCoefficientsVector());
      coefficientVectorCopy = new DMatrixRMaj(coefficientsVector);
      numberOfCoefficients = other.getNumberOfCoefficients();
      derivativeCoefficientsVector = new DMatrixRMaj(numberOfCoefficients, 1);
      doubleDerivativeCoefficientsVector = new DMatrixRMaj(numberOfCoefficients, 1);

      for (int coeffIdx = 0; coeffIdx < numberOfCoefficients; coeffIdx++)
      {
         derivativeCoefficientsVector.set(coeffIdx, 1, other.getDerivativeCoefficient(coeffIdx));
         doubleDerivativeCoefficientsVector.set(coeffIdx, 1, other.getDoubleDerivativeCoefficient(coeffIdx));
      }

      solver = LinearSolverFactory_DDRM.general(maximumNumberOfCoefficients, maximumNumberOfCoefficients);
      timeInterval.setInterval(other.getInitialTime(), other.getFinalTime());
      currentTime = other.getCurrentTime();

      this.xPowers = new double[maximumNumberOfCoefficients];
      this.xPowersDerivativeVector = new DMatrixRMaj(1, maximumNumberOfCoefficients);

      compute(currentTime);
   }

   /** Deep copy constructor */
   public Polynomial(Polynomial other)
   {
      maximumNumberOfCoefficients = other.maximumNumberOfCoefficients;
      constraintMatrix = new DMatrixRMaj(other.constraintMatrix);
      constraintVector = new DMatrixRMaj(other.constraintVector);

      coefficientsVector = new DMatrixRMaj(other.coefficientsVector);
      coefficientVectorCopy = new DMatrixRMaj(other.coefficientVectorCopy);
      derivativeCoefficientsVector = new DMatrixRMaj(other.derivativeCoefficientsVector);
      doubleDerivativeCoefficientsVector = new DMatrixRMaj(other.doubleDerivativeCoefficientsVector);

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
         f += coefficientsVector.get(i) * xPowers[i];
      for (int i = 0; i < numberOfCoefficients - 1; i++)
         df += derivativeCoefficientsVector.get(i) * xPowers[i];
      for (int i = 0; i < numberOfCoefficients - 2; i++)
         ddf += doubleDerivativeCoefficientsVector.get(i) * xPowers[i];
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
      return coefficientsVector.get(i);
   }

   @Override
   public double getDerivativeCoefficient(int i)
   {
      return derivativeCoefficientsVector.get(i);
   }

   @Override
   public double getDoubleDerivativeCoefficient(int i)
   {
      return doubleDerivativeCoefficientsVector.get(i);
   }

   @Override
   public void setNumberOfCoefficients(int numberOfCoefficients)
   {
      this.numberOfCoefficients = numberOfCoefficients;
      coefficientsVector.reshape(numberOfCoefficients, 1);
      derivativeCoefficientsVector.reshape(numberOfCoefficients, 1);
      doubleDerivativeCoefficientsVector.reshape(numberOfCoefficients, 1);
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
      setCoefficientsCopy();
      return coefficientVectorCopy;
   }

   public double[] getCoefficients()
   {
      setCoefficientsCopy();
      return coefficientVectorCopy.data;
   }

   public void solveForCoefficients()
   {
      solver.setA(constraintMatrix);
      solver.solve(constraintVector, coefficientsVector);
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
         this.coefficientsVector.reshape(power + 1, 1);
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
      coefficientVectorCopy.set(coefficientsVector);
   }

   public void reshape(int numberOfCoefficientsRequired)
   {
      if (numberOfCoefficientsRequired > getMaximumNumberOfCoefficients())
         throw new RuntimeException("Maximum number of coefficients is: " + getMaximumNumberOfCoefficients() + ", can't build the polynomial as it requires: "
               + numberOfCoefficientsRequired + " coefficients.");

      this.coefficientsVector.reshape(numberOfCoefficientsRequired, 1);
      this.constraintMatrix.reshape(numberOfCoefficientsRequired, numberOfCoefficientsRequired);
      this.constraintVector.reshape(numberOfCoefficientsRequired, 1);
      this.xPowersDerivativeVector.reshape(1, numberOfCoefficientsRequired);

      constraintMatrix.zero();
      constraintVector.zero();
      coefficientsVector.zero();

      setNumberOfCoefficients(numberOfCoefficientsRequired);

      for (int i = numberOfCoefficientsRequired; i < getMaximumNumberOfCoefficients(); i++)
         setCoefficient(i, Double.NaN);
   }

   @Override
   public void setCoefficientUnsafe(int idx, double value)
   {
      coefficientsVector.set(idx, value);
      if (idx > 0)
         derivativeCoefficientsVector.set(idx - 1, value * idx);
      if (idx > 1)
         doubleDerivativeCoefficientsVector.set(idx - 2, value * (idx - 1) * idx);
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