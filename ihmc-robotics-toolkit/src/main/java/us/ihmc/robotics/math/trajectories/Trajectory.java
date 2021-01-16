package us.ihmc.robotics.math.trajectories;

import org.apache.commons.lang3.ArrayUtils;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalBasics;

/**
 * Simple trajectory class. Does not use the {@code Polynomial} class since its weird
 * Coefficients are stored with lowest order at the lowest index (what else would you do?)
 */
public class Trajectory implements PolynomialInterface
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

   public Trajectory(int maxNumberOfCoefficients)
   {
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

   public Trajectory(double tInitial, double tFinal, double[] coefficents)
   {
      this(coefficents.length);
      this.numberOfCoefficients = coefficents.length;
      setTime(tInitial, tFinal);
      for (int i = 0; i < maximumNumberOfCoefficients; i++)
         this.coefficients[i] = coefficents[i];
   }

   public void reset()
   {
      getTimeInterval().reset();
      numberOfCoefficients = 0;
      for (int i = 0; i < maximumNumberOfCoefficients; i++)
      {
         coefficients[i] = Double.NaN;
      }
   }

   @Override
   public void compute(double x)
   {
      this.currentTime = x;
      setXPowers(xPowers, x);
      ddf = df = f = 0.0;
      for (int i = 0; i < numberOfCoefficients; i++)
         f += coefficients[i] * xPowers[i];
      for (int i = 1; i < numberOfCoefficients; i++)
         df += coefficients[i] * (i) * xPowers[i - 1];
      for (int i = 2; i < numberOfCoefficients; i++)
         ddf += coefficients[i] * (i - 1) * (i) * xPowers[i - 2];
   }

   @Override
   public boolean isDone()
   {
      return currentTime >= getTimeInterval().getEndTime();
   }

   /**
    * Sets the given array to be:
    * <br> [1, x, x<sup>2</sup>, ..., x<sup>N</sup>]
    * <br> where N+1 is the length of the given array
    *
    * @param xPowers vector to set
    * @param x base of the power series
    */
   private static void setXPowers(double[] xPowers, double x)
   {
      xPowers[0] = 1.0;
      for (int i = 1; i < xPowers.length; i++)
      {
         xPowers[i] = xPowers[i - 1] * x;
      }
   }

   public double getDerivative(int derivativeOrder, double x)
   {
      double derivative = 0.0;
      double xPower = 1.0;
      for (int i = derivativeOrder; i < numberOfCoefficients; i++)
      {
         derivative += coefficients[i] * getCoefficientMultiplierForDerivative(derivativeOrder, i) * xPower;
         xPower *= x;
      }
      return derivative;
   }

   /**
    * Given the following vector and it's n-th derivative:
    * <br> v(x) = [ 1, x, ... , x<sup>N-1</sup>, x<sup>N</sup> ]
    * <br> d<sup>n</sup>v(x)/dx<sup>n</sup> = [ a<sub>0</sub>, a<sub>1</sub>x , ... , a<sub>N-n-1</sub>x<sup>N-n-1</sup> , a<sub>N-n</sub>x<sup>N-n</sup>, ... , 0, 0 ]
    *
    * <br> This method returns a matrix such that matrixToPack.get(i) returns the i-th element of d<sup>n</sup>v(x)/dx<sup>n</sup> evaluated at x0
    *
    * @param order highest order exponent, the value N in the above equation
    * @param x0 value at which the derivative is evaluated
    */
   public DMatrixRMaj evaluateGeometricSequenceDerivative(int order, double x0)
   {
      xPowersDerivativeVector.zero();

      double x0Power = 1.0;

      for (int i = order; i < numberOfCoefficients; i++)
      {
         xPowersDerivativeVector.set(i, getCoefficientMultiplierForDerivative(order, i) * x0Power);
         x0Power *= x0;
      }
      return xPowersDerivativeVector;
   }

   /**
    * Computes the coefficient resulting for computing the <tt>n</tt><sup>th</sup> derivative of <tt>x<sup>k</sup></tt>:
    * <pre>
    * d<sup>n</sup>x<sup>k</sup>    k!
    * --- = ------ x<sup>k-n</sup>
    * dx<sup>n</sup>   (k-n)!
    * </pre>
    * The coefficient computed here is: <tt>(k! / (k-n)!)</tt>
    *
    * @param order the order of the derivative, i.e. the variable <tt>n</tt>.
    * @param exponent the exponent of the power, i.e. the variable <tt>k</tt>.
    * @return the coefficient <tt>(k! / (k-n)!)</tt>
    */
   private static int getCoefficientMultiplierForDerivative(int order, int exponent)
   {
      int coeff = 1;
      for (int i = exponent - order + 1; i <= exponent; i++)
      {
         coeff *= i;
      }
      return coeff;
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

   public double getCoefficient(int i)
   {
      setCoefficientsCopy();
      return coefficientsCopy[i];
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

   public void set(Trajectory other)
   {
      reset();
      getTimeInterval().set(other.getTimeInterval());
      reshape(other.getNumberOfCoefficients());
      int index = 0;
      for (; index < other.getNumberOfCoefficients(); index++)
         coefficients[index] = other.getCoefficient(index);
      for (; index < maximumNumberOfCoefficients; index++)
         coefficients[index] = Double.NaN;
   }

   public void setZero()
   {
      this.setConstant(0.0);
   }


   public void solveForCoefficients()
   {
      solver.setA(constraintMatrix);
      solver.solve(constraintVector, coefficientVector);
   }

   public void setDirectly(DMatrixRMaj coefficients)
   {
      reshape(coefficients.getNumRows());
      int index = 0;
      for (; index < numberOfCoefficients; index++)
         this.coefficients[index] = coefficients.get(index, 0);
      for (; index < maximumNumberOfCoefficients; index++)
         this.coefficients[index] = Double.NaN;
   }

   public void setDirectly(double[] coefficients)
   {
      reshape(coefficients.length);
      int index = 0;
      for (; index < numberOfCoefficients; index++)
         this.coefficients[index] = coefficients[index];
      for (; index < maximumNumberOfCoefficients; index++)
         this.coefficients[index] = Double.NaN;
   }

   public void setDirectly(int power, double coefficient)
   {
      if (power >= maximumNumberOfCoefficients)
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
      coefficients[power] = coefficient;
   }

   public void setInitialTimeMaintainingBounds(double tInitial)
   {
      int numStartingConstraints = (int) Math.ceil(getNumberOfCoefficients() / 2.0);
      int numEndingConstraints = getNumberOfCoefficients() - numStartingConstraints;

      int constraintNumber = 0;
      for (int order = 0; order < numStartingConstraints; order++, constraintNumber++)
      {
         double value = getDerivative(order, getTimeInterval().getStartTime());
         setConstraintRow(constraintNumber, tInitial, value, order);
      }
      for (int order = 0; order < numEndingConstraints; order++, constraintNumber++)
      {
         double value = getDerivative(order, getTimeInterval().getEndTime());
         setConstraintRow(constraintNumber, getTimeInterval().getEndTime(), value, order);
      }

      solveForCoefficients();
      initialize();

      getTimeInterval().setStartTime(tInitial);
   }

   public void setFinalTimeMaintainingBounds(double tFinal)
   {
      int numStartingConstraints = (int) Math.ceil(getNumberOfCoefficients() / 2.0);
      int numEndingConstraints = getNumberOfCoefficients() - numStartingConstraints;

      int constraintNumber = 0;
      for (int order = 0; order < numStartingConstraints; order++, constraintNumber++)
      {
         double value = getDerivative(order, getTimeInterval().getStartTime());
         setConstraintRow(constraintNumber, getTimeInterval().getStartTime(), value, order);
      }
      for (int order = 0; order < numEndingConstraints; order++, constraintNumber++)
      {
         double value = getDerivative(order, getTimeInterval().getEndTime());
         setConstraintRow(constraintNumber, tFinal, value, order);
      }

      solveForCoefficients();
      initialize();


      getTimeInterval().setEndTime(tFinal);
   }

   /**
    * Set a specific coefficient of the polynomial. A sequence of calls to this function should typically be followed by a call to {@code reshape(int)} later.
    * @param power
    * @param coefficient
    */
   public void setDirectlyFast(int power, double coefficient)
   {
      if (power >= maximumNumberOfCoefficients)
         return;
      if (power >= getNumberOfCoefficients())
         numberOfCoefficients = power + 1;
      this.coefficients[power] = coefficient;
   }

   public void setDirectlyReverse(double[] coefficients)
   {
      ArrayUtils.reverse(coefficients);
      setDirectly(coefficients);
   }

   public void offsetTrajectoryPosition(double offsetValue)
   {
      coefficients[0] += offsetValue;
   }

   /**
    * Dont use this. It creates garbage
    * @param from
    * @param to
    * @return
    */
   public double getIntegral(double from, double to)
   {
      double[] fromPowers = new double[numberOfCoefficients + 1];
      double[] toPowers = new double[numberOfCoefficients + 1];
      setXPowers(fromPowers, from);
      setXPowers(toPowers, to);
      double integral = 0;
      for (int i = 0; i < numberOfCoefficients; i++)
      {
         integral += (1.0 / ((double) i + 1.0)) * this.coefficients[i] * (toPowers[i + 1] - fromPowers[i + 1]);
      }
      return integral;
   }

   public int getNumberOfCoefficients()
   {
      return numberOfCoefficients;
   }

   public int getMaximumNumberOfCoefficients()
   {
      return maximumNumberOfCoefficients;
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

   @Override
   public void initialize()
   {
      solveForCoefficients();

      int row = 0;
      for (; row < numberOfCoefficients; row++)
         coefficients[row] = coefficientVector.get(row, 0);
      for (; row < maximumNumberOfCoefficients; row++)
         coefficients[row] = Double.NaN;
   }

   private void setCoefficientsCopy()
   {
      for (int row = 0; row < numberOfCoefficients; row++)
         coefficientsCopy[row] = coefficients[row];
   }

   public void reshape(int numberOfCoefficientsRequired)
   {
      if (numberOfCoefficientsRequired > maximumNumberOfCoefficients)
         throw new RuntimeException("Maximum number of coefficients is: " + maximumNumberOfCoefficients + ", can't build the polynomial as it requires: "
               + numberOfCoefficientsRequired + " coefficients.");

      this.coefficientVector.reshape(numberOfCoefficientsRequired, 1);
      this.constraintMatrix.reshape(numberOfCoefficientsRequired, numberOfCoefficientsRequired);
      this.constraintVector.reshape(numberOfCoefficientsRequired, 1);
      this.xPowersDerivativeVector.reshape(1, numberOfCoefficientsRequired);
      numberOfCoefficients = numberOfCoefficientsRequired;

      for (int i = numberOfCoefficientsRequired; i < maximumNumberOfCoefficients; i++)
         this.coefficients[i] = Double.NaN;
   }

   public String toString()
   {
      double tInitial = getTimeInterval().getStartTime();
      double tFinal = getTimeInterval().getEndTime();
      String inString = "Polynomial: " + coefficients[0];
      for (int i = 1; i < getNumberOfCoefficients(); i++)
      {
         inString += " ";
         if (coefficients[i] >= 0)
            inString += "+";
         inString += coefficients[i] + " x^" + i;
      }
      return inString + " TInitial: " + tInitial + " TFinal: " + tFinal;
   }

   public String toString2()
   {
      double tInitial = getTimeInterval().getStartTime();
      double tFinal = getTimeInterval().getEndTime();
      compute(tInitial);
      String retString = "TInitial: " + tInitial + " Val: " + f;
      compute(tFinal);
      retString = retString + " TFinal: " + tFinal + " Val: " + f;
      return retString;
   }

   public boolean isValidTrajectory()
   {
      boolean retVal = (getInitialTime() < getFinalTime()) && Double.isFinite(getInitialTime()) && Double.isFinite(getFinalTime());
      for (int i = 0; retVal && i < numberOfCoefficients; i++)
         retVal &= Double.isFinite(coefficients[i]);
      return retVal;
   }

   @Override
   public TimeIntervalBasics getTimeInterval()
   {
      return timeInterval;
   }
}