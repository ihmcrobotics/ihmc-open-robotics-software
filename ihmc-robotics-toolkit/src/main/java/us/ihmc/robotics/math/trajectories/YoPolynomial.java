package us.ihmc.robotics.math.trajectories;

import org.apache.commons.lang3.ArrayUtils;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolynomial3D.PolynomialVariableHolder;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoPolynomial implements PolynomialVariableHolder, PolynomialInterface
{
   private final int maximumNumberOfCoefficients;
   private double pos, vel, acc, jerk, dPos;
   private final YoDouble[] a;
   private final YoInteger numberOfCoefficients;
   private final DMatrixRMaj constraintMatrix;
   private final DMatrixRMaj constraintVector;
   private final DMatrixRMaj coefficientVector;
   private final double[] xPowers;

   private final TimeIntervalBasics timeInterval = new TimeInterval();
   private double tCurrent;


   // Stores the (n-th order) derivative of the xPowers vector
   private final DMatrixRMaj xPowersDerivativeVector;

   private final LinearSolverDense<DMatrixRMaj> solver;

   public YoPolynomial(String name, int maximumNumberOfCoefficients, YoRegistry registry)
   {
      this.maximumNumberOfCoefficients = maximumNumberOfCoefficients;

      solver = LinearSolverFactory_DDRM.general(maximumNumberOfCoefficients, maximumNumberOfCoefficients);

      a = new YoDouble[maximumNumberOfCoefficients];
      constraintMatrix = new DMatrixRMaj(maximumNumberOfCoefficients, maximumNumberOfCoefficients);
      constraintVector = new DMatrixRMaj(maximumNumberOfCoefficients, 1);
      coefficientVector = new DMatrixRMaj(maximumNumberOfCoefficients, 1);
      xPowers = new double[maximumNumberOfCoefficients];

      xPowersDerivativeVector = new DMatrixRMaj(maximumNumberOfCoefficients, 1);

      numberOfCoefficients = new YoInteger(name + "_nCoeffs", registry);

      for (int i = 0; i < maximumNumberOfCoefficients; i++)
      {
         a[i] = new YoDouble(name + "_a" + i, registry);
      }
   }

   public YoPolynomial(YoDouble[] coefficients, YoInteger numberOfCoefficients)
   {
      a = coefficients;
      this.numberOfCoefficients = numberOfCoefficients;

      maximumNumberOfCoefficients = coefficients.length;

      solver = LinearSolverFactory_DDRM.general(maximumNumberOfCoefficients, maximumNumberOfCoefficients);

      constraintMatrix = new DMatrixRMaj(maximumNumberOfCoefficients, maximumNumberOfCoefficients);
      constraintVector = new DMatrixRMaj(maximumNumberOfCoefficients, 1);
      coefficientVector = new DMatrixRMaj(maximumNumberOfCoefficients, 1);
      xPowers = new double[maximumNumberOfCoefficients];

      xPowersDerivativeVector = new DMatrixRMaj(maximumNumberOfCoefficients, 1);
   }

   @Override
   public double getValue()
   {
      return pos;
   }

   @Override
   public double getVelocity()
   {
      return vel;
   }

   @Override
   public double getAcceleration()
   {
      return acc;
   }

   public double getJerk()
   {
      return jerk;
   }

   public double getCoefficient(int i)
   {
      return a[i].getDoubleValue();
   }

   /**
    * To be used only for testing. Garbage creating function
    */
   public double[] getCoefficients()
   {
      double[] ret = new double[numberOfCoefficients.getIntegerValue()];
      for (int i = 0; i < numberOfCoefficients.getIntegerValue(); i++)
      {
         ret[i] = a[i].getDoubleValue();
      }
      return ret;
   }

   public DMatrixRMaj getCoefficientsVector()
   {
      return coefficientVector;
   }

   @Override
   public YoDouble[] getYoCoefficients()
   {
      return a;
   }

   public void reset()
   {
      getTimeInterval().reset();
      numberOfCoefficients.set(0);
      for (int i = 0; i < maximumNumberOfCoefficients; i++)
         a[i].setToNaN();
   }

   public void set(YoPolynomial other)
   {
      reset();

      reshape(other.getNumberOfCoefficients());

      for (int i = 0; i < other.getNumberOfCoefficients(); i++)
         a[i].set(other.getCoefficient(i));
   }

   protected void solveForCoefficients()
   {
      solver.setA(constraintMatrix);
      solver.solve(constraintVector, coefficientVector);
   }

   public void setDirectly(DMatrixRMaj coefficients)
   {
      reshape(coefficients.getNumRows());
      for (int i = 0; i < numberOfCoefficients.getIntegerValue(); i++)
      {
         a[i].set(coefficients.get(i, 0));
      }
   }

   public void setDirectly(double[] coefficients)
   {
      reshape(coefficients.length);
      for (int i = 0; i < numberOfCoefficients.getIntegerValue(); i++)
      {
         a[i].set(coefficients[i]);
      }
   }

   public void setDirectly(int power, double coefficient)
   {
      if (power >= maximumNumberOfCoefficients)
         throw new RuntimeException("Maximum number of coefficients is: " + maximumNumberOfCoefficients + ", can't set coefficient as it requires: " + power + 1
               + " coefficients");

      if (power >= getNumberOfCoefficients())
      {
         for (int i = getNumberOfCoefficients(); i <= power; i++)
            a[i].set(0.0);
         coefficientVector.reshape(power + 1, 1);
         constraintMatrix.reshape(power + 1, power + 1);
         constraintVector.reshape(power + 1, 1);
         xPowersDerivativeVector.reshape(power + 1, 1);
         numberOfCoefficients.set(power + 1);
      }
      a[power].set(coefficient);
   }

   /**
    * Set a specific coefficient of the polynomial. A sequence of calls to this function should
    * typically be followed by a call to {@code reshape(int)} later.
    *
    * @param power
    * @param coefficient
    */
   public void setDirectlyFast(int power, double coefficient)
   {
      if (power >= maximumNumberOfCoefficients)
         return;
      if (power >= getNumberOfCoefficients())
         numberOfCoefficients.set(power + 1);
      a[power].set(coefficient);
   }

   public void setDirectlyReverse(double[] coefficients)
   {
      ArrayUtils.reverse(coefficients);
      setDirectly(coefficients);
   }

   @Override
   public void compute(double x)
   {
      tCurrent = x;
      setXPowers(xPowers, x);

      pos = vel = acc = jerk = 0.0;
      for (int i = 0; i < numberOfCoefficients.getIntegerValue(); i++)
      {
         pos += a[i].getDoubleValue() * xPowers[i];
      }

      for (int i = 1; i < numberOfCoefficients.getIntegerValue(); i++)
      {
         vel += i * a[i].getDoubleValue() * xPowers[i - 1];
      }

      for (int i = 2; i < numberOfCoefficients.getIntegerValue(); i++)
      {
         acc += (i - 1) * i * a[i].getDoubleValue() * xPowers[i - 2];
      }

      for (int i = 3; i < numberOfCoefficients.getIntegerValue(); i++)
      {
         jerk += (i - 2) * (i - 1) * i * a[i].getDoubleValue() * xPowers[i - 3];
      }
   }

   public double getIntegral(double from, double to)
   {
      double[] fromPowers = new double[numberOfCoefficients.getIntegerValue() + 1];
      double[] toPowers = new double[numberOfCoefficients.getIntegerValue() + 1];
      setXPowers(fromPowers, from);
      setXPowers(toPowers, to);
      double integral = 0;
      for (int i = 0; i < numberOfCoefficients.getIntegerValue(); i++)
      {
         integral += 1.0 / (i + 1.0) * a[i].getDoubleValue() * (toPowers[i + 1] - fromPowers[i + 1]);
      }
      return integral;
   }

   public void setXPowers(double[] xPowers, double x)
   {
      xPowers[0] = 1.0;
      for (int i = 1; i < xPowers.length; i++)
      {
         xPowers[i] = xPowers[i - 1] * x;
      }
   }

   // Returns the order-th derivative of the polynomial at x
   public double getDerivative(int order, double x)
   {
      setXPowers(xPowers, x);

      dPos = 0.0;
      int derivativeCoefficient = 0;
      for (int i = order; i < numberOfCoefficients.getIntegerValue(); i++)
      {
         derivativeCoefficient = getDerivativeCoefficient(order, i);
         dPos += derivativeCoefficient * a[i].getDoubleValue() * xPowers[i - order];
      }
      return dPos;
   }

   /**
    * Returns the order-th derivative of the xPowers vector at value x (Note: does NOT return the
    * YoPolynomials order-th derivative at x)
    *
    * @param order
    * @param x
    * @return
    */
   public DMatrixRMaj getXPowersDerivativeVector(int order, double x)
   {
      setXPowers(xPowers, x);
      xPowersDerivativeVector.zero();

      int derivativeCoefficient = 0;
      for (int i = order; i < numberOfCoefficients.getIntegerValue(); i++)
      {
         derivativeCoefficient = getDerivativeCoefficient(order, i);
         xPowersDerivativeVector.set(i, derivativeCoefficient * xPowers[i - order]);
      }

      return xPowersDerivativeVector;
   }

   // Returns the constant coefficient at the exponent-th entry of the order-th derivative vector
   // Example: order = 4, exponent = 5 ==> returns 5*4*3*2
   public int getDerivativeCoefficient(int order, int exponent)
   {
      int coeff = 1;
      for (int i = exponent; i > exponent - order; i--)
      {
         coeff *= i;
      }
      return coeff;
   }

   public int getNumberOfCoefficients()
   {
      return numberOfCoefficients.getIntegerValue();
   }

   @Override
   public YoInteger getYoNumberOfCoefficients()
   {
      return numberOfCoefficients;
   }

   @Override
   public int getMaximumNumberOfCoefficients()
   {
      return maximumNumberOfCoefficients;
   }

   @Override
   public void setConstraintRow(int row, double x, double desiredZDerivative, int derivativeOrderWithPositionBeingZero)
   {
      double xPower = 1.0;

      for (int col = derivativeOrderWithPositionBeingZero; col < numberOfCoefficients.getIntegerValue(); col++)
      {
         double columnPower = 1.0;
         for (int i = 0; i < derivativeOrderWithPositionBeingZero; i++)
         {
            columnPower *= col - i;
         }
         constraintMatrix.set(row, col, xPower * columnPower);
         xPower *= x;
      }

      constraintVector.set(row, 0, desiredZDerivative);
   }

   protected void setYoVariables()
   {
      for (int row = 0; row < numberOfCoefficients.getIntegerValue(); row++)
      {
         a[row].set(coefficientVector.get(row, 0));
      }
   }

   @Override
   public void initialize()
   {
      solveForCoefficients();
      setYoVariables();
   }

   @Override
   public boolean isDone()
   {
      return tCurrent >= getTimeInterval().getEndTime();
   }

   public void reshape(int numberOfCoefficientsRequired)
   {
      if (numberOfCoefficientsRequired > maximumNumberOfCoefficients)
         throw new RuntimeException("Maximum number of coefficients is: " + maximumNumberOfCoefficients + ", can't build the polynomial as it requires: "
               + numberOfCoefficientsRequired + " coefficients.");

      coefficientVector.reshape(numberOfCoefficientsRequired, 1);
      constraintMatrix.reshape(numberOfCoefficientsRequired, numberOfCoefficientsRequired);
      constraintVector.reshape(numberOfCoefficientsRequired, 1);
      xPowersDerivativeVector.reshape(numberOfCoefficientsRequired, 1);
      numberOfCoefficients.set(numberOfCoefficientsRequired);

      for (int i = numberOfCoefficientsRequired; i < maximumNumberOfCoefficients; i++)
         a[i].set(Double.NaN);
   }

   @Override
   public String toString()
   {
      String inString = "Polynomial: " + a[0].getDoubleValue();
      for (int i = 1; i < getNumberOfCoefficients(); i++)
      {
         inString += " ";
         if (a[i].getDoubleValue() >= 0)
            inString += "+";
         inString += a[i].getDoubleValue() + " x^" + i;
      }
      return inString;
   }

   @Override
   public TimeIntervalBasics getTimeInterval()
   {
      return timeInterval;
   }
}
