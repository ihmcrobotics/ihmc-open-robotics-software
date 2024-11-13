package us.ihmc.robotics.math.trajectories.yoVariables;

import org.apache.commons.lang3.ArrayUtils;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolynomial3D.PolynomialVariableHolder;
import us.ihmc.robotics.math.trajectories.core.PolynomialTools;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialBasics;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoPolynomial implements PolynomialVariableHolder, PolynomialBasics
{
   private final int maximumNumberOfCoefficients;
   private double pos, vel, acc, jerk, dPos;
   private final YoDouble[] a;
   private final double[] coefficientsCopy;
   private final YoDouble currentTime;
   private final YoInteger numberOfCoefficients;
   private final DMatrixRMaj constraintMatrix;
   private final DMatrixRMaj constraintVector;
   private final DMatrixRMaj coefficientVector;
   private final double[] xPowers;

   private final TimeIntervalBasics timeInterval = new TimeInterval();
   private boolean isConstraintMatrixUpToDate = false;

   // Stores the (n-th order) derivative of the xPowers vector
   private final DMatrixRMaj xPowersDerivativeVector;

   private final LinearSolverDense<DMatrixRMaj> solver;

   public YoPolynomial(String name, int maximumNumberOfCoefficients, YoRegistry registry)
   {

      this(createCoefficientsArray(name, maximumNumberOfCoefficients, registry),
           new YoInteger(name + "_nCoeffs", registry),
           new YoDouble(name + "_tCurrent", registry));
   }

   public YoPolynomial(YoDouble[] coefficients, YoInteger numberOfCoefficients,
                       YoDouble currentTime)
   {
      this.a = coefficients;
      this.numberOfCoefficients = numberOfCoefficients;
      this.currentTime = currentTime;

      maximumNumberOfCoefficients = coefficients.length;
      this.coefficientsCopy = new double[maximumNumberOfCoefficients];

      solver = LinearSolverFactory_DDRM.general(maximumNumberOfCoefficients, maximumNumberOfCoefficients);

      constraintMatrix = new DMatrixRMaj(maximumNumberOfCoefficients, maximumNumberOfCoefficients);
      constraintVector = new DMatrixRMaj(maximumNumberOfCoefficients, 1);
      coefficientVector = new DMatrixRMaj(maximumNumberOfCoefficients, 1);
      xPowers = new double[maximumNumberOfCoefficients];

      xPowersDerivativeVector = new DMatrixRMaj(maximumNumberOfCoefficients, 1);
   }

   private static YoDouble[] createCoefficientsArray(String name, int maximumNumberOfCoefficients, YoRegistry registry)
   {
      if (maximumNumberOfCoefficients < 1)
         throw new IllegalArgumentException("You have to make the polynomial to have at least 1 coefficient!");

      YoDouble[] a = new YoDouble[maximumNumberOfCoefficients];
      for (int i = 0; i < maximumNumberOfCoefficients; i++)
      {
         a[i] = new YoDouble(name + "_a" + i, registry);
      }
      return a;
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

   @Override
   public double getCoefficient(int i)
   {
      return a[i].getDoubleValue();
   }

   @Override
   public void setNumberOfCoefficients(int numberOfCoefficients)
   {
      this.numberOfCoefficients.set(numberOfCoefficients);
   }

   public double getJerk()
   {
      return jerk;
   }

   @Override
   public double[] getCoefficients()
   {
      setCoefficientsCopy();
      return coefficientsCopy;
   }

   @Override
   public double getCurrentTime()
   {
      return currentTime.getDoubleValue();
   }

   @Override
   public void setCurrentTime(double currentTime)
   {
      this.currentTime.set(currentTime);
   }

   private void setCoefficientsCopy()
   {
      int row = 0;
      for (; row < getNumberOfCoefficients(); row++)
         coefficientsCopy[row] = a[row].getDoubleValue();
      for (; row < maximumNumberOfCoefficients; row++)
         coefficientsCopy[row] = Double.NaN;
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

   @Override
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
      setIsConstraintMatrixUpToDate(false);
      if (power >= maximumNumberOfCoefficients)
         throw new RuntimeException(
               "Maximum number of coefficients is: " + maximumNumberOfCoefficients + ", can't set coefficient as it requires: " + power + 1 + " coefficients");

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

   @Override
   public void compute(double x)
   {
      setCurrentTime(x);
      PolynomialTools.setXPowers(xPowers, x);

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

   @Override
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

      for (int col = derivativeOrderWithPositionBeingZero; col < getNumberOfCoefficients(); col++)
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

      constraintMatrix.zero();
      constraintVector.zero();
      coefficientVector.zero();

      for (int i = numberOfCoefficientsRequired; i < maximumNumberOfCoefficients; i++)
         a[i].set(Double.NaN);
   }

   @Override
   public void setCoefficientUnsafe(int idx, double value)
   {
      a[idx].set(value);
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
