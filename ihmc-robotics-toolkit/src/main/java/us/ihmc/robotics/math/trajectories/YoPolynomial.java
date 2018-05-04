package us.ihmc.robotics.math.trajectories;

import org.apache.commons.lang3.ArrayUtils;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;

import us.ihmc.commons.MathTools;
import us.ihmc.robotics.dataStructures.PolynomialReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoPolynomial implements PolynomialReadOnly
{
   private final int maximumNumberOfCoefficients;
   private double pos, vel, acc, dPos;
   private final YoDouble[] a;
   private final YoInteger numberOfCoefficients;
   private final DenseMatrix64F constraintMatrix;
   private final DenseMatrix64F constraintVector;
   private final DenseMatrix64F coefficientVector;
   private final double[] xPowers;

   // Stores the (n-th order) derivative of the xPowers vector
   private final DenseMatrix64F xPowersDerivativeVector;

   private final LinearSolver<DenseMatrix64F> solver;

   public YoPolynomial(String name, int maximumNumberOfCoefficients, YoVariableRegistry registry)
   {
      this.maximumNumberOfCoefficients = maximumNumberOfCoefficients;

      solver = LinearSolverFactory.general(maximumNumberOfCoefficients, maximumNumberOfCoefficients);

      a = new YoDouble[maximumNumberOfCoefficients];
      constraintMatrix = new DenseMatrix64F(maximumNumberOfCoefficients, maximumNumberOfCoefficients);
      constraintVector = new DenseMatrix64F(maximumNumberOfCoefficients, 1);
      coefficientVector = new DenseMatrix64F(maximumNumberOfCoefficients, 1);
      xPowers = new double[maximumNumberOfCoefficients];

      xPowersDerivativeVector = new DenseMatrix64F(maximumNumberOfCoefficients, 1);

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

      this.maximumNumberOfCoefficients = coefficients.length;

      solver = LinearSolverFactory.general(maximumNumberOfCoefficients, maximumNumberOfCoefficients);

      constraintMatrix = new DenseMatrix64F(maximumNumberOfCoefficients, maximumNumberOfCoefficients);
      constraintVector = new DenseMatrix64F(maximumNumberOfCoefficients, 1);
      coefficientVector = new DenseMatrix64F(maximumNumberOfCoefficients, 1);
      xPowers = new double[maximumNumberOfCoefficients];

      xPowersDerivativeVector = new DenseMatrix64F(maximumNumberOfCoefficients, 1);
   }

   @Override
   public double getPosition()
   {
      return pos;
   }

   public double getVelocity()
   {
      return vel;
   }

   public double getAcceleration()
   {
      return acc;
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

   public DenseMatrix64F getCoefficientsVector()
   {
      return coefficientVector;
   }

   public YoDouble[] getYoCoefficients()
   {
      return a;
   }

   public void reset()
   {
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

   public void setConstant(double z)
   {
      reshape(1);
      coefficientVector.set(0, 0, z);
      setYoVariables();
   }

   public void setLinear(double t0, double tFinal, double z0, double zf)
   {
      reshape(2);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tFinal, zf);
      solveForCoefficients();
      setYoVariables();
   }

   public void setLinear(double t, double z, double zd)
   {
      reshape(2);
      setPositionRow(0, t, z);
      setVelocityRow(1, t, zd);
      solveForCoefficients();
      setYoVariables();
   }

   public void setQuintic(double t0, double tFinal, double z0, double zd0, double zdd0, double zf, double zdf, double zddf)
   {
      reshape(6);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tFinal, zf);
      setVelocityRow(4, tFinal, zdf);
      setAccelerationRow(5, tFinal, zddf);
      solveForCoefficients();
      setYoVariables();
   }
   
   public void setQuinticUsingWayPoint(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zdd0, double zIntermediate, double zf,
                                       double zdf)
   {
      reshape(6);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tIntermediate, zIntermediate);
      setPositionRow(4, tFinal, zf);
      setVelocityRow(5, tFinal, zdf);
      solveForCoefficients();
      setYoVariables();
   }

   public void setQuinticUsingWayPoint2(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zdd0, double zIntermediate,
                                        double zdIntermediate, double zf)
   {
      reshape(6);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tIntermediate, zIntermediate);
      setVelocityRow(4, tIntermediate, zdIntermediate);
      setPositionRow(5, tFinal, zf);
      solveForCoefficients();
      setYoVariables();
   }

   public void setQuinticTwoWaypoints(double t0, double tIntermediate0, double tIntermediate1, double tFinal, double z0, double zd0, double zIntermediate0,
                                      double zIntermediate1, double zf, double zdf)
   {
      reshape(6);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tIntermediate0, zIntermediate0);
      setPositionRow(3, tIntermediate1, zIntermediate1);
      setPositionRow(4, tFinal, zf);
      setVelocityRow(5, tFinal, zdf);
      solveForCoefficients();
      setYoVariables();
   }

   public void setQuinticUsingIntermediateVelocityAndAcceleration(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zdIntermediate,
                                                                  double zddIntermediate, double zFinal, double zdFinal)
   {
      reshape(6);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setVelocityRow(2, tIntermediate, zdIntermediate);
      setAccelerationRow(3, tIntermediate, zddIntermediate);
      setPositionRow(4, tFinal, zFinal);
      setVelocityRow(5, tFinal, zdFinal);
      solveForCoefficients();
      setYoVariables();
   }

   public void setQuarticUsingOneIntermediateVelocity(double t0, double tIntermediate0, double tIntermediate1, double tFinal, double z0, double zIntermediate0,
                                                      double zIntermediate1, double zFinal, double zdIntermediate1)
   {
      reshape(5);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tIntermediate0, zIntermediate0);
      setPositionRow(2, tIntermediate1, zIntermediate1);
      setVelocityRow(3, tIntermediate1, zdIntermediate1);
      setPositionRow(4, tFinal, zFinal);
      solveForCoefficients();
      setYoVariables();
   }

   public void setQuinticWithZeroTerminalAcceleration(double t0, double tFinal, double z0, double zd0, double zFinal, double zdFinal)
   {
      setQuintic(t0, tFinal, z0, zd0, 0.0, zFinal, zdFinal, 0.0);
   }
   
   public void setSexticUsingWaypoint(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zdd0, double zIntermediate, double zf,
                                      double zdf, double zddf)
   {
      reshape(7);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tIntermediate, zIntermediate);
      setPositionRow(4, tFinal, zf);
      setVelocityRow(5, tFinal, zdf);
      setAccelerationRow(6, tFinal, zddf);
      solveForCoefficients();
      setYoVariables();
   }

   public void setSeptic(double t0, double tIntermediate0, double tIntermediate1, double tFinal, double z0, double zd0, double zIntermediate0,
                         double zdIntermediate0, double zIntermediate1, double zdIntermediate1, double zf, double zdf)
   {
      reshape(8);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tIntermediate0, zIntermediate0);
      setVelocityRow(3, tIntermediate0, zdIntermediate0);
      setPositionRow(4, tIntermediate1, zIntermediate1);
      setVelocityRow(5, tIntermediate1, zdIntermediate1);
      setPositionRow(6, tFinal, zf);
      setVelocityRow(7, tFinal, zdf);
      solveForCoefficients();
      setYoVariables();
   }

   public void setSepticInitialAndFinalAcceleration(double t0, double tIntermediate0, double tIntermediate1, double tFinal, double z0, double zd0, double zdd0,
                                                    double zIntermediate0, double zIntermediate1, double zf, double zdf, double zddf)
   {
      reshape(8);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tIntermediate0, zIntermediate0);
      setPositionRow(4, tIntermediate1, zIntermediate1);
      setPositionRow(5, tFinal, zf);
      setVelocityRow(6, tFinal, zdf);
      setAccelerationRow(7, tFinal, zddf);
      solveForCoefficients();
      setYoVariables();
   }

   public void setNonic(double t0, double tIntermediate0, double tIntermediate1, double tFinal, double z0, double zd0, double zIntermediate0,
                        double zdIntermediate0, double zIntermediate1, double zdIntermediate1, double zf, double zdf)
   {
      reshape(10);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tIntermediate0, zIntermediate0);
      setVelocityRow(3, tIntermediate0, zdIntermediate0);
      setPositionRow(4, tIntermediate1, zIntermediate1);
      setVelocityRow(5, tIntermediate1, zdIntermediate1);
      setPositionRow(6, tFinal, zf);
      setVelocityRow(7, tFinal, zdf);
      setAccelerationRow(8, t0, 0.0);
      setAccelerationRow(9, tFinal, 0.0);
      solveForCoefficients();
      setYoVariables();
   }

   public void setSexticUsingWaypointVelocityAndAcceleration(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zdd0,
                                                             double zdIntermediate, double zddIntermediate, double zFinal, double zdFinal)
   {
      reshape(7);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setVelocityRow(3, tIntermediate, zdIntermediate);
      setAccelerationRow(4, tIntermediate, zddIntermediate);
      setPositionRow(5, tFinal, zFinal);
      setVelocityRow(6, tFinal, zdFinal);
      solveForCoefficients();
      setYoVariables();
   }

   public void setQuarticUsingIntermediateVelocity(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zdIntermediate, double zFinal,
                                                   double zdFinal)
   {
      reshape(5);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setVelocityRow(2, tIntermediate, zdIntermediate);
      setPositionRow(3, tFinal, zFinal);
      setVelocityRow(4, tFinal, zdFinal);
      solveForCoefficients();
      setYoVariables();
   }

   public void setQuartic(double t0, double tFinal, double z0, double zd0, double zdd0, double zFinal, double zdFinal)
   {
      reshape(5);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tFinal, zFinal);
      setVelocityRow(4, tFinal, zdFinal);
      solveForCoefficients();
      setYoVariables();
   }

   public void setQuarticUsingMidPoint(double t0, double tFinal, double z0, double zd0, double zMid, double zFinal, double zdFinal)
   {
      double tMid = t0 + (tFinal - t0) / 2.0;
      setQuarticUsingWayPoint(t0, tMid, tFinal, z0, zd0, zMid, zFinal, zdFinal);
   }

   public void setQuarticUsingWayPoint(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zIntermediate, double zf, double zdf)
   {
      reshape(5);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tIntermediate, zIntermediate);
      setPositionRow(3, tFinal, zf);
      setVelocityRow(4, tFinal, zdf);
      solveForCoefficients();
      setYoVariables();
   }

   public void setQuarticUsingFinalAcceleration(double t0, double tFinal, double z0, double zd0, double zFinal, double zdFinal, double zddFinal)
   {
      reshape(5);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tFinal, zFinal);
      setVelocityRow(3, tFinal, zdFinal);
      setAccelerationRow(4, tFinal, zddFinal);
      solveForCoefficients();
      setYoVariables();
   }

   public void setCubic(double t0, double tFinal, double z0, double zFinal)
   {
      reshape(4);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, 0.0);
      setPositionRow(2, tFinal, zFinal);
      setVelocityRow(3, tFinal, 0.0);
      solveForCoefficients();
      setYoVariables();
   }

   public void setCubic(double t0, double tFinal, double z0, double zd0, double zFinal, double zdFinal)
   {
      reshape(4);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tFinal, zFinal);
      setVelocityRow(3, tFinal, zdFinal);
      solveForCoefficients();
      setYoVariables();
   }

   public void setCubicWithIntermediatePositionAndInitialVelocityConstraint(double t0, double tIntermediate, double tFinal, double z0, double zd0,
                                                                            double zIntermediate, double zFinal)
   {
      reshape(4);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tIntermediate, zIntermediate);
      setPositionRow(3, tFinal, zFinal);
      solveForCoefficients();
      setYoVariables();
   }

   public void setCubicWithIntermediatePositionAndFinalVelocityConstraint(double t0, double tIntermediate, double tFinal, double z0, double zIntermediate,
                                                                          double zFinal, double zdFinal)
   {
      reshape(4);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tIntermediate, zIntermediate);
      setPositionRow(2, tFinal, zFinal);
      setVelocityRow(3, tFinal, zdFinal);
      solveForCoefficients();
      setYoVariables();
   }

   public void setCubicBezier(double t0, double tFinal, double z0, double zR1, double zR2, double zFinal)
   {
      reshape(4);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tFinal, zFinal);
      setVelocityRow(2, t0, 3 * (zR1 - z0) / (tFinal - t0));
      setVelocityRow(3, tFinal, 3 * (zFinal - zR2) / (tFinal - t0));
      solveForCoefficients();
      setYoVariables();
   }

   public void setInitialPositionVelocityZeroFinalHighOrderDerivatives(double t0, double tFinal, double z0, double zd0, double zFinal, double zdFinal)
   {
      if (maximumNumberOfCoefficients < 4)
         throw new RuntimeException("Need at least 4 coefficients in order to set initial and final positions and velocities");
      reshape(maximumNumberOfCoefficients);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tFinal, zFinal);
      setVelocityRow(3, tFinal, zdFinal);

      int order = 2;

      for (int row = 4; row < maximumNumberOfCoefficients; row++)
      {
         setConstraintRow(row, tFinal, 0.0, order++);
      }

      solveForCoefficients();
      setYoVariables();
   }

   public void setCubicUsingFinalAccelerationButNotFinalPosition(double t0, double tFinal, double z0, double zd0, double zdFinal, double zddFinal)
   {
      reshape(4);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setVelocityRow(2, tFinal, zdFinal);
      setAccelerationRow(3, tFinal, zddFinal);
      solveForCoefficients();
      setYoVariables();
   }

   public void setQuadratic(double t0, double tFinal, double z0, double zd0, double zFinal)
   {
      reshape(3);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tFinal, zFinal);
      solveForCoefficients();
      setYoVariables();
   }

   public void setQuadraticWithFinalVelocityConstraint(double t0, double tFinal, double z0, double zFinal, double zdFinal)
   {
      reshape(3);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tFinal, zFinal);
      setVelocityRow(2, tFinal, zdFinal);
      solveForCoefficients();
      setYoVariables();
   }

   public void setQuadraticUsingInitialAcceleration(double t0, double tFinal, double z0, double zd0, double zdd0)
   {
      reshape(3);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      solveForCoefficients();
      setYoVariables();
   }

   public void setQuadraticUsingIntermediatePoint(double t0, double tIntermediate, double tFinal, double z0, double zIntermediate, double zFinal)
   {
      reshape(3);
      MathTools.checkIntervalContains(tIntermediate, t0, tFinal);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tIntermediate, zIntermediate);
      setPositionRow(2, tFinal, zFinal);
      solveForCoefficients();
      setYoVariables();
   }

   public void setCubicUsingIntermediatePoint(double t0, double tIntermediate1, double tFinal, double z0, double zIntermediate1, double zFinal)
   {
      reshape(4);
      MathTools.checkIntervalContains(tIntermediate1, t0, tFinal);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tIntermediate1, zIntermediate1);
      setPositionRow(2, tFinal, zFinal);
      solveForCoefficients();
      setYoVariables();
   }

   public void setCubicUsingIntermediatePoints(double t0, double tIntermediate1, double tIntermediate2, double tFinal, double z0, double zIntermediate1,
                                               double zIntermediate2, double zFinal)
   {
      reshape(4);
      MathTools.checkIntervalContains(tIntermediate1, t0, tIntermediate1);
      MathTools.checkIntervalContains(tIntermediate2, tIntermediate1, tFinal);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tIntermediate1, zIntermediate1);
      setPositionRow(2, tIntermediate2, zIntermediate2);
      setPositionRow(3, tFinal, zFinal);
      solveForCoefficients();
      setYoVariables();
   }

   public void setCubicThreeInitialConditionsFinalPosition(double t0, double tFinal, double z0, double zd0, double zdd0, double zFinal)
   {
      reshape(4);

      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tFinal, zFinal);

      solveForCoefficients();
      setYoVariables();
   }

   public void setCubicInitialPositionThreeFinalConditions(double t0, double tFinal, double z0, double zFinal, double zdFinal, double zddFinal)
   {
      reshape(4);

      setPositionRow(0, t0, z0);
      setPositionRow(1, tFinal, zFinal);
      setVelocityRow(2, tFinal, zdFinal);
      setAccelerationRow(3, tFinal, zddFinal);

      solveForCoefficients();
      setYoVariables();
   }

   private void solveForCoefficients()
   {
      solver.setA(constraintMatrix);
      solver.solve(constraintVector, coefficientVector);
   }

   public void setDirectly(DenseMatrix64F coefficients)
   {
      reshape(coefficients.getNumRows());
      for (int i = 0; i < numberOfCoefficients.getIntegerValue(); i++)
      {
         this.a[i].set(coefficients.get(i, 0));
      }
   }
   
   public void setDirectly(double[] coefficients)
   {
      reshape(coefficients.length);
      for (int i = 0; i < numberOfCoefficients.getIntegerValue(); i++)
      {
         this.a[i].set(coefficients[i]);
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
         this.coefficientVector.reshape(power + 1, 1);
         this.constraintMatrix.reshape(power + 1, power + 1);
         this.constraintVector.reshape(power + 1, 1);
         this.xPowersDerivativeVector.reshape(power + 1, 1);
         numberOfCoefficients.set(power + 1);
      }
      a[power].set(coefficient);
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
      setXPowers(xPowers, x);

      pos = vel = acc = 0.0;
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
         integral += (1.0 / (i + 1.0)) * a[i].getDoubleValue() * (toPowers[i + 1] - fromPowers[i + 1]);
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
    *  Returns the order-th derivative of the xPowers vector at value x (Note: does NOT return the YoPolynomials order-th derivative at x)
    * @param order
    * @param x
    * @return
    */
   public DenseMatrix64F getXPowersDerivativeVector(int order, double x)
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

   public YoInteger getYoNumberOfCoefficients()
   {
      return numberOfCoefficients;
   }

   public int getMaximumNumberOfCoefficients()
   {
      return maximumNumberOfCoefficients;
   }

   private void setConstraintRow(int row, double x, double desiredZDerivative, int derivativeOrderWithPositionBeingZero)
   {
      double xPower = 1.0;

      for (int col = derivativeOrderWithPositionBeingZero; col < numberOfCoefficients.getIntegerValue(); col++)
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

   private void setPositionRow(int row, double x, double z)
   {
      setConstraintRow(row, x, z, 0);
   }

   private void setVelocityRow(int row, double x, double zVelocity)
   {
      setConstraintRow(row, x, zVelocity, 1);
   }

   private void setAccelerationRow(int row, double x, double zAcceleration)
   {
      setConstraintRow(row, x, zAcceleration, 2);
   }

   private void setYoVariables()
   {
      for (int row = 0; row < numberOfCoefficients.getIntegerValue(); row++)
      {
         a[row].set(coefficientVector.get(row, 0));
      }
   }

   public void reshape(int numberOfCoefficientsRequired)
   {
      if (numberOfCoefficientsRequired > maximumNumberOfCoefficients)
         throw new RuntimeException("Maximum number of coefficients is: " + maximumNumberOfCoefficients + ", can't build the polynomial as it requires: "
               + numberOfCoefficientsRequired + " coefficients.");

      this.coefficientVector.reshape(numberOfCoefficientsRequired, 1);
      this.constraintMatrix.reshape(numberOfCoefficientsRequired, numberOfCoefficientsRequired);
      this.constraintVector.reshape(numberOfCoefficientsRequired, 1);
      this.xPowersDerivativeVector.reshape(numberOfCoefficientsRequired, 1);
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
}
