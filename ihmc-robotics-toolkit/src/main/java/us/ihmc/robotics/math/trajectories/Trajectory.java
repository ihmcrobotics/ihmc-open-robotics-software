package us.ihmc.robotics.math.trajectories;

import org.apache.commons.lang3.ArrayUtils;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;

/**
 * Simple trajectory class. Does not use the {@code Polynomial} class since its weird
 * Coefficients are stored with lowest order at the lowest index (what else would you do?) 
 */
public class Trajectory
{
   private final int maximumNumberOfCoefficients;
   private final double[] coefficients;
   private final double[] coefficientsCopy;
   private final DenseMatrix64F constraintMatrix;
   private final DenseMatrix64F constraintVector;
   private final DenseMatrix64F coefficientVector;
   private final LinearSolver<DenseMatrix64F> solver;

   private double tInitial;
   private double tFinal;
   private int numberOfCoefficients;
   private double f, df, ddf;
   private final double[] xPowers;
   private final DenseMatrix64F xPowersDerivativeVector;

   public Trajectory(int maxNumberOfCoefficients)
   {
      this.maximumNumberOfCoefficients = maxNumberOfCoefficients;
      this.coefficients = new double[maxNumberOfCoefficients];
      this.coefficientsCopy = new double[maxNumberOfCoefficients];
      this.constraintMatrix = new DenseMatrix64F(maxNumberOfCoefficients, maxNumberOfCoefficients);
      this.constraintVector = new DenseMatrix64F(maxNumberOfCoefficients, 1);
      this.coefficientVector = new DenseMatrix64F(maxNumberOfCoefficients, 1);
      this.xPowersDerivativeVector = new DenseMatrix64F(maxNumberOfCoefficients, 1);
      this.solver = LinearSolverFactory.general(maxNumberOfCoefficients, maxNumberOfCoefficients);
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
      numberOfCoefficients = 0;
      tInitial = Double.NaN;
      tFinal = Double.NaN;
      for (int i = 0; i < maximumNumberOfCoefficients; i++)
      {
         //xPowers[i] = Double.NaN;
         coefficients[i] = Double.NaN;
      }
      //xPowersDerivativeVector.zero();
   }

   public void compute(double x)
   {
      setXPowers(xPowers, x);
      ddf = df = f = 0.0;
      for (int i = 0; i < numberOfCoefficients; i++)
         f += coefficients[i] * xPowers[i];
      for (int i = 1; i < numberOfCoefficients; i++)
         df += coefficients[i] * (i) * xPowers[i - 1];
      for (int i = 2; i < numberOfCoefficients; i++)
         ddf += coefficients[i] * (i - 1) * (i) * xPowers[i - 2];
   }

   public void setXPowers(double[] xPowers, double x)
   {
      xPowers[0] = 1.0;
      for (int i = 1; i < xPowers.length; i++)
         xPowers[i] = xPowers[i - 1] * x;
   }

   public double getDerivative(int derivativeOrder, double x)
   {
      double val = 0.0;
      setXPowers(xPowers, x);
      for (int i = derivativeOrder; i < numberOfCoefficients; i++)
         val += coefficients[i] * getCoefficientMultiplierForDerivative(derivativeOrder, i) * xPowers[i - derivativeOrder];
      return val;
   }

   /**
    * Returns the order-th derivative of the xPowers vector at value x (Note: does NOT return the Trajectories order-th derivative at x)
    * @param order
    * @param x
    * @return
    */
   public DenseMatrix64F getXPowersDerivativeVector(int order, double x)
   {
      setXPowers(xPowers, x);
      xPowersDerivativeVector.zero();
      for (int i = order; i < numberOfCoefficients; i++)
         xPowersDerivativeVector.set(i, getCoefficientMultiplierForDerivative(order, i) * xPowers[i - order]);
      return xPowersDerivativeVector;
   }

   public void getDerivative(Trajectory trajectory, int order)
   {
      trajectory.reshape(numberOfCoefficients - 1);
      for (int i = 1; i < numberOfCoefficients; i++)
         trajectory.setDirectlyFast(i - 1, getCoefficient(i) * (i));
   }

   public int getCoefficientMultiplierForDerivative(int order, int exponent)
   {
      int coeff = 1;
      for (int i = exponent; i > exponent - order; i--)
         coeff *= i;
      return coeff;
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

   public double getCoefficient(int i)
   {
      setCoefficientsCopy();
      return coefficientsCopy[i];
   }

   public DenseMatrix64F getCoefficientsVector()
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
      this.tInitial = other.tInitial;
      this.tFinal = other.tFinal;
      reshape(other.getNumberOfCoefficients());
      int index = 0;
      for (; index < other.getNumberOfCoefficients(); index++)
         coefficients[index] = other.getCoefficient(index);
      for (; index < maximumNumberOfCoefficients; index++)
         coefficients[index] = Double.NaN;
   }

   public void setZero()
   {
      setConstant(tInitial, tFinal, 0.0);
   }

   public void setConstant(double t0, double tFinal, double z)
   {
      reshape(1);
      setTime(t0, tFinal);
      coefficientVector.set(0, 0, z);
      setCoefficientVariables();
   }

   public void setLinear(double t0, double tFinal, double z0, double zf)
   {
      reshape(2);
      setTime(t0, tFinal);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tFinal, zf);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setQuintic(double t0, double tFinal, double z0, double zd0, double zdd0, double zf, double zdf, double zddf)
   {
      reshape(6);
      setTime(t0, tFinal);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tFinal, zf);
      setVelocityRow(4, tFinal, zdf);
      setAccelerationRow(5, tFinal, zddf);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setQuinticUsingWayPoint(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zdd0, double zIntermediate, double zf,
                                       double zdf)
   {
      reshape(6);
      setTime(t0, tFinal);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tIntermediate, zIntermediate);
      setPositionRow(4, tFinal, zf);
      setVelocityRow(5, tFinal, zdf);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setQuinticUsingWayPoint2(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zdd0, double zIntermediate,
                                        double zdIntermediate, double zf)
   {
      reshape(6);
      setTime(t0, tFinal);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tIntermediate, zIntermediate);
      setVelocityRow(4, tIntermediate, zdIntermediate);
      setPositionRow(5, tFinal, zf);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setQuinticTwoWaypoints(double t0, double tIntermediate0, double tIntermediate1, double tFinal, double z0, double zd0, double zIntermediate0,
                                      double zIntermediate1, double zf, double zdf)
   {
      reshape(6);
      setTime(t0, tFinal);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tIntermediate0, zIntermediate0);
      setPositionRow(3, tIntermediate1, zIntermediate1);
      setPositionRow(4, tFinal, zf);
      setVelocityRow(5, tFinal, zdf);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setQuinticUsingIntermediateVelocityAndAcceleration(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zdIntermediate,
                                                                  double zddIntermediate, double zFinal, double zdFinal)
   {
      reshape(6);
      setTime(t0, tFinal);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setVelocityRow(2, tIntermediate, zdIntermediate);
      setAccelerationRow(3, tIntermediate, zddIntermediate);
      setPositionRow(4, tFinal, zFinal);
      setVelocityRow(5, tFinal, zdFinal);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setQuarticUsingOneIntermediateVelocity(double t0, double tIntermediate0, double tIntermediate1, double tFinal, double z0, double zIntermediate0,
                                                      double zIntermediate1, double zFinal, double zdIntermediate1)
   {
      reshape(5);
      setTime(t0, tFinal);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tIntermediate0, zIntermediate0);
      setPositionRow(2, tIntermediate1, zIntermediate1);
      setVelocityRow(3, tIntermediate1, zdIntermediate1);
      setPositionRow(4, tFinal, zFinal);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setQuinticWithZeroTerminalAcceleration(double t0, double tFinal, double z0, double zd0, double zFinal, double zdFinal)
   {
      setQuintic(t0, tFinal, z0, zd0, 0.0, zFinal, zdFinal, 0.0);
   }

   public void setSexticUsingWaypoint(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zdd0, double zIntermediate, double zf,
                                      double zdf, double zddf)
   {
      reshape(7);
      setTime(t0, tFinal);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tIntermediate, zIntermediate);
      setPositionRow(4, tFinal, zf);
      setVelocityRow(5, tFinal, zdf);
      setAccelerationRow(6, tFinal, zddf);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setSeptic(double t0, double tIntermediate0, double tIntermediate1, double tFinal, double z0, double zd0, double zIntermediate0,
                         double zdIntermediate0, double zIntermediate1, double zdIntermediate1, double zf, double zdf)
   {
      reshape(8);
      setTime(t0, tFinal);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tIntermediate0, zIntermediate0);
      setVelocityRow(3, tIntermediate0, zdIntermediate0);
      setPositionRow(4, tIntermediate1, zIntermediate1);
      setVelocityRow(5, tIntermediate1, zdIntermediate1);
      setPositionRow(6, tFinal, zf);
      setVelocityRow(7, tFinal, zdf);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setSepticInitialAndFinalAcceleration(double t0, double tIntermediate0, double tIntermediate1, double tFinal, double z0, double zd0, double zdd0,
                                                    double zIntermediate0, double zIntermediate1, double zf, double zdf, double zddf)
   {
      reshape(8);
      setTime(t0, tFinal);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tIntermediate0, zIntermediate0);
      setPositionRow(4, tIntermediate1, zIntermediate1);
      setPositionRow(5, tFinal, zf);
      setVelocityRow(6, tFinal, zdf);
      setAccelerationRow(7, tFinal, zddf);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setNonic(double t0, double tIntermediate0, double tIntermediate1, double tFinal, double z0, double zd0, double zIntermediate0,
                        double zdIntermediate0, double zIntermediate1, double zdIntermediate1, double zf, double zdf)
   {
      reshape(10);
      setTime(t0, tFinal);
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
      setCoefficientVariables();
   }

   public void setSexticUsingWaypointVelocityAndAcceleration(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zdd0,
                                                             double zdIntermediate, double zddIntermediate, double zFinal, double zdFinal)
   {
      reshape(7);
      setTime(t0, tFinal);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setVelocityRow(3, tIntermediate, zdIntermediate);
      setAccelerationRow(4, tIntermediate, zddIntermediate);
      setPositionRow(5, tFinal, zFinal);
      setVelocityRow(6, tFinal, zdFinal);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setQuarticUsingIntermediateVelocity(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zdIntermediate, double zFinal,
                                                   double zdFinal)
   {
      reshape(5);
      setTime(t0, tFinal);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setVelocityRow(2, tIntermediate, zdIntermediate);
      setPositionRow(3, tFinal, zFinal);
      setVelocityRow(4, tFinal, zdFinal);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setQuartic(double t0, double tFinal, double z0, double zd0, double zdd0, double zFinal, double zdFinal)
   {
      reshape(5);
      setTime(t0, tFinal);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tFinal, zFinal);
      setVelocityRow(4, tFinal, zdFinal);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setQuarticUsingMidPoint(double t0, double tFinal, double z0, double zd0, double zMid, double zFinal, double zdFinal)
   {
      double tMid = t0 + (tFinal - t0) / 2.0;
      setQuarticUsingWayPoint(t0, tMid, tFinal, z0, zd0, zMid, zFinal, zdFinal);
   }

   public void setQuarticUsingWayPoint(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zIntermediate, double zf, double zdf)
   {
      reshape(5);
      setTime(t0, tFinal);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tIntermediate, zIntermediate);
      setPositionRow(3, tFinal, zf);
      setVelocityRow(4, tFinal, zdf);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setQuarticUsingFinalAcceleration(double t0, double tFinal, double z0, double zd0, double zFinal, double zdFinal, double zddFinal)
   {
      reshape(5);
      setTime(t0, tFinal);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tFinal, zFinal);
      setVelocityRow(3, tFinal, zdFinal);
      setAccelerationRow(4, tFinal, zddFinal);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setCubic(double t0, double tFinal, double z0, double zFinal)
   {
      reshape(4);
      setTime(t0, tFinal);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, 0.0);
      setPositionRow(2, tFinal, zFinal);
      setVelocityRow(3, tFinal, 0.0);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setCubic(double t0, double tFinal, double z0, double zd0, double zFinal, double zdFinal)
   {
      reshape(4);
      setTime(t0, tFinal);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tFinal, zFinal);
      setVelocityRow(3, tFinal, zdFinal);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setCubicWithIntermediatePositionAndInitialVelocityConstraint(double t0, double tIntermediate, double tFinal, double z0, double zd0,
                                                                            double zIntermediate, double zFinal)
   {
      reshape(4);
      setTime(t0, tFinal);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tIntermediate, zIntermediate);
      setPositionRow(3, tFinal, zFinal);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setCubicWithIntermediatePositionAndFinalVelocityConstraint(double t0, double tIntermediate, double tFinal, double z0, double zIntermediate,
                                                                          double zFinal, double zdFinal)
   {
      reshape(4);
      setTime(t0, tFinal);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tIntermediate, zIntermediate);
      setPositionRow(2, tFinal, zFinal);
      setVelocityRow(3, tFinal, zdFinal);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setCubicBezier(double t0, double tFinal, double z0, double zR1, double zR2, double zFinal)
   {
      reshape(4);
      setTime(t0, tFinal);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tFinal, zFinal);
      setVelocityRow(2, t0, 3 * (zR1 - z0) / (tFinal - t0));
      setVelocityRow(3, tFinal, 3 * (zFinal - zR2) / (tFinal - t0));
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setInitialPositionVelocityZeroFinalHighOrderDerivatives(double t0, double tFinal, double z0, double zd0, double zFinal, double zdFinal)
   {
      if (maximumNumberOfCoefficients < 4)
         throw new RuntimeException("Need at least 4 coefficients in order to set initial and final positions and velocities");
      reshape(maximumNumberOfCoefficients);
      setTime(t0, tFinal);
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
      setCoefficientVariables();
   }

   public void setCubicUsingFinalAccelerationButNotFinalPosition(double t0, double tFinal, double z0, double zd0, double zdFinal, double zddFinal)
   {
      reshape(4);
      setTime(t0, tFinal);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setVelocityRow(2, tFinal, zdFinal);
      setAccelerationRow(3, tFinal, zddFinal);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setQuadratic(double t0, double tFinal, double z0, double zd0, double zFinal)
   {
      reshape(3);
      setTime(t0, tFinal);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tFinal, zFinal);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setQuadraticWithFinalVelocityConstraint(double t0, double tFinal, double z0, double zFinal, double zdFinal)
   {
      reshape(3);
      setTime(t0, tFinal);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tFinal, zFinal);
      setVelocityRow(2, tFinal, zdFinal);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setQuadraticUsingInitialAcceleration(double t0, double tFinal, double z0, double zd0, double zdd0)
   {
      reshape(3);
      setTime(t0, tFinal);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setQuadraticUsingIntermediatePoint(double t0, double tIntermediate, double tFinal, double z0, double zIntermediate, double zFinal)
   {
      reshape(3);
      setTime(t0, tFinal);
      MathTools.checkIntervalContains(tIntermediate, t0, tFinal);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tIntermediate, zIntermediate);
      setPositionRow(2, tFinal, zFinal);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setCubicUsingIntermediatePoint(double t0, double tIntermediate1, double tFinal, double z0, double zIntermediate1, double zFinal)
   {
      reshape(4);
      setTime(t0, tFinal);
      MathTools.checkIntervalContains(tIntermediate1, t0, tFinal);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tIntermediate1, zIntermediate1);
      setPositionRow(2, tFinal, zFinal);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setCubicUsingIntermediatePoints(double t0, double tIntermediate1, double tIntermediate2, double tFinal, double z0, double zIntermediate1,
                                               double zIntermediate2, double zFinal)
   {
      reshape(4);
      setTime(t0, tFinal);
      MathTools.checkIntervalContains(tIntermediate1, t0, tIntermediate1);
      MathTools.checkIntervalContains(tIntermediate2, tIntermediate1, tFinal);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tIntermediate1, zIntermediate1);
      setPositionRow(2, tIntermediate2, zIntermediate2);
      setPositionRow(3, tFinal, zFinal);
      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setCubicThreeInitialConditionsFinalPosition(double t0, double tFinal, double z0, double zd0, double zdd0, double zFinal)
   {
      reshape(4);
      setTime(t0, tFinal);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tFinal, zFinal);

      solveForCoefficients();
      setCoefficientVariables();
   }

   public void setCubicInitialPositionThreeFinalConditions(double t0, double tFinal, double z0, double zFinal, double zdFinal, double zddFinal)
   {
      reshape(4);
      setTime(t0, tFinal);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tFinal, zFinal);
      setVelocityRow(2, tFinal, zdFinal);
      setAccelerationRow(3, tFinal, zddFinal);

      solveForCoefficients();
      setCoefficientVariables();
   }

   private void solveForCoefficients()
   {
      solver.setA(constraintMatrix);
      solver.solve(constraintVector, coefficientVector);
   }

   public void setDirectly(DenseMatrix64F coefficients)
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

   public void setTime(double t0, double tFinal)
   {
      setInitialTime(t0);
      setFinalTime(tFinal);
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

   public boolean timeIntervalContains(double timeToCheck, double EPSILON)
   {
      return MathTools.intervalContains(timeToCheck, getInitialTime(), getFinalTime(), EPSILON);
   }

   public boolean timeIntervalContains(double timeToCheck)
   {
      return MathTools.intervalContains(timeToCheck, getInitialTime(), getFinalTime(), Epsilons.ONE_MILLIONTH);
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

   private void setConstraintRow(int row, double x, double desiredZDerivative, int derivativeOrderWithPositionBeingZero)
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

   private void setCoefficientVariables()
   {
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
}