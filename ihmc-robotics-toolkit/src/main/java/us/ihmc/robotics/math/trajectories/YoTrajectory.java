package us.ihmc.robotics.math.trajectories;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoTrajectory
{
   private String name;
   private YoPolynomial polynomial;
   private YoDouble tInitial;
   private YoDouble tFinal;
   private double[] coefficients;

   public YoTrajectory(String name, int maximumNumberOfCoefficients, YoVariableRegistry registry)
   {
      this.name = name;
      tInitial = new YoDouble(name + "t0", registry);
      tFinal = new YoDouble(name + "tF", registry);
      polynomial = new YoPolynomial(name + "Poly", maximumNumberOfCoefficients, registry);
      coefficients = new double[maximumNumberOfCoefficients];
   }

   public YoTrajectory(YoDouble[] coefficients, YoInteger numberOfCoefficients, YoDouble tInitial, YoDouble tFinal)
   {
      polynomial = new YoPolynomial(coefficients, numberOfCoefficients);
      this.tInitial = tInitial;
      this.tFinal = tFinal;
      this.coefficients = new double[numberOfCoefficients.getIntegerValue()];
   }

   public void setTime(double tInital, double tFinal)
   {
      setInitialTime(tInital);
      setFinalTime(tFinal);
   }

   public void setInitialTime(double tInitial)
   {
      this.tInitial.set(tInitial);
   }

   public void setFinalTime(double tFinal)
   {
      this.tFinal.set(tFinal);
   }

   public double getInitialTime()
   {
      return tInitial.getDoubleValue();
   }

   public double getFinalTime()
   {
      return tFinal.getDoubleValue();
   }

   public boolean timeIntervalContains(double timeToCheck, double EPSILON)
   {
      return MathTools.intervalContains(timeToCheck, getInitialTime(), getFinalTime(), EPSILON);
   }
   
   public boolean timeIntervalContains(double timeToCheck)
   {
      return MathTools.intervalContains(timeToCheck, getInitialTime(), getFinalTime(), Epsilons.ONE_MILLIONTH);
   }
   
   public double getPosition()
   {
      return polynomial.getPosition();
   }

   public double getVelocity()
   {
      return polynomial.getVelocity();
   }

   public double getAcceleration()
   {
      return polynomial.getAcceleration();
   }

   public DenseMatrix64F getCoefficientsVector()
   {
      return polynomial.getCoefficientsVector();
   }

   public void reset()
   {
      polynomial.reset();
      tInitial.set(0.0);
      tFinal.set(0.0);
   }

   public double getCoefficient(int i)
   {
      return polynomial.getCoefficient(i);
   }

   public double[] getCoefficients()
   {
      int index = 0;
      for(; index < polynomial.getNumberOfCoefficients(); index++)
         coefficients[index] = polynomial.getCoefficient(index);
      for(; index < coefficients.length; index++)
         coefficients[index] = 0;
      return coefficients;
   }

   public int getNumberOfCoefficients()
   {
      return polynomial.getNumberOfCoefficients();
   }

   public YoInteger getYoNumberOfCoefficients()
   {

      return polynomial.getYoNumberOfCoefficients();
   }

   public int getMaximumNumberOfCoefficients()
   {
      return polynomial.getMaximumNumberOfCoefficients();
   }

   public void set(YoTrajectory other)
   {
      polynomial.set(other.polynomial);
      tInitial.set(other.getInitialTime());
      tFinal.set(other.getFinalTime());
   }

   public void reshape(int numberOfCoefficientsRequired)
   {
      polynomial.reshape(numberOfCoefficientsRequired);
      tInitial.setToNaN();
      tFinal.setToNaN();
   }

   public void setDirectlyFast(int power, double coefficient)
   {
      this.polynomial.setDirectlyFast(power, coefficient);
   }
   
   /**
    * Sets the coefficients to zero. Initial and final time are retained.
    */
   public void setZero()
   {
      polynomial.setConstant(0.0);
   }

   public void setConstant(double t0, double tFinal, double z)
   {
      setTime(t0, tFinal);
      polynomial.setConstant(z);
   }

   public void setLinear(double t0, double tFinal, double z0, double zf)
   {
      setTime(t0, tFinal);
      polynomial.setLinear(t0, tFinal, z0, zf);
   }

   public void setQuintic(double t0, double tFinal, double z0, double zd0, double zdd0, double zf, double zdf, double zddf)
   {
      setTime(t0, tFinal);
      polynomial.setQuintic(t0, tFinal, z0, zd0, zdd0, zf, zdf, zddf);
   }

   public void setQuinticUsingWayPoint(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zdd0, double zIntermediate, double zf,
                                       double zdf)
   {
      setTime(t0, tFinal);
      polynomial.setQuinticUsingWayPoint(t0, tIntermediate, tFinal, z0, zd0, zdd0, zIntermediate, zf, zdf);
   }

   public void setQuinticUsingWayPoint2(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zdd0, double zIntermediate,
                                        double zdIntermediate, double zf)
   {
      setTime(t0, tFinal);
      polynomial.setQuinticUsingWayPoint2(t0, tIntermediate, tFinal, z0, zd0, zdd0, zIntermediate, zdIntermediate, zf);
   }

   public void setQuinticTwoWaypoints(double t0, double tIntermediate0, double tIntermediate1, double tFinal, double z0, double zd0, double zIntermediate0,
                                      double zIntermediate1, double zf, double zdf)
   {
      setTime(t0, tFinal);
      polynomial.setQuinticTwoWaypoints(t0, tIntermediate0, tIntermediate1, tFinal, z0, zd0, zIntermediate0, zIntermediate1, zf, zdf);
   }

   public void setQuinticUsingIntermediateVelocityAndAcceleration(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zdIntermediate,
                                                                  double zddIntermediate, double zFinal, double zdFinal)
   {
      setTime(t0, tFinal);
      polynomial.setQuinticUsingIntermediateVelocityAndAcceleration(t0, tIntermediate, tFinal, z0, zd0, zdIntermediate, zddIntermediate, zFinal, zdFinal);
   }

   public void setQuarticUsingOneIntermediateVelocity(double t0, double tIntermediate0, double tIntermediate1, double tFinal, double z0, double zIntermediate0,
                                                      double zIntermediate1, double zFinal, double zdIntermediate1)
   {
      setTime(t0, tFinal);
      polynomial.setQuarticUsingOneIntermediateVelocity(t0, tIntermediate0, tIntermediate1, tFinal, z0, zIntermediate0, zIntermediate1, zFinal,
                                                        zdIntermediate1);
   }
   
   public void setQuinticWithZeroTerminalAcceleration(double t0, double tFinal, double z0, double zd0, double zFinal, double zdFinal)
   {
      setTime(t0, tFinal);
      polynomial.setQuinticWithZeroTerminalAcceleration(t0, tFinal, z0, zd0, zFinal, zdFinal);
   }
   
   public void setSexticUsingWaypoint(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zdd0, double zIntermediate, double zf,
                                      double zdf, double zddf)
   {
      setTime(t0, tFinal);
      polynomial.setSexticUsingWaypoint(t0, tIntermediate, tFinal, z0, zd0, zdd0, zIntermediate, zf, zdf, zddf);
   }

   public void setSeptic(double t0, double tIntermediate0, double tIntermediate1, double tFinal, double z0, double zd0, double zIntermediate0,
                         double zdIntermediate0, double zIntermediate1, double zdIntermediate1, double zf, double zdf)
   {
      setTime(t0, tFinal);
      polynomial.setSeptic(t0, tIntermediate0, tIntermediate1, tFinal, z0, zd0, zIntermediate0, zdIntermediate0, zIntermediate1, zdIntermediate1, zf, zdf);
   }

   public void setSepticInitialAndFinalAcceleration(double t0, double tIntermediate0, double tIntermediate1, double tFinal, double z0, double zd0, double zdd0,
                                                    double zIntermediate0, double zIntermediate1, double zf, double zdf, double zddf)
   {
      setTime(t0, tFinal);
      polynomial.setSepticInitialAndFinalAcceleration(t0, tIntermediate0, tIntermediate1, tFinal, z0, zd0, zdd0, zIntermediate0, zIntermediate1, zf, zdf, zddf);
   }

   public void setNonic(double t0, double tIntermediate0, double tIntermediate1, double tFinal, double z0, double zd0, double zIntermediate0,
                        double zdIntermediate0, double zIntermediate1, double zdIntermediate1, double zf, double zdf)
   {
      setTime(t0, tFinal);
      polynomial.setNonic(t0, tIntermediate0, tIntermediate1, tFinal, z0, zd0, zIntermediate0, zdIntermediate0, zIntermediate1, zdIntermediate1, zf, zdf);
   }

   public void setSexticUsingWaypointVelocityAndAcceleration(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zdd0,
                                                             double zdIntermediate, double zddIntermediate, double zFinal, double zdFinal)
   {
      setTime(t0, tFinal);
      polynomial.setSexticUsingWaypointVelocityAndAcceleration(t0, tIntermediate, tFinal, z0, zd0, zdd0, zdIntermediate, zddIntermediate, zFinal, zdFinal);
   }

   public void setQuarticUsingIntermediateVelocity(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zdIntermediate, double zFinal,
                                                   double zdFinal)
   {
      setTime(t0, tFinal);
      polynomial.setQuarticUsingIntermediateVelocity(t0, tIntermediate, tFinal, z0, zd0, zdIntermediate, zFinal, zdFinal);
   }

   public void setQuartic(double t0, double tFinal, double z0, double zd0, double zdd0, double zFinal, double zdFinal)
   {
      setTime(t0, tFinal);
      polynomial.setQuartic(t0, tFinal, z0, zd0, zdd0, zFinal, zdFinal);
   }

   public void setQuarticUsingMidPoint(double t0, double tFinal, double z0, double zd0, double zMid, double zFinal, double zdFinal)
   {
      setTime(t0, tFinal);
      polynomial.setQuarticUsingMidPoint(t0, tFinal, z0, zd0, zMid, zFinal, zdFinal);
   }

   public void setQuarticUsingWayPoint(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zIntermediate, double zf, double zdf)
   {
      setTime(t0, tFinal);
      polynomial.setQuarticUsingWayPoint(t0, tIntermediate, tFinal, z0, zd0, zIntermediate, zf, zdf);
   }

   public void setQuarticUsingFinalAcceleration(double t0, double tFinal, double z0, double zd0, double zFinal, double zdFinal, double zddFinal)
   {
      setTime(t0, tFinal);
      polynomial.setQuarticUsingFinalAcceleration(t0, tFinal, z0, zd0, zFinal, zdFinal, zddFinal);
   }

   public void setCubic(double t0, double tFinal, double z0, double zFinal)
   {
      setTime(t0, tFinal);
      polynomial.setCubic(t0, tFinal, z0, zFinal);
   }

   public void setCubic(double t0, double tFinal, double z0, double zd0, double zFinal, double zdFinal)
   {
      setTime(t0, tFinal);
      polynomial.setCubic(t0, tFinal, z0, zd0, zFinal, zdFinal);
   }

   public void setCubicWithIntermediatePositionAndInitialVelocityConstraint(double t0, double tIntermediate, double tFinal, double z0, double zd0,
                                                                            double zIntermediate, double zFinal)
   {
      setTime(t0, tFinal);
      polynomial.setCubicWithIntermediatePositionAndInitialVelocityConstraint(t0, tIntermediate, tFinal, z0, zd0, zIntermediate, zFinal);
   }

   public void setCubicWithIntermediatePositionAndFinalVelocityConstraint(double t0, double tIntermediate, double tFinal, double z0, double zIntermediate,
                                                                          double zFinal, double zdFinal)
   {
      setTime(t0, tFinal);
      polynomial.setCubicWithIntermediatePositionAndFinalVelocityConstraint(t0, tIntermediate, tFinal, z0, zIntermediate, zFinal, zdFinal);
   }

   public void setCubicBezier(double t0, double tFinal, double z0, double zR1, double zR2, double zFinal)
   {
      setTime(t0, tFinal);
      polynomial.setCubicBezier(t0, tFinal, z0, zR1, zR2, zFinal);
   }

   public void setInitialPositionVelocityZeroFinalHighOrderDerivatives(double t0, double tFinal, double z0, double zd0, double zFinal, double zdFinal)
   {
      setTime(t0, tFinal);
      polynomial.setInitialPositionVelocityZeroFinalHighOrderDerivatives(t0, tFinal, z0, zd0, zFinal, zdFinal);
   }

   public void setCubicUsingFinalAccelerationButNotFinalPosition(double t0, double tFinal, double z0, double zd0, double zdFinal, double zddFinal)
   {
      setTime(t0, tFinal);
      polynomial.setCubicUsingFinalAccelerationButNotFinalPosition(t0, tFinal, z0, zd0, zdFinal, zddFinal);
   }

   public void setQuadratic(double t0, double tFinal, double z0, double zd0, double zFinal)
   {
      setTime(t0, tFinal);
      polynomial.setQuadratic(t0, tFinal, z0, zd0, zFinal);
   }

   public void setQuadraticWithFinalVelocityConstraint(double t0, double tFinal, double z0, double zFinal, double zdFinal)
   {
      setTime(t0, tFinal);
      polynomial.setQuadraticWithFinalVelocityConstraint(t0, tFinal, z0, zFinal, zdFinal);
   }

   public void setQuadraticUsingInitialAcceleration(double t0, double tFinal, double z0, double zd0, double zdd0)
   {
      setTime(t0, tFinal);
      polynomial.setQuadraticUsingInitialAcceleration(t0, tFinal, z0, zd0, zdd0);
   }

   public void setQuadraticUsingIntermediatePoint(double t0, double tIntermediate, double tFinal, double z0, double zIntermediate, double zFinal)
   {
      setTime(t0, tFinal);
      polynomial.setQuadraticUsingIntermediatePoint(t0, tIntermediate, tFinal, z0, zIntermediate, zFinal);
   }

   public void setCubicUsingIntermediatePoint(double t0, double tIntermediate1, double tFinal, double z0, double zIntermediate1, double zFinal)
   {
      setTime(t0, tFinal);
      polynomial.setCubicUsingIntermediatePoint(t0, tIntermediate1, tFinal, z0, zIntermediate1, zFinal);
   }

   public void setCubicUsingIntermediatePoints(double t0, double tIntermediate1, double tIntermediate2, double tFinal, double z0, double zIntermediate1,
                                               double zIntermediate2, double zFinal)
   {
      setTime(t0, tFinal);
      polynomial.setCubicUsingIntermediatePoints(t0, tIntermediate1, tIntermediate2, tFinal, z0, zIntermediate1, zIntermediate2, zFinal);
   }

   public void setCubicThreeInitialConditionsFinalPosition(double t0, double tFinal, double z0, double zd0, double zdd0, double zFinal)
   {
      setTime(t0, tFinal);
      polynomial.setCubicThreeInitialConditionsFinalPosition(t0, tFinal, z0, zd0, zdd0, zFinal);
   }

   public void setCubicInitialPositionThreeFinalConditions(double t0, double tFinal, double z0, double zFinal, double zdFinal, double zddFinal)
   {
      setTime(t0, tFinal);
      polynomial.setCubicInitialPositionThreeFinalConditions(t0, tFinal, z0, zFinal, zdFinal, zddFinal);
   }
   
   public void setDirectly(DenseMatrix64F coefficients)
   {
      polynomial.setDirectly(coefficients);
   }

   public void compute(double x)
   {
      polynomial.compute(x);
   } 

   public double getIntegral(double from, double to)
   {
      //if (from < tInitial.getDoubleValue() || to > tFinal.getDoubleValue())
      //   return Double.NaN;
      return polynomial.getIntegral(from, to);
   }
   
   public double getDerivative(int order, double x)
   {
      return polynomial.getDerivative(order, x);
   }

   /**
    *  Returns the order-th derivative of the xPowers vector at value x (Note: does NOT return the YoPolynomials order-th derivative at x)
    * @param order
    * @param x
    * @return
    */
   public DenseMatrix64F getXPowersDerivativeVector(int order, double x)
   {
      //if (MathTools.intervalContains(x, tInitial.getDoubleValue(), tFinal.getDoubleValue()))
         return polynomial.getXPowersDerivativeVector(order, x);
      //else
      //   return null;
   }

   public String toString()
   {
      return name + ":\n" + polynomial.toString() + " TInitial: " + tInitial.getDoubleValue() + " TFinal: " + tFinal.getDoubleValue();
   }

   public void getDerivative(YoTrajectory dervTraj, int order)
   {
      if (dervTraj.getMaximumNumberOfCoefficients() < this.getNumberOfCoefficients() - order)
         return;
      dervTraj.polynomial.reshape(getNumberOfCoefficients() - order);
      for (int i = order; i < this.getNumberOfCoefficients(); i++)
      {
         dervTraj.polynomial.setDirectlyFast(i - order, this.polynomial.getDerivativeCoefficient(order, i) * this.polynomial.getCoefficient(i));
      }
      dervTraj.setInitialTime(this.getInitialTime());
      dervTraj.setFinalTime(this.getFinalTime());
   }
   
   public YoPolynomial getPolynomial()
   {
      return polynomial;
   }

   public boolean isValidTrajectory()
   {
      boolean retVal = (getInitialTime() < getFinalTime()) && Double.isFinite(getInitialTime()) && Double.isFinite(getFinalTime());
      double[] coeffs = getCoefficients();
      for(int i = 0; retVal && i < coeffs.length; i++)
         retVal &= Double.isFinite(coeffs[i]);
      return retVal;
   }

   public void set(Trajectory trajectory)
   {
      reshape(trajectory.getNumberOfCoefficients());
      this.tInitial.set(trajectory.getInitialTime());
      this.tFinal.set(trajectory.getFinalTime());
      for(int i = 0; i < getNumberOfCoefficients(); i++)
         polynomial.setDirectly(i, trajectory.getCoefficient(i));
   }
}
