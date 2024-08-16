package us.ihmc.robotics.math.trajectories.interfaces;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.commons.MathTools;
import us.ihmc.robotics.math.trajectories.core.PolynomialTools;

public interface PolynomialBasics extends PolynomialReadOnly
{
   void setConstraintRow(int row, double x, double desiredZDerivative, int derivativeOrderWithPositionBeingZero);

   void reshape(int numberOfCoefficientsRequired);

   default void setCoefficient(int idx, double value)
   {
      setIsConstraintMatrixUpToDate(false);
      setCoefficientUnsafe(idx, value);
   }

   void setCoefficientUnsafe(int idx, double value);

   double getCoefficient(int idx);

   void setNumberOfCoefficients(int numberOfCoefficients);

   void setCurrentTime(double currentTime);

   DMatrixRMaj getCoefficientsVector();

   void solveForCoefficients();

   void setIsConstraintMatrixUpToDate(boolean isConstraintMatrixUpToDate);

   boolean isConstraintMatrixUpToDate();

   @Override
   default void initialize()
   {
      solveForCoefficients();
      commitCoefficientsToMemory();
   }

   default void set(PolynomialReadOnly other)
   {
      reset();

      getTimeInterval().set(other.getTimeInterval());
      reshape(other.getNumberOfCoefficients());
      setNumberOfCoefficients(other.getNumberOfCoefficients());
      setCurrentTime(getCurrentTime());

      int index = 0;
      for (; index < other.getNumberOfCoefficients(); index++)
         setCoefficient(index, other.getCoefficient(index));
      for (; index < getMaximumNumberOfCoefficients(); index++)
         setCoefficient(index, Double.NaN);
      setIsConstraintMatrixUpToDate(false);
   }

   /**
    * Sets the coefficients assuming the following ordering [a0, a1, a2, ..., aN]^T given the following
    * notation:
    * 
    * <pre>
    * p(x) = a0 + a1 x + a2 x^2 + ... + aN x^N
    * </pre>
    * 
    * @param coefficients
    */
   default void setDirectly(DMatrixRMaj coefficients)
   {
      setDirectly(coefficients, 0, coefficients.getNumRows());
   }

   /**
    * Sets the coefficients assuming the following ordering [a0, a1, a2, ..., aN]^T given the following
    * notation:
    * 
    * <pre>
    * p(x) = a0 + a1 x + a2 x^2 + ... + aN x^N
    * </pre>
    * 
    * @param coefficients the column vector containing the coefficients at rows in [{@code startRow},
    *                     {@code startRow + numberOfCoefficients - 1}].
    */
   default void setDirectly(DMatrixRMaj coefficients, int startRow, int numberOfCoefficients)
   {
      reshape(numberOfCoefficients);
      int index = 0;
      for (; index < getNumberOfCoefficients(); index++)
         setCoefficient(index, coefficients.get(startRow + index, 0));
      for (; index < getMaximumNumberOfCoefficients(); index++)
         setCoefficient(index, Double.NaN);
      setIsConstraintMatrixUpToDate(false);
   }

   /**
    * Sets the coefficients assuming the following ordering [aN, ..., a2, a1, a0]^T given the following
    * notation:
    * 
    * <pre>
    * p(x) = a0 + a1 x + a2 x^2 + ... + aN x^N
    * </pre>
    * 
    * @param coefficients
    */
   default void setDirectlyReverse(DMatrixRMaj coefficients)
   {
      setDirectlyReverse(coefficients, 0, coefficients.getNumRows());
   }

   /**
    * Sets the coefficients assuming the following ordering [aN, ..., a2, a1, a0]^T given the following
    * notation:
    * 
    * <pre>
    * p(x) = a0 + a1 x + a2 x^2 + ... + aN x^N
    * </pre>
    * 
    * @param coefficients the column vector containing the coefficients at rows in [{@code startRow},
    *                     {@code startRow + numberOfCoefficients - 1}].
    */
   default void setDirectlyReverse(DMatrixRMaj coefficients, int startRow, int numberOfCoefficients)
   {
      reshape(numberOfCoefficients);

      int index = 0;
      int argIndex = startRow + numberOfCoefficients - 1;

      for (; index < getNumberOfCoefficients(); index++)
         setCoefficient(index, coefficients.get(argIndex--, 0));
      for (; index < getMaximumNumberOfCoefficients(); index++)
         setCoefficient(index, Double.NaN);
      setIsConstraintMatrixUpToDate(false);
   }

   /**
    * Sets the coefficients assuming the following ordering [a0, a1, a2, ..., aN] given the following
    * notation:
    * 
    * <pre>
    * p(x) = a0 + a1 x + a2 x^2 + ... + aN x^N
    * </pre>
    * 
    * @param coefficients
    */
   default void setDirectly(double[] coefficients)
   {
      reshape(coefficients.length);
      int index = 0;
      for (; index < getNumberOfCoefficients(); index++)
         setCoefficient(index, coefficients[index]);
      for (; index < getMaximumNumberOfCoefficients(); index++)
         setCoefficient(index, Double.NaN);
      setIsConstraintMatrixUpToDate(false);
   }

   /**
    * Sets the coefficients assuming the following ordering [aN, ..., a2, a1, a0] given the following
    * notation:
    * 
    * <pre>
    * p(x) = a0 + a1 x + a2 x^2 + ... + aN x^N
    * </pre>
    * 
    * @param coefficients
    */
   default void setDirectlyReverse(double[] coefficients)
   {
      reshape(coefficients.length);

      int index = 0;
      int argIndex = getNumberOfCoefficients() - 1;

      for (; index < getNumberOfCoefficients(); index++)
         setCoefficient(index, coefficients[argIndex--]);
      for (; index < getMaximumNumberOfCoefficients(); index++)
         setCoefficient(index, Double.NaN);
      setIsConstraintMatrixUpToDate(false);
   }

   default void shiftTrajectory(double offsetValue)
   {
      setCoefficient(0, getCoefficient(0) + offsetValue);
   }

   /**
    * Dont use this. It creates garbage
    *
    * @param from
    * @param to
    * @return
    */
   default double getIntegral(double from, double to)
   {
      double[] fromPowers = new double[getNumberOfCoefficients() + 1];
      double[] toPowers = new double[getNumberOfCoefficients() + 1];
      PolynomialTools.setXPowers(fromPowers, from);
      PolynomialTools.setXPowers(toPowers, to);
      double integral = 0;
      for (int i = 0; i < getNumberOfCoefficients(); i++)
      {
         integral += (1.0 / ((double) i + 1.0)) * getCoefficient(i) * (toPowers[i + 1] - fromPowers[i + 1]);
      }
      return integral;
   }

   default void reset()
   {
      getTimeInterval().reset();
      setNumberOfCoefficients(0);
      setIsConstraintMatrixUpToDate(false);
      for (int i = 0; i < getMaximumNumberOfCoefficients(); i++)
         setCoefficient(i, Double.NaN);
   }

   default void setTime(double t0, double tFinal)
   {
      getTimeInterval().setInterval(t0, tFinal);
   }

   default void setFinalTime(double tFinal)
   {
      getTimeInterval().setEndTime(tFinal);
   }

   default void setInitialTime(double t0)
   {
      getTimeInterval().setStartTime(t0);
   }

   default void setZero()
   {
      this.setConstant(0.0);
   }

   default void setConstant(double z)
   {
      setConstant(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, z);
   }

   default void setConstant(double t0, double tFinal, double z)
   {
      setTime(t0, tFinal);
      reshape(1);
      setPositionRow(0, 0.0, z);
      setIsConstraintMatrixUpToDate(true);

      setCoefficient(0, z);
      //      initialize();
   }

   default void setLinear(double t0, double tFinal, double z0, double zf)
   {
      setTime(t0, tFinal);
      reshape(2);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tFinal, zf);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setLinear(double t, double z, double zd)
   {
      setTime(t, Double.POSITIVE_INFINITY);
      reshape(2);
      setPositionRow(0, t, z);
      setVelocityRow(1, t, zd);
      setIsConstraintMatrixUpToDate(true);

      setCoefficient(0, z - zd * t);
      setCoefficient(1, zd);
//      initialize();
   }

   default void setQuintic(double t0, double tFinal, double z0, double zd0, double zdd0, double zf, double zdf, double zddf)
   {
      setTime(t0, tFinal);
      reshape(6);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tFinal, zf);
      setVelocityRow(4, tFinal, zdf);
      setAccelerationRow(5, tFinal, zddf);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setQuinticUsingWayPoint(double t0,
                                        double tIntermediate,
                                        double tFinal,
                                        double z0,
                                        double zd0,
                                        double zdd0,
                                        double zIntermediate,
                                        double zf,
                                        double zdf)
   {
      setTime(t0, tFinal);
      reshape(6);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tIntermediate, zIntermediate);
      setPositionRow(4, tFinal, zf);
      setVelocityRow(5, tFinal, zdf);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setQuinticUsingWayPoint2(double t0,
                                         double tIntermediate,
                                         double tFinal,
                                         double z0,
                                         double zd0,
                                         double zdd0,
                                         double zIntermediate,
                                         double zdIntermediate,
                                         double zf)
   {
      setTime(t0, tFinal);
      reshape(6);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tIntermediate, zIntermediate);
      setVelocityRow(4, tIntermediate, zdIntermediate);
      setPositionRow(5, tFinal, zf);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setQuinticTwoWaypoints(double t0,
                                       double tIntermediate0,
                                       double tIntermediate1,
                                       double tFinal,
                                       double z0,
                                       double zd0,
                                       double zIntermediate0,
                                       double zIntermediate1,
                                       double zf,
                                       double zdf)
   {
      setTime(t0, tFinal);
      reshape(6);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tIntermediate0, zIntermediate0);
      setPositionRow(3, tIntermediate1, zIntermediate1);
      setPositionRow(4, tFinal, zf);
      setVelocityRow(5, tFinal, zdf);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setQuinticUsingIntermediateVelocityAndAcceleration(double t0,
                                                                   double tIntermediate,
                                                                   double tFinal,
                                                                   double z0,
                                                                   double zd0,
                                                                   double zdIntermediate,
                                                                   double zddIntermediate,
                                                                   double zFinal,
                                                                   double zdFinal)
   {
      setTime(t0, tFinal);
      reshape(6);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setVelocityRow(2, tIntermediate, zdIntermediate);
      setAccelerationRow(3, tIntermediate, zddIntermediate);
      setPositionRow(4, tFinal, zFinal);
      setVelocityRow(5, tFinal, zdFinal);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setQuarticUsingOneIntermediateVelocity(double t0,
                                                       double tIntermediate0,
                                                       double tIntermediate1,
                                                       double tFinal,
                                                       double z0,
                                                       double zIntermediate0,
                                                       double zIntermediate1,
                                                       double zFinal,
                                                       double zdIntermediate1)
   {
      setTime(t0, tFinal);
      reshape(5);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tIntermediate0, zIntermediate0);
      setPositionRow(2, tIntermediate1, zIntermediate1);
      setVelocityRow(3, tIntermediate1, zdIntermediate1);
      setPositionRow(4, tFinal, zFinal);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setQuinticWithZeroTerminalAcceleration(double t0, double tFinal, double z0, double zd0, double zFinal, double zdFinal)
   {
      setQuintic(t0, tFinal, z0, zd0, 0.0, zFinal, zdFinal, 0.0);
   }

   default void setSexticUsingWaypoint(double t0,
                                       double tIntermediate,
                                       double tFinal,
                                       double z0,
                                       double zd0,
                                       double zdd0,
                                       double zIntermediate,
                                       double zf,
                                       double zdf,
                                       double zddf)
   {
      setTime(t0, tFinal);
      reshape(7);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tIntermediate, zIntermediate);
      setPositionRow(4, tFinal, zf);
      setVelocityRow(5, tFinal, zdf);
      setAccelerationRow(6, tFinal, zddf);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setSeptic(double t0,
                          double tIntermediate0,
                          double tIntermediate1,
                          double tFinal,
                          double z0,
                          double zd0,
                          double zIntermediate0,
                          double zdIntermediate0,
                          double zIntermediate1,
                          double zdIntermediate1,
                          double zf,
                          double zdf)
   {
      setTime(t0, tFinal);
      reshape(8);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tIntermediate0, zIntermediate0);
      setVelocityRow(3, tIntermediate0, zdIntermediate0);
      setPositionRow(4, tIntermediate1, zIntermediate1);
      setVelocityRow(5, tIntermediate1, zdIntermediate1);
      setPositionRow(6, tFinal, zf);
      setVelocityRow(7, tFinal, zdf);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setSepticInitialAndFinalAcceleration(double t0,
                                                     double tIntermediate0,
                                                     double tIntermediate1,
                                                     double tFinal,
                                                     double z0,
                                                     double zd0,
                                                     double zdd0,
                                                     double zIntermediate0,
                                                     double zIntermediate1,
                                                     double zf,
                                                     double zdf,
                                                     double zddf)
   {
      setTime(t0, tFinal);
      reshape(8);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tIntermediate0, zIntermediate0);
      setPositionRow(4, tIntermediate1, zIntermediate1);
      setPositionRow(5, tFinal, zf);
      setVelocityRow(6, tFinal, zdf);
      setAccelerationRow(7, tFinal, zddf);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setNonic(double t0,
                         double tIntermediate0,
                         double tIntermediate1,
                         double tFinal,
                         double z0,
                         double zd0,
                         double zIntermediate0,
                         double zdIntermediate0,
                         double zIntermediate1,
                         double zdIntermediate1,
                         double zf,
                         double zdf)
   {
      setTime(t0, tFinal);
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
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setSexticUsingWaypointVelocityAndAcceleration(double t0,
                                                              double tIntermediate,
                                                              double tFinal,
                                                              double z0,
                                                              double zd0,
                                                              double zdd0,
                                                              double zdIntermediate,
                                                              double zddIntermediate,
                                                              double zFinal,
                                                              double zdFinal)
   {
      setTime(t0, tFinal);
      reshape(7);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setVelocityRow(3, tIntermediate, zdIntermediate);
      setAccelerationRow(4, tIntermediate, zddIntermediate);
      setPositionRow(5, tFinal, zFinal);
      setVelocityRow(6, tFinal, zdFinal);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setQuarticUsingIntermediateVelocity(double t0,
                                                    double tIntermediate,
                                                    double tFinal,
                                                    double z0,
                                                    double zd0,
                                                    double zdIntermediate,
                                                    double zFinal,
                                                    double zdFinal)
   {
      setTime(t0, tFinal);
      reshape(5);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setVelocityRow(2, tIntermediate, zdIntermediate);
      setPositionRow(3, tFinal, zFinal);
      setVelocityRow(4, tFinal, zdFinal);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setQuartic(double t0, double tFinal, double z0, double zd0, double zdd0, double zFinal, double zdFinal)
   {
      setTime(t0, tFinal);
      reshape(5);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tFinal, zFinal);
      setVelocityRow(4, tFinal, zdFinal);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setQuarticUsingMidPoint(double t0, double tFinal, double z0, double zd0, double zMid, double zFinal, double zdFinal)
   {
      double tMid = t0 + (tFinal - t0) / 2.0;
      setQuarticUsingWayPoint(t0, tMid, tFinal, z0, zd0, zMid, zFinal, zdFinal);
   }

   default void setQuarticUsingWayPoint(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zIntermediate, double zf, double zdf)
   {
      setTime(t0, tFinal);
      reshape(5);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tIntermediate, zIntermediate);
      setPositionRow(3, tFinal, zf);
      setVelocityRow(4, tFinal, zdf);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setQuarticUsingFinalAcceleration(double t0, double tFinal, double z0, double zd0, double zFinal, double zdFinal, double zddFinal)
   {
      setTime(t0, tFinal);
      reshape(5);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tFinal, zFinal);
      setVelocityRow(3, tFinal, zdFinal);
      setAccelerationRow(4, tFinal, zddFinal);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setCubic(double t0, double tFinal, double z0, double zFinal)
   {
      setTime(t0, tFinal);
      reshape(4);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, 0.0);
      setPositionRow(2, tFinal, zFinal);
      setVelocityRow(3, tFinal, 0.0);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setCubic(double t0, double tFinal, double z0, double zd0, double zFinal, double zdFinal)
   {
      setTime(t0, tFinal);
      reshape(4);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tFinal, zFinal);
      setVelocityRow(3, tFinal, zdFinal);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setCubicDirectly(double duration, double z0, double zd0, double zFinal, double zdFinal)
   {
      reset();

      getTimeInterval().setIntervalUnsafe(0.0, duration);
      reshape(4);

      double d2 = duration * duration;
      double d3 = duration * d2;
      double c3 = 2.0 / d3 * (z0 - zFinal) + 1.0 / d2 * (zd0 + zdFinal);
      double c2 = 1.0 / (2.0 * duration) * (zdFinal - zd0) - 1.5 * duration * c3;
      setCoefficient(0, z0);
      setCoefficient(1, zd0);
      setCoefficient(2, c2);
      setCoefficient(3, c3);
      setIsConstraintMatrixUpToDate(false);
   }

   default void setCubicWithIntermediatePositionAndInitialVelocityConstraint(double t0,
                                                                             double tIntermediate,
                                                                             double tFinal,
                                                                             double z0,
                                                                             double zd0,
                                                                             double zIntermediate,
                                                                             double zFinal)
   {
      setTime(t0, tFinal);
      reshape(4);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tIntermediate, zIntermediate);
      setPositionRow(3, tFinal, zFinal);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setCubicWithIntermediatePositionAndFinalVelocityConstraint(double t0,
                                                                           double tIntermediate,
                                                                           double tFinal,
                                                                           double z0,
                                                                           double zIntermediate,
                                                                           double zFinal,
                                                                           double zdFinal)
   {
      setTime(t0, tFinal);
      reshape(4);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tIntermediate, zIntermediate);
      setPositionRow(2, tFinal, zFinal);
      setVelocityRow(3, tFinal, zdFinal);
      setIsConstraintMatrixUpToDate(true);
      initialize();
   }

   default void setCubicBezier(double t0, double tFinal, double z0, double zR1, double zR2, double zFinal)
   {
      setTime(t0, tFinal);
      reshape(4);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tFinal, zFinal);
      setVelocityRow(2, t0, 3 * (zR1 - z0) / (tFinal - t0));
      setVelocityRow(3, tFinal, 3 * (zFinal - zR2) / (tFinal - t0));
      setIsConstraintMatrixUpToDate(true);
      initialize();
   }


   default void setCubicUsingFinalAccelerationButNotFinalPosition(double t0, double tFinal, double z0, double zd0, double zdFinal, double zddFinal)
   {
      setTime(t0, tFinal);
      reshape(4);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setVelocityRow(2, tFinal, zdFinal);
      setAccelerationRow(3, tFinal, zddFinal);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setCubicUsingFinalAccelerationButNotFinalVelocity(double t0, double tFinal, double z0, double zd0, double zFinal, double zddFinal)
   {
      setTime(t0, tFinal);
      reshape(4);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tFinal, zFinal);
      setAccelerationRow(3, tFinal, zddFinal);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setQuadratic(double t0, double tFinal, double z0, double zd0, double zFinal)
   {
      setTime(t0, tFinal);
      reshape(3);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tFinal, zFinal);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setQuadraticWithFinalVelocityConstraint(double t0, double tFinal, double z0, double zFinal, double zdFinal)
   {
      setTime(t0, tFinal);
      reshape(3);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tFinal, zFinal);
      setVelocityRow(2, tFinal, zdFinal);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setQuadraticUsingInitialAcceleration(double t0, double tFinal, double z0, double zd0, double zdd0)
   {
      setTime(t0, tFinal);
      reshape(3);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setQuadraticUsingIntermediatePoint(double t0, double tIntermediate, double tFinal, double z0, double zIntermediate, double zFinal)
   {
      setTime(t0, tFinal);
      reshape(3);
      MathTools.checkIntervalContains(tIntermediate, t0, tFinal);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tIntermediate, zIntermediate);
      setPositionRow(2, tFinal, zFinal);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setCubicUsingIntermediatePoint(double t0, double tIntermediate1, double tFinal, double z0, double zIntermediate1, double zFinal)
   {
      setTime(t0, tFinal);
      reshape(4);
      MathTools.checkIntervalContains(tIntermediate1, t0, tFinal);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tIntermediate1, zIntermediate1);
      setPositionRow(2, tFinal, zFinal);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setCubicUsingIntermediatePoints(double t0,
                                                double tIntermediate1,
                                                double tIntermediate2,
                                                double tFinal,
                                                double z0,
                                                double zIntermediate1,
                                                double zIntermediate2,
                                                double zFinal)
   {
      setTime(t0, tFinal);
      reshape(4);
      MathTools.checkIntervalContains(tIntermediate1, t0, tIntermediate1);
      MathTools.checkIntervalContains(tIntermediate2, tIntermediate1, tFinal);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tIntermediate1, zIntermediate1);
      setPositionRow(2, tIntermediate2, zIntermediate2);
      setPositionRow(3, tFinal, zFinal);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setCubicThreeInitialConditionsFinalPosition(double t0, double tFinal, double z0, double zd0, double zdd0, double zFinal)
   {
      setTime(t0, tFinal);
      reshape(4);

      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tFinal, zFinal);
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setCubicInitialPositionThreeFinalConditions(double t0, double tFinal, double z0, double zFinal, double zdFinal, double zddFinal)
   {
      setTime(t0, tFinal);
      reshape(4);

      setPositionRow(0, t0, z0);
      setPositionRow(1, tFinal, zFinal);
      setVelocityRow(2, tFinal, zdFinal);
      setAccelerationRow(3, tFinal, zddFinal);

      setIsConstraintMatrixUpToDate(true);

      initialize();
   }

   default void setInitialPositionVelocityZeroFinalHighOrderDerivatives(double t0, double tFinal, double z0, double zd0, double zFinal, double zdFinal)
   {
      if (getMaximumNumberOfCoefficients() < 4)
         throw new RuntimeException("Need at least 4 coefficients in order to set initial and final positions and velocities");
      setTime(t0, tFinal);
      reshape(getMaximumNumberOfCoefficients());
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tFinal, zFinal);
      setVelocityRow(3, tFinal, zdFinal);

      int order = 2;

      for (int row = 4; row < getMaximumNumberOfCoefficients(); row++)
      {
         setConstraintRow(row, tFinal, 0.0, order++);
      }
      setIsConstraintMatrixUpToDate(true);

      initialize();
   }


   default void setPositionRow(int row, double x, double z)
   {
      setIsConstraintMatrixUpToDate(false);
      setConstraintRow(row, x, z, 0);
   }

   default void setVelocityRow(int row, double x, double zVelocity)
   {
      setIsConstraintMatrixUpToDate(false);
      setConstraintRow(row, x, zVelocity, 1);
   }

   default void setAccelerationRow(int row, double x, double zAcceleration)
   {
      setIsConstraintMatrixUpToDate(false);
      setConstraintRow(row, x, zAcceleration, 2);
   }

   default void commitCoefficientsToMemory()
   {
      if (!isConstraintMatrixUpToDate())
         throw new RuntimeException("The constraint matrix is out of date, setting from it will change data.");

      int row = 0;
      for (; row < getNumberOfCoefficients(); row++)
         setCoefficient(row, getCoefficientsVector().get(row, 0));
      for (; row < getMaximumNumberOfCoefficients(); row++)
         setCoefficient(row, Double.NaN);

      setIsConstraintMatrixUpToDate(true);
   }
}
