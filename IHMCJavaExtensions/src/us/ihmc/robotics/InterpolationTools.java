package us.ihmc.robotics;

public class InterpolationTools
{
   /**
    * Performs a piecewise (nearest neighbor) interpolation from {@code a} to {@code b} given the percentage
    * {@code alpha}. Rounds up.
    *
    * @param a the first value used in the interpolation.
    * @param b the second value used in the interpolation.
    * @param alpha the percentage to use for the interpolation. A value of 0 will return {@code a},
    *           while a value of 1 will return {@code b}.
    * @return the interpolated value.
    */
   public static double piecewiseInterpolate(double a, double b, double alpha)
   {
      if (alpha < 0.5)
         return a;
      else
         return b;
   }

   /**
    * Performs a piecewise (nearest neighbor) from {@code a} to {@code b} given the percentage
    * {@code alpha}. Rounds up.
    *
    * @param a the first value used in the interpolation.
    * @param b the second value used in the interpolation.
    * @param alpha the percentage to use for the interpolation. A value of 0 will return {@code a},
    *           while a value of 1 will return {@code b}.
    * @return the interpolated value.
    */
   public static double piecewiseInterpolateRoundDown(double a, double b, double alpha)
   {
      if (alpha > 0.5)
         return b;
      else
         return a;
   }

   /**
    * Performs a linear interpolation from {@code a} to {@code b} given the percentage
    * {@code alpha}.
    * <p>
    * result = (1.0 - alpha) * a + alpha * b
    * </p>
    *
    * @param a the first value used in the interpolation.
    * @param b the second value used in the interpolation.
    * @param alpha the percentage to use for the interpolation. A value of 0 will return {@code a},
    *           while a value of 1 will return {@code b}.
    * @return the interpolated value.
    */
   public static double linearInterpolate(double a, double b, double alpha)
   {
      return (1.0 - alpha) * a + alpha * b;
   }

   /**
    * Performs an interpolation from {@code a} to {@code b} using a cubic Hermite 0,1 spline
    * given the percentage
    * {@code alpha}.
    * <p>
    * result = -2.0 * alpha^3.0 + 3.0 * alpha^2.0
    * </p>
    * <p>
    * result = (1.0 - beta) * a + beta * b
    * </p>
    *
    * @param a the first value used in the interpolation.
    * @param b the second value used in the interpolation.
    * @param alpha the percentage to use for the interpolation. A value of 0 will return {@code a},
    *           while a value of 1 will return {@code b}.
    * @return the interpolated value.
    */
   public static double hermite01Interpolate(double a, double b, double alpha)
   {
      double beta = hermite01Coefficient(alpha);
      return linearInterpolate(a, b, beta);
   }

   /**
    * Performs an interpolation from {@code a} to {@code b} using a cubic Hermite spline
    * given the percentage {@code alpha}. In this implementation, the tangents at the
    * boundaries are assumed to be zero.
    *
    * @param a the first value used in the interpolation.
    * @param b the second value used in the interpolation.
    * @param alpha the percentage to use for the interpolation. A value of 0 will return {@code a},
    *           while a value of 1 will return {@code b}.
    * @return the interpolated value.
    * @throws IllegalArgumentException If the interpolation value would produce a result outside
    * the set [{@code a} and {@code b}]
    */
   public static double hermiteInterpolate(double a, double b, double alpha) throws IllegalArgumentException
   {
      alpha = MathTools.clamp(alpha, 0.0, 1.0);

      double h00 = hermite00Coefficient(alpha);
      double h01 = hermite01Coefficient(alpha);
      return h00 * a + h01 * b;
   }

   /**
    * Performs an interpolation from {@code a} to {@code b} using a cubic Hermite spline
    * given the percentage {@code alpha}.
    *
    * @param a the first value used in the interpolation.
    * @param aTangent the tangent at the first value in the interpolation.
    * @param b the second value used in the interpolation.
    * @param bTangent the tangent at the second value in the interpolation.
    * @param alpha the percentage to use for the interpolation. A value of 0 will return {@code a},
    *           while a value of 1 will return {@code b}.
    * @return the interpolated value.
    * @throws IllegalArgumentException If the interpolation value would produce a result outside
    * the set [{@code a} and {@code b}]
    */
   public static double hermiteInterpolate(double a, double aTangent, double b, double bTangent, double alpha) throws IllegalArgumentException
   {
      alpha = MathTools.clamp(alpha, 0.0, 1.0);

      double h00 = hermite00Coefficient(alpha);
      double h10 = hermite10Coefficient(alpha);
      double h01 = hermite01Coefficient(alpha);
      double h11 = hermite11Coefficient(alpha);
      return h00 * a + h10 * aTangent + h01 * b + h11 * bTangent;
   }

   /**
    * Calculates the coefficient for the initial value (value when {@code alpha} = 0.0)
    * of the Hermite Cubic spline for interpolation.
    * <p>
    *    result = 2.0 * alpha^3.0 - 3.0 * alpha^2 + 1.0
    * </p>
    * @param alpha the percentage to use for the interpolation.
    * @return coefficient
    */
   public static double hermite00Coefficient(double alpha)
   {
      return 2.0 * Math.pow(alpha, 3.0) - 3.0 * Math.pow(alpha, 2.0) + 1.0;
   }

   /**
    * Calculates the coefficient for the initial tangent value
    * (tangent value when {@code alpha} = 0.0) of the Hermite Cubic spline for interpolation.
    * <p>
    *    result = alpha^3.0 - 2.0 * alpha^2 + alpha
    * </p>
    * @param alpha the percentage to use for the interpolation.
    * @return coefficient
    */
   public static double hermite10Coefficient(double alpha)
   {
      return Math.pow(alpha, 3.0) - 2.0 * Math.pow(alpha, 2.0) + alpha;
   }

   /**
    * Calculates the coefficient for the final value (value when {@code alpha} = 1.0)
    * of the Hermite Cubic spline for interpolation.
    * <p>
    *    result = -2.0 * alpha^3.0 + 3.0 * alpha^2
    * </p>
    * @param alpha the percentage to use for the interpolation.
    * @return coefficient
    */
   public static double hermite01Coefficient(double alpha)
   {
      return -2.0 * Math.pow(alpha, 3.0) + 3.0 * Math.pow(alpha, 2.0);
   }

   /**
    * Calculates the coefficient for the final tangent value
    * (tangent value when {@code alpha} = 1.0) of the Hermite Cubic spline for interpolation.
    * <p>
    *    result = alpha^3.0 - alpha^2
    * </p>
    * @param alpha the percentage to use for the interpolation.
    * @return coefficient
    */
   public static double hermite11Coefficient(double alpha)
   {
      return Math.pow(alpha, 3.0) - Math.pow(alpha, 2.0);
   }

   /**
    * Performs a logistic interpolation from {@code a} to {@code b} given the percentage
    * {@code alpha}. Assumes a function steepness of 1.0.
    *
    *
    * @param a the first value used in the interpolation.
    * @param b the second value used in the interpolation.
    * @param alpha the percentage to use for the interpolation. A value of 0 will return {@code a},
    *           while a value of 1 will return {@code b}.
    * @return the interpolated value.
    */
   public static double logisticInterpolate(double a, double b, double alpha)
   {
      return logisticInterpolate(a, b, alpha, 1.0);
   }

   /**
    * Performs a logistic interpolation from {@code a} to {@code b} given the percentage
    * {@code alpha}.
    * A higher steepness value causes it to dwell longer at {@code a} and {@code b} and
    * quickly change between teh two values.
    *
    * @param a the first value used in the interpolation.
    * @param b the second value used in the interpolation.
    * @param alpha the percentage to use for the interpolation. A value of 0 will return {@code a},
    *           while a value of 1 will return {@code b}.
    * @param steepness the steepness of the exponential function.
    * @return the interpolated value.
    */
   public static double logisticInterpolate(double a, double b, double alpha, double steepness)
   {
      double value = 12.0 * alpha - 6.0;
      double beta = 1.0 / (1.0 + Math.exp(-steepness * value));
      return linearInterpolate(a, b, beta);
   }
}
