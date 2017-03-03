package us.ihmc.robotics;

import java.util.List;

import us.ihmc.commons.Epsilons;

public class MathTools
{
   private MathTools()
   {
      // Disallow construction
   }
   
   /**
    * Returns the sign of the argument. 1.0 if positive or 0.0, -1.0 if negative.
    * (Unlike Math.signum that returns 0.0 if the argument is 0.0)
    *
    * @param argument double
    * @return double 1.0 if d >= 0.0, else -1.0
    */
   public static double sign(double argument)
   {
      if (argument >= 0.0)
         return 1.0;

      return -1.0;
   }

   /**
    * Adds parameter 'addToAllElements' to all elements of the
    * integer array and returns the new array
    *
    * @param array int[]
    * @param addToAllElementsOfA int
    * @return int[]
    */
   public static int[] dotPlus(int[] array, int addToAllElementsOfA)
   {
      int[] ret = new int[array.length];
      for (int i = 0; i < array.length; i++)
      {
         ret[i] = array[i] + addToAllElementsOfA;
      }

      return ret;
   }

   /**
    * Adds parameter 'addToAllElements' to all elements of the
    * double array and returns the new array
    *
    * @param array double[]
    * @param addToAllElementsOfA double
    * @return double[]
    */
   public static double[] dotPlus(double[] array, double addToAllElementsOfA)
   {
      double[] ret = new double[array.length];
      for (int i = 0; i < array.length; i++)
      {
         ret[i] = array[i] + addToAllElementsOfA;
      }

      return ret;
   }

   /**
    * Subtracts each element of a double array by the previous element in
    * the array, returns the new array
    *
    * @param array double[]
    * @return double[]
    */
   public static double[] diff(double[] array)
   {
      double[] ret = new double[array.length - 1];
      for (int i = 1; i < array.length; i++)
      {
         ret[i - 1] = array[i] - array[i - 1];
      }

      return ret;
   }

   /**
    * True if value |(v1-v2)| <= |epsilon|
    * True if v1 and v2 are Double.NaN
    * True if v1 and v2 are Positive Infinity
    * True if v1 and v2 are Negative Infinity
    * false if not
    *
    * @param v1 double
    * @param v2 double
    * @param epsilon double
    * @return boolean
    */
   public static boolean epsilonEquals(double v1, double v2, double epsilon)
   {
      if(Double.isNaN(v1) && Double.isNaN(v2))
      {
         return true;
      }
      
      //catches infinites
      if(v1 == v2)
      {
         return true;
      }
      
      return Math.abs(v1 - v2) <= Math.abs(epsilon);
   }

   /**
    * True if value |(v1-v2)| <= |epsilon|
    * True if v1 and v2 are Float.NaN
    * True if v1 and v2 are Positive Infinity
    * True if v1 and v2 are Negative Infinity
    * false if not
    *
    * @param v1 float
    * @param v2 float
    * @param epsilon float
    * @return boolean
    */
   public static boolean epsilonEquals(float v1, float v2, float epsilon)
   {
      if(Float.isNaN(v1) && Float.isNaN(v2))
      {
         return true;
      }
      
      //catches infinites
      if(v1 == v2)
      {
         return true;
      }
      
      return Math.abs(v1 - v2) <= Math.abs(epsilon);
   }

   /**
    * True if v2 is within given percent of v1
    * False otherwise
    *
    * @param v1 double
    * @param v2 double
    * @param percent double
    * @return boolean
    */
   public static boolean withinPercentEquals(double v1, double v2, double percent)
   {
      return (Math.abs(v1 - v2) <= Math.abs(percent * v1));
   }

   /**
    * Clamps value to the given range, defined by <code>-minMax</code> and <code>minMax</code>, inclusive.
    *
    * @param value value
    * @param minMax inclusive absolute boundary
    * @return <li><code>-minMax</code> if <code>value</code> is less than <code>-minMax</code></li>
    *         <li><code>minMax</code> if <code>value</code> is greater than <code>minMax</code></li>
    *         <li><code>value</code> if <code>value</code> is between or equal to <code>-minMax</code> and <code>minMax</code></li>
    */
   public static double clamp(double value, double minMax)
   {
      return clamp(value, -minMax, minMax);
   }
   
   /**
    * Clamps value to the given range, defined by <code>-minMax</code> and <code>minMax</code>, inclusive.
    *
    * @param value value
    * @param minMax inclusive absolute boundary
    * @return <li><code>-minMax</code> if <code>value</code> is less than <code>-minMax</code></li>
    *         <li><code>minMax</code> if <code>value</code> is greater than <code>minMax</code></li>
    *         <li><code>value</code> if <code>value</code> is between or equal to <code>-minMax</code> and <code>minMax</code></li>
    */
   public static float clamp(float value, float minMax)
   {
      return clamp(value, -minMax, minMax);
   }
   
   /**
    * Clamps value to the given range, defined by <code>-minMax</code> and <code>minMax</code>, inclusive.
    *
    * @param value value
    * @param minMax inclusive absolute boundary
    * @return <li><code>-minMax</code> if <code>value</code> is less than <code>-minMax</code></li>
    *         <li><code>minMax</code> if <code>value</code> is greater than <code>minMax</code></li>
    *         <li><code>value</code> if <code>value</code> is between or equal to <code>-minMax</code> and <code>minMax</code></li>
    */
   public static int clamp(int value, int minMax)
   {
      return clamp(value, -minMax, minMax);
   }

   /**
    * Clamps value to the given range, inclusive.
    *
    * @param value value
    * @param min inclusive boundary start
    * @param max inclusive boundary end
    * @return <li><code>min</code> if <code>value</code> is less than <code>min</code></li>
    *         <li><code>max</code> if <code>value</code> is greater than <code>max</code></li>
    *         <li><code>value</code> if <code>value</code> is between or equal to <code>min</code> and <code>max</code></li>
    */
   public static double clamp(double value, double min, double max)
   {
      if (min > (max + Epsilons.ONE_TEN_BILLIONTH))
      {
         throw new RuntimeException(MathTools.class.getSimpleName() + ".clamp(double, double, double): min > max (" + min + " > " + max + ")");
      }

      return (Math.min(max, Math.max(value, min)));
   }

   /**
    * Clamps value to the given range, inclusive.
    *
    * @param value value
    * @param min inclusive boundary start
    * @param max inclusive boundary end
    * @return <li><code>min</code> if <code>value</code> is less than <code>min</code></li>
    *         <li><code>max</code> if <code>value</code> is greater than <code>max</code></li>
    *         <li><code>value</code> if <code>value</code> is between or equal to <code>min</code> and <code>max</code></li>
    */
   public static float clamp(float value, float min, float max)
   {
      if (min > (max + Epsilons.ONE_TEN_BILLIONTH))
      {
         throw new RuntimeException(MathTools.class.getSimpleName() + ".clamp(float, float, float): min > max (" + min + " > " + max + ")");
      }
   
      return (Math.min(max, Math.max(value, min)));
   }

   /**
    * Clamps value to the given range, inclusive.
    *
    * @param value value
    * @param min inclusive boundary start
    * @param max inclusive boundary end
    * @return <li><code>min</code> if <code>value</code> is less than <code>min</code></li>
    *         <li><code>max</code> if <code>value</code> is greater than <code>max</code></li>
    *         <li><code>value</code> if <code>value</code> is between or equal to <code>min</code> and <code>max</code></li>
    */
   public static int clamp(int value, int min, int max)
   {
      if (min > max)
      {
         throw new RuntimeException(MathTools.class.getSimpleName() + ".clamp(int, int, int): min > max (" + min + " > " + max + ")");
      }
      
      return (Math.min(max, Math.max(value, min)));
   }
   
   /**
    * <p>Rounds <code>value</code> to the given precision.</p>
    * 
    * <p>Example: roundToPrecision(19.5, 1.0) = 20.0;</p> 
    * 
    * @param value value to round to precision
    * @param precision precision to round to
    * @return <code>value</code> rounded to <code>precision</code>
    */
   public static double roundToPrecision(double value, double precision)
   {
      return Math.round(value / precision) * precision;
   }
   
   /**
    * <p>Floors <code>value</code> to the given precision.</p>
    * 
    * <p>Example: floorToPrecision(19.9, 1.0) = 19.0;</p> 
    * 
    * @param value value to floor to precision
    * @param precision precision to floor to
    * @return <code>value</code> floored to <code>precision</code>
    */
   public static double floorToPrecision(double value, double precision)
   {
      return Math.floor(value / precision) * precision;
   }
   
   /**
    * <p>Ceils <code>value</code> to the given precision.</p>
    * 
    * <p>Example: ceilToPrecision(19.2, 1.0) = 20.0;</p> 
    * 
    * @param value value to ceil to precision
    * @param precision precision to ceil to
    * @return <code>value</code> ceiled to <code>precision</code>
    */
   public static double ceilToPrecision(double value, double precision)
   {
      return Math.ceil(value / precision) * precision;
   }

   /**
    * Returns if the interval contains the given value. Interval is defined by
    * <code>lowerEndpoint</code> and <code>upperEndpoint</code>. Interval can be
    * open or closed using <code>includeLowerEndpoint</code> and
    * <code>includeUpperEndpoint</code>.
    * 
    * @param value the values to check contains
    * @param lowerEndpoint lower endpoint of the interval
    * @param upperEndpoint upper endpoint of the interval
    * @param includeLowerEndpoint whether to return true if <code>value == lowerEndpoint</code>
    * @param includeUpperEndpoint whether to return true if <code>value == upperEndpoint</code>
    * @return <code>lowerEndpoint <(=) value <(=) upperEndpoint</code>
    */
   public static boolean intervalContains(double value, double lowerEndpoint, double upperEndpoint, boolean includeLowerEndpoint, boolean includeUpperEndpoint)
   {
      if (lowerEndpoint > upperEndpoint + Epsilons.ONE_TEN_BILLIONTH)
      {
         throw new RuntimeException(MathTools.class.getSimpleName()
               + ".intervalContains(double, double, double, boolean, boolean): lowerEndpoint > upperEndpoint (" + lowerEndpoint + " > " + upperEndpoint + ")");
      }

      return (includeLowerEndpoint ? value >= lowerEndpoint : value > (lowerEndpoint + Epsilons.ONE_TEN_BILLIONTH))
            && (includeUpperEndpoint ? value <= upperEndpoint : value < (upperEndpoint - Epsilons.ONE_TEN_BILLIONTH));
   }

   /**
    * Returns if the closed interval contains the given value. Interval is defined by
    * <code>lowerEndpoint</code> and <code>upperEndpoint</code>. Interval is closed,
    * meaning that if <code>value == upperEndpoint</code> or <code>value == lowerEndpoint</code>,
    * true is returned.
    * 
    * @param value the values to check contains
    * @param lowerEndpoint lower endpoint of the interval
    * @param upperEndpoint upper endpoint of the interval
    * @return <code>lowerEndpoint <= value <= upperEndpoint</code>
    */
   public static boolean intervalContains(double value, double lowerEndpoint, double upperEndpoint)
   {
      return intervalContains(value, lowerEndpoint, upperEndpoint, true, true);
   }

   /**
    * Returns if the closed interval contains the given value. Interval is defined by
    * <code>-endpointMinMax</code> and <code>endpointMinMax</code>. Interval is closed,
    * meaning that if <code>value == -endpointMinMax</code> or <code>value == endpointMinMax</code>,
    * true is returned.
    * 
    * @param value the values to check contains
    * @param endpointMinMax min-max style parameter defining the interval endpoints as
    * <code>-endpointMinMax</code> and <code>endpointMinMax</code>
    * @return <code>-endpointMinMax <= value <= endpointMinMax</code>
    */
   public static boolean intervalContains(double value, double endpointMinMax)
   {
      return intervalContains(value, -endpointMinMax, endpointMinMax);
   }

   /**
    * Returns if the closed interval contains the given value. Interval is defined by
    * <code>lowerEndpoint</code> and <code>upperEndpoint</code>. Interval is closed,
    * meaning that if <code>value == upperEndpoint</code> or <code>value == lowerEndpoint</code>,
    * true is returned. This method rounds <code>value</code>, <code>lowerEndpoint</code>, and <code>upperEndpoint</code>
    * to the given precision before performing comparisons.
    * 
    * @param value the values to check contains
    * @param lowerEndpoint lower endpoint of the interval
    * @param upperEndpoint upper endpoint of the interval
    * @param precision the precision to round <code>value</code>, <code>lowerEndpoint</code>, and <code>upperEndpoint</code> to
    * @return <code>lowerEndpoint <= value <= upperEndpoint</code>
    */
   public static boolean intervalContains(double value, double lowerEndpoint, double upperEndpoint, double precision)
   {
      return intervalContains(roundToPrecision(value, precision), roundToPrecision(lowerEndpoint, precision), roundToPrecision(upperEndpoint, precision));
   }

   /**
    * Returns if the interval contains the given value. Interval is defined by
    * <code>lowerEndpoint</code> and <code>upperEndpoint</code>. Interval can be
    * open or closed using <code>includeLowerEndpoint</code> and
    * <code>includeUpperEndpoint</code>. This method rounds <code>value</code>,
    * <code>lowerEndpoint</code>, and <code>upperEndpoint</code>
    * to the given precision before performing comparisons.
    * 
    * @param value the values to check contains
    * @param lowerEndpoint lower endpoint of the interval
    * @param upperEndpoint upper endpoint of the interval
    * @param precision the precision to round <code>value</code>, <code>lowerEndpoint</code>, and <code>upperEndpoint</code> to
    * @param includeLowerEndpoint whether to return true if <code>value == lowerEndpoint</code>
    * @param includeUpperEndpoint whether to return true if <code>value == upperEndpoint</code>
    * @return <code>lowerEndpoint <(=) value <(=) upperEndpoint</code>
    */
   public static boolean intervalContains(double value, double lowerEndpoint, double upperEndpoint, double precision, boolean includeLowerEndpoint, boolean includeUpperEndpoint)
   {
      return intervalContains(roundToPrecision(value, precision), roundToPrecision(lowerEndpoint, precision), roundToPrecision(upperEndpoint, precision), includeLowerEndpoint, includeUpperEndpoint);
   }

   /**
    * Throws exception if the closed interval does not contain the given value. Interval is defined by
    * <code>lowerEndpoint</code> and <code>upperEndpoint</code>. Interval is closed,
    * meaning that if <code>value == upperEndpoint</code> or <code>value == lowerEndpoint</code>,
    * no exception is thrown.
    * 
    * @param value the values to check contains
    * @param lowerEndpoint lower endpoint of the interval
    * @param upperEndpoint upper endpoint of the interval
    * @throws RuntimeException if !(<code>lowerEndpoint <= value <= upperEndpoint</code>)
    */
   public static void checkIntervalContains(double value, double lowerEndpoint, double upperEndpoint)
   {
      if (!intervalContains(value, lowerEndpoint, upperEndpoint))
      {
         throw new RuntimeException("Argument " + value + " not in range [" + lowerEndpoint + ", " + upperEndpoint + "].");
      }
   }

   /**
    * Throws exception if the closed interval does not contain the given value. Interval is defined by
    * <code>lowerEndpoint</code> and <code>upperEndpoint</code>. Interval is closed,
    * meaning that if <code>value == upperEndpoint</code> or <code>value == lowerEndpoint</code>,
    * no exception is thrown.
    * 
    * @param value the values to check contains
    * @param lowerEndpoint lower endpoint of the interval
    * @param upperEndpoint upper endpoint of the interval
    * @throws RuntimeException if !(<code>lowerEndpoint <= value <= upperEndpoint</code>)
    */
   public static void checkIntervalContains(long value, long lowerEndpoint, long upperEndpoint)
   {
      if (!intervalContains(value, lowerEndpoint, upperEndpoint))
      {
         throw new RuntimeException("Argument " + value + " not in range [" + lowerEndpoint + ", " + upperEndpoint + "].");
      }
   }

   /**
    * Sums the integers in an array.
    *
    * @param array array of integers
    * @return The sum of integers.
    */
   public static int sum(int[] array)
   {
      int sum = 0;
      for (int i = 0; i < array.length; i++)
      {
         sum += array[i];
      }
      return sum;
   }

   /**
    * Sums the doubles in an array.
    *
    * @param array array of doubles
    * @return The sum of doubles.
    */
   public static double sum(double[] array)
   {
      double sum = 0.0;
      for (int i = 0; i < array.length; i++)
      {
         sum += array[i];
      }
      return sum;
   }

   /**
    * Sums the doubles in a list.
    *
    * @param list list of doubles
    * @return The sum of doubles.
    */
   public static double sum(List<Double> list)
   {
      double sum = 0.0;
      for (int i = 0; i < list.size(); i++)
      {
         sum += list.get(i);
      }
      return sum;
   }


   /**
    * <p>Computes the cumulative sum array for a given array of doubles.</p>
    * 
    * <p>Example: The cumulative sum sequence of the input sequence {a, b, c, ...}
    * is {a, a + b, a + b + c, ...}</p>
    * 
    * @param array input sequence
    * @return Cumulative sum sequence.
    */
   public static double[] cumulativeSum(double[] array)
   {
      double[] ret = new double[array.length];
      double sum = 0.0;
      for (int i = 0; i < array.length; i++)
      {
         sum += array[i];
         ret[i] = sum;
      }

      return ret;
   }

   /**
    * The minimum value in an array of doubles.
    *
    * @param array double[]
    * @return Minimum value.
    */
   public static double min(double[] array)
   {
      double min = Double.MAX_VALUE;
      for (int i = 0; i < array.length; i++)
      {
         min = Math.min(min, array[i]);
      }
      return min;
   }

   /**
    * The maximum value in an array of doubles.
    *
    * @param array double[]
    * @return Maximum value.
    */
   public static double max(double[] array)
   {
      double max = -Double.MAX_VALUE;
      for (int i = 0; i < array.length; i++)
      {
         max = Math.max(max, array[i]);
      }
      return max;
   }

   /**
    * Average value in an array of doubles.
    *
    * @param array double[]
    * @return Average value.
    */
   public static double average(double[] array)
   {
      return sum(array) / array.length;
   }

   /**
    * Average value in a list of doubles.
    *
    * @param list list of doubles
    * @return Average value.
    */
   public static double average(List<Double> list)
   {
      return sum(list) / list.size();
   }

   public static void checkIfPositive(double argument)
   {
      if (argument < 0.0)
         throw new RuntimeException("Argument " + argument + " not positive.");
   }

   public static void checkIfGreaterOrEqual(double argument, double desired)
   {
      if (argument < desired)
         throw new RuntimeException("Argument " + argument + " less than: " + desired);
   }

   public static void checkIfNegative(double argument)
   {
      if (argument > 0.0)
         throw new RuntimeException("Argument " + argument + " not negative.");
   }

   public static void checkIfLessOrEqual(double argument, double desired)
   {
      if (argument > desired)
         throw new RuntimeException("Argument " + argument + " greater than: " + desired);
   }

   public static void checkIfLessOrEqual(int argument, int desired)
   {
      if (argument > desired)
         throw new RuntimeException("Argument " + argument + " greater than: " + desired);
   }

   public static void checkIfEqual(double val, double desired, double epsilon)
   {
      if (!(Math.abs(val - desired) < epsilon))
      {
         throw new RuntimeException("Argument " + val + " does not equal desired " + desired);
      }
   }

   public static void checkIfEqual(int val, int desired)
   {
      if (val != desired)
      {
         throw new RuntimeException("Argument " + val + " does not equal desired " + desired);
      }
   }

   public static double square(double x)
   {
      return x * x;
   }

   public static double cube(double x)
   {
      return x * x * x;
   }

   public static double powWithInteger(double x, int exponent)
   {
      double ret = 1.0;
      if (exponent >= 0)
      {
         for (int i = 0; i < exponent; i++)
            ret *= x;
      }
      else
      {
         for (int i = 0; i > exponent; i--)
            ret /= x;
      }
      return ret;
   }

   public static boolean isSignificantlyGreaterThan(double numberOne, double numberTwo, int significantFigures)
   {
      return roundToSignificantFigures(numberOne, significantFigures) > roundToSignificantFigures(numberTwo, significantFigures);
   }
   
   public static boolean isPreciselyGreaterThan(double numberOne, double numberTwo, double precision)
   {
      return roundToPrecision(numberOne, precision) > roundToPrecision(numberTwo, precision);
   }
   
   public static boolean isSignificantlyGreaterThanOrEqualTo(double numberOne, double numberTwo, int significantFigures)
   {
      return roundToSignificantFigures(numberOne, significantFigures) >= roundToSignificantFigures(numberTwo, significantFigures);
   }
   
   public static boolean isPreciselyGreaterThanOrEqualTo(double numberOne, double numberTwo, double precision)
   {
      return roundToPrecision(numberOne, precision) >= roundToPrecision(numberTwo, precision);
   }

   public static boolean isSignificantlyLessThan(double numberOne, double numberTwo, int significantFigures)
   {
      return roundToSignificantFigures(numberOne, significantFigures) < roundToSignificantFigures(numberTwo, significantFigures);
   }
   
   public static boolean isPreciselyLessThan(double numberOne, double numberTwo, double precision)
   {
      return roundToPrecision(numberOne, precision) < roundToPrecision(numberTwo, precision);
   }
   
   public static boolean isSignificantlyLessThanOrEqualTo(double numberOne, double numberTwo, int significantFigures)
   {
      return roundToSignificantFigures(numberOne, significantFigures) <= roundToSignificantFigures(numberTwo, significantFigures);
   }
   
   public static boolean isPreciselyLessThanOrEqualTo(double numberOne, double numberTwo, double precision)
   {
      return roundToPrecision(numberOne, precision) <= roundToPrecision(numberTwo, precision);
   }
   
   public static long gcd(long a, long b)
   {
      while(b > 0)
      {
         long c = a % b;
         a = b;
         b = c;
      }
      
      return a;
   }
   
   public static long lcm(long... a)
   {
      if(a.length < 2)
      {
         throw new RuntimeException("Need at least two arguments");
      }
      
      if(a.length == 2)
      {
         return Math.abs(a[0] * a[1])/gcd(a[0], a[1]);         
      }
      
      long[] b = new long[a.length - 1];
      System.arraycopy(a, 1, b, 0, b.length);
      
      return lcm(a[0], lcm(b));
   }

   public static double roundToSignificantFigures(double value, int significantFigures)
   {
      if (Math.abs(value) < Double.MIN_VALUE)
      {
         return 0.0;
      }
   
      final double log10 = Math.ceil(Math.log10(Math.abs(value)));
      final int power = significantFigures - (int) log10;
   
      final double magnitude = Math.pow(10, power);
      final long shifted = Math.round(value * magnitude);
      return shifted / magnitude;
   }

   public static int orderOfMagnitude(double number)
   {
      return (int) Math.floor(Math.log10(Math.abs(number)));
   }

   /**
    * Returns value - |deadband| if value is greater than |deadband|
    * Returns value + |deadband| if value is less than -|deadband|
    * Returns 0 if value is in the range [-deadband, deadband]
    *
    * @param value double
    * @param deadband double
    * @return double
    */
   public static double applyDeadband(double value, double deadband)
   {
      deadband = Math.abs(deadband);

      if (value > deadband)
      {
         return value - deadband;
      }
      else if (value < -deadband)
      {
         return value + deadband;
      }
      else
      {
         return 0.0;
      }
   }
}
