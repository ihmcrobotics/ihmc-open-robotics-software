package us.ihmc.robotics;

import java.math.BigDecimal;
import java.math.MathContext;
import java.math.RoundingMode;
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
    * True if value |(a-b)| <= epsilon
    * True if {@code a} and b are Positive Infinity
    * True if a and b are Negative Infinity
    * False if a or b is Double.NaN
    * false if not
    *
    * @param a double
    * @param b double
    * @param epsilon double
    * @return boolean
    * @throws RuntimeException is epsilon is less than zero.
    */
   public static boolean epsilonEquals(double a, double b, double epsilon)
   {
      if (epsilon < 0.0)
      {
         throw new RuntimeException("epilson is less than 0.0");
      }
      
      if (Double.isNaN(a) || Double.isNaN(b))
      {
         return false;
      }
      if (Double.compare(a, b) == 0)
      {
         return true;
      }
      if (Math.abs(a - b) <= epsilon)
      {
         return true;
      }

      return false;
   }

   /**
    * True if value |(a-b)| <= epsilon
    * True if a and b are Positive Infinity
    * True if a and b are Negative Infinity
    * True if a and b are Double.NaN
    * false if not
    *
    * @param a double
    * @param b double
    * @param epsilon double
    * @return boolean
    * @throws RuntimeException is epsilon is less than zero.
    */
   public static boolean epsilonCompare(double a, double b, double epsilon)
   {
      if (epsilon < 0.0)
      {
         throw new RuntimeException("epilson is less than 0.0");
      }
      
      if (Double.compare(a, b) == 0)
      {
         return true;
      }
      if (Math.abs(a - b) <= epsilon)
      {
         return true;
      }
   
      return false;
   }

   /**
    * True if b is within given percent of a.
    * False otherwise. Uses {@link MathTools#epsilonEquals(double, double, double)}
    * 
    * @see {@link MathTools#epsilonEquals(double, double, double)}
    *
    * @param a double
    * @param b double
    * @param percent double
    * @return boolean
    */
   public static boolean percentEquals(double a, double b, double percent)
   {
      return epsilonEquals(a, b, Math.abs(percent * a));
   }

   /**
    * Clamps value to the given range, defined by {@code -minMax} and {@code minMax}, inclusive.
    *
    * @param value value
    * @param minMax inclusive absolute boundary
    * @return <li>{@code -minMax} if {@code value} is less than {@code -minMax}</li>
    *         <li>{@code minMax} if {@code value} is greater than {@code minMax}</li>
    *         <li>{@code value} if {@code value} is between or equal to {@code -minMax} and {@code minMax}</li>
    */
   public static double clamp(double value, double minMax)
   {
      return clamp(value, -minMax, minMax);
   }
   
   /**
    * Clamps value to the given range, defined by {@code -minMax} and {@code minMax}, inclusive.
    *
    * @param value value
    * @param minMax inclusive absolute boundary
    * @return <li>{@code -minMax} if {@code value} is less than {@code -minMax}</li>
    *         <li>{@code minMax} if {@code value} is greater than {@code minMax}</li>
    *         <li>{@code value} if {@code value} is between or equal to {@code -minMax} and {@code minMax}</li>
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
    * @return <li>{@code min} if {@code value} is less than {@code min}</li>
    *         <li>{@code max} if {@code value} is greater than {@code max}</li>
    *         <li>{@code value} if {@code value} is between or equal to {@code min} and {@code max}</li>
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
    * @return <li>{@code min} if {@code value} is less than {@code min}</li>
    *         <li>{@code max} if {@code value} is greater than {@code max}</li>
    *         <li>{@code value} if {@code value} is between or equal to {@code min} and {@code max}</li>
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
    * <p>Rounds {@code value} to the given precision.</p>
    * 
    * <p>Example: roundToPrecision(19.5, 1.0) = 20.0;</p> 
    * 
    * @param value value to round to precision
    * @param precision precision to round to
    * @return {@code value} rounded to {@code precision}
    */
   public static double roundToPrecision(double value, double precision)
   {
      return Math.round(value / precision) * precision;
   }
   
   /**
    * <p>Rounds to number of significant figures.</p>
    * 
    * <p>NOTE: For now, this method creates garbage. A garbage free solution needs to be found.</p>
    * 
    * @param value
    * @param significantFigures
    * @return Rounded to significant figures.
    */
   public static double roundToSignificantFigures(double value, int significantFigures)
   {
      if (Math.abs(value) < Double.MIN_VALUE)
      {
         return 0.0;
      }
   
      return new BigDecimal(value, new MathContext(significantFigures, RoundingMode.HALF_UP)).doubleValue();
   }

   /**
    * <p>Floors {@code value} to the given precision.</p>
    * 
    * <p>Example: floorToPrecision(19.9, 1.0) = 19.0;</p> 
    * 
    * @param value value to floor to precision
    * @param precision precision to floor to
    * @return {@code value} floored to {@code precision}
    */
   public static double floorToPrecision(double value, double precision)
   {
      return Math.floor(value / precision) * precision;
   }
   
   /**
    * <p>Ceils {@code value} to the given precision.</p>
    * 
    * <p>Example: ceilToPrecision(19.2, 1.0) = 20.0;</p> 
    * 
    * @param value value to ceil to precision
    * @param precision precision to ceil to
    * @return {@code value} ceiled to {@code precision}
    */
   public static double ceilToPrecision(double value, double precision)
   {
      return Math.ceil(value / precision) * precision;
   }

   /**
    * Returns if the interval contains the given value. Interval is defined by
    * {@code lowerEndpoint} and {@code upperEndpoint}. Interval can be
    * open or closed using {@code includeLowerEndpoint} and
    * {@code includeUpperEndpoint}.
    * 
    * @param value the values to check contains
    * @param lowerEndpoint lower endpoint of the interval
    * @param upperEndpoint upper endpoint of the interval
    * @param includeLowerEndpoint whether to return true if {@code value == lowerEndpoint}
    * @param includeUpperEndpoint whether to return true if {@code value == upperEndpoint}
    * @return {@code lowerEndpoint <(=) value <(=) upperEndpoint}
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
    * {@code lowerEndpoint} and {@code upperEndpoint}. Interval is closed,
    * meaning that if {@code value == upperEndpoint} or {@code value == lowerEndpoint},
    * true is returned.
    * 
    * @param value the values to check contains
    * @param lowerEndpoint lower endpoint of the interval
    * @param upperEndpoint upper endpoint of the interval
    * @return {@code lowerEndpoint <= value <= upperEndpoint}
    */
   public static boolean intervalContains(double value, double lowerEndpoint, double upperEndpoint)
   {
      return intervalContains(value, lowerEndpoint, upperEndpoint, true, true);
   }

   /**
    * Returns if the closed interval contains the given value. Interval is defined by
    * {@code -endpointMinMax} and {@code endpointMinMax}. Interval is closed,
    * meaning that if {@code value == -endpointMinMax} or {@code value == endpointMinMax},
    * true is returned.
    * 
    * @param value the values to check contains
    * @param endpointMinMax min-max style parameter defining the interval endpoints as
    * {@code -endpointMinMax} and {@code endpointMinMax}
    * @return {@code -endpointMinMax <= value <= endpointMinMax}
    */
   public static boolean intervalContains(double value, double endpointMinMax)
   {
      return intervalContains(value, -endpointMinMax, endpointMinMax);
   }

   /**
    * Returns if the closed interval contains the given value. Interval is defined by
    * {@code lowerEndpoint} and {@code upperEndpoint}. Interval is closed,
    * meaning that if {@code value == upperEndpoint} or {@code value == lowerEndpoint},
    * true is returned. This method rounds {@code value}, {@code lowerEndpoint}, and {@code upperEndpoint}
    * to the given precision before performing comparisons.
    * 
    * @param value the values to check contains
    * @param lowerEndpoint lower endpoint of the interval
    * @param upperEndpoint upper endpoint of the interval
    * @param precision the precision to round {@code value}, {@code lowerEndpoint}, and {@code upperEndpoint} to
    * @return {@code lowerEndpoint <= value <= upperEndpoint}
    */
   public static boolean intervalContains(double value, double lowerEndpoint, double upperEndpoint, double precision)
   {
      return intervalContains(roundToPrecision(value, precision), roundToPrecision(lowerEndpoint, precision), roundToPrecision(upperEndpoint, precision));
   }

   /**
    * Returns if the interval contains the given value. Interval is defined by
    * {@code lowerEndpoint} and {@code upperEndpoint}. Interval can be
    * open or closed using {@code includeLowerEndpoint} and
    * {@code includeUpperEndpoint}. This method rounds {@code value},
    * {@code lowerEndpoint}, and {@code upperEndpoint}
    * to the given precision before performing comparisons.
    * 
    * @param value the values to check contains
    * @param lowerEndpoint lower endpoint of the interval
    * @param upperEndpoint upper endpoint of the interval
    * @param precision the precision to round {@code value}, {@code lowerEndpoint}, and {@code upperEndpoint} to
    * @param includeLowerEndpoint whether to return true if {@code value == lowerEndpoint}
    * @param includeUpperEndpoint whether to return true if {@code value == upperEndpoint}
    * @return {@code lowerEndpoint <(=) value <(=) upperEndpoint}
    */
   public static boolean intervalContains(double value, double lowerEndpoint, double upperEndpoint, double precision, boolean includeLowerEndpoint, boolean includeUpperEndpoint)
   {
      return intervalContains(roundToPrecision(value, precision), roundToPrecision(lowerEndpoint, precision), roundToPrecision(upperEndpoint, precision), includeLowerEndpoint, includeUpperEndpoint);
   }

   /**
    * Throws exception if the closed interval does not contain the given value. Interval is defined by
    * {@code lowerEndpoint} and {@code upperEndpoint}. Interval is closed,
    * meaning that if {@code value == upperEndpoint} or {@code value == lowerEndpoint},
    * no exception is thrown.
    * 
    * @param value the values to check contains
    * @param lowerEndpoint lower endpoint of the interval
    * @param upperEndpoint upper endpoint of the interval
    * @throws RuntimeException if !({@code lowerEndpoint <= value <= upperEndpoint})
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
    * {@code lowerEndpoint} and {@code upperEndpoint}. Interval is closed,
    * meaning that if {@code value == upperEndpoint} or {@code value == lowerEndpoint},
    * no exception is thrown.
    * 
    * @param value the values to check contains
    * @param lowerEndpoint lower endpoint of the interval
    * @param upperEndpoint upper endpoint of the interval
    * @throws RuntimeException if !({@code lowerEndpoint <= value <= upperEndpoint})
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

   /**
    * Throw exception if value is less than zero.
    * No exception thrown if {@code value == 0.0}.
    * 
    * @param value value to check
    * @throws RuntimeException if value is negative.
    */
   public static void checkPositive(double value)
   {
      if (value < 0.0)
      {
         throw new RuntimeException("Value " + value + " is negative.");
      }
   }

   /**
    * Throw exception if value is greater than zero.
    * No exception thrown if {@code value == 0.0}.
    * 
    * @param value value to check
    * @throws RuntimeException if value is positive.
    */
   public static void checkNegative(double value)
   {
      if (value > 0.0)
      {
         throw new RuntimeException("Value " + value + " is positive.");
      }
   }

   /**
    * Throw exception if {@code greater} not greater than or equal to {@code lesser}.
    * No exception thrown if {@code greater == lesser}.
    * 
    * @param greater greater value
    * @param lesser lesser value
    * @throws RuntimeException if {@code greater} not greater than or equal to {@code lesser}.
    */
   public static void checkGreaterThanOrEquals(double greater, double lesser)
   {
      if (greater < lesser)
      {
         throw new RuntimeException("Not greater than or equal. " + greater + " < " + lesser);
      }
   }

   /**
    * Throw exception if {@code lesser} not less than or equal to {@code greater}.
    * No exception thrown if {@code lesser == greater}.
    * 
    * @param lesser lesser value
    * @param greater greater value
    * @throws RuntimeException if {@code lesser} not less than or equal to {@code greater}.
    */
   public static void checkLessThanOrEquals(double lesser, double greater)
   {
      if (lesser > greater)
      {
         throw new RuntimeException("Not less than or equal. " + lesser + " > " + greater);
      }
   }

   /**
    * Throw exception if {@code value1} not epsilon equal to {@code value2}.
    * 
    * @param value1 value to check
    * @param value2 value to check
    * @param epsilon epsilon
    * @throws RuntimeException if {@code value1} not epsilon equal to {@code value2}.
    */
   public static void checkEpsilonEquals(double value1, double value2, double epsilon)
   {
      if (!epsilonEquals(value1, value2, epsilon))
      {
         throw new RuntimeException("Not epsilon equal.  " + value1 + " != " + value2 + " +/- " + epsilon);
      }
   }

   /**
    * Throw exception if {@code value1} not equal to {@code value2}.
    * 
    * @param value1 value to check
    * @param value2 value to check
    * @throws RuntimeException if {@code value1} not equal to {@code value2}.
    */
   public static void checkEquals(int value1, int value2)
   {
      if (value1 != value2)
      {
         throw new RuntimeException("Not equal. " + value1 + " !=" + value2);
      }
   }

   /**
    * Squares value.
    * 
    * @param value value to be squared
    * @return {@code value * value}
    */
   public static double square(double value)
   {
      return value * value;
   }

   /**
    * Cubes value.
    * 
    * @param value value to be cubed
    * @return {@code value * value * value}
    */
   public static double cube(double value)
   {
      return value * value * value;
   }

   /**
    * Calculates value to the power of exponent. Uses integer for efficiency.
    * 
    * @param value
    * @param exponent
    * @return Value to the power of exponent.
    */
   public static double pow(double value, int exponent)
   {
      double pow = 1.0;
      if (exponent >= 0)
      {
         for (int i = 0; i < exponent; i++)
         {
            pow *= value;
         }
      }
      else
      {
         for (int i = 0; i > exponent; i--)
         {
            pow /= value;
         }
      }
      return pow;
   }

   /**
    * Compares
    * 
    * @param numberOne
    * @param numberTwo
    * @param significantFigures
    * @return
    */
   public static boolean isGreaterThanWithSignificantFigures(double numberOne, double numberTwo, int significantFigures)
   {
      return roundToSignificantFigures(numberOne, significantFigures) > roundToSignificantFigures(numberTwo, significantFigures);
   }
   
   public static boolean isGreaterThanWithPrecision(double numberOne, double numberTwo, double precision)
   {
      return roundToPrecision(numberOne, precision) > roundToPrecision(numberTwo, precision);
   }
   
   public static boolean isGreaterThanOrEqualToWithSignificantFigures(double numberOne, double numberTwo, int significantFigures)
   {
      return roundToSignificantFigures(numberOne, significantFigures) >= roundToSignificantFigures(numberTwo, significantFigures);
   }
   
   public static boolean isGreaterThanOrEqualToWithPrecision(double numberOne, double numberTwo, double precision)
   {
      return roundToPrecision(numberOne, precision) >= roundToPrecision(numberTwo, precision);
   }

   public static boolean isLessThanWithSignificantFigures(double numberOne, double numberTwo, int significantFigures)
   {
      return roundToSignificantFigures(numberOne, significantFigures) < roundToSignificantFigures(numberTwo, significantFigures);
   }
   
   public static boolean isLessThanWithPrecision(double numberOne, double numberTwo, double precision)
   {
      return roundToPrecision(numberOne, precision) < roundToPrecision(numberTwo, precision);
   }
   
   public static boolean isLessThanOrEqualToWithSignificantFigures(double numberOne, double numberTwo, int significantFigures)
   {
      return roundToSignificantFigures(numberOne, significantFigures) <= roundToSignificantFigures(numberTwo, significantFigures);
   }
   
   public static boolean isLessThanOrEqualToWithPrecision(double numberOne, double numberTwo, double precision)
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
