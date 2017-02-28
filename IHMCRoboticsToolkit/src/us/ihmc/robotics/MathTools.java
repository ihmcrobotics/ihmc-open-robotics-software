package us.ihmc.robotics;

import java.util.ArrayList;
import java.util.Collection;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;

public class MathTools
{
   private MathTools()
   {
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
    *
    * @param array ArrayList
    * @return ArrayList
    */
   public static ArrayList<FrameVector> diff(ArrayList<FrameVector> array)
   {
      ArrayList<FrameVector> ret = new ArrayList<FrameVector>();
      for (int i = 1; i < array.size(); i++)
      {
         FrameVector diffedVector = new FrameVector(array.get(i));
         diffedVector.sub(array.get(i - 1));
         ret.add(diffedVector);
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

   public static double clipToMinMax(double val, double minMax)
   {
      return clipToMinMax(val, -minMax, minMax);
   }
   
   public static double clipToMinMax(float val, float minMax)
   {
      return clipToMinMax(val, -minMax, minMax);
   }

   /**
    * Returns max if max greater than given value
    * Returns min if min less than given value
    * Returns value if value is between max and min
    *
    *
    * @param val double
    * @param min double
    * @param max double
    * @return double
    */
   public static double clipToMinMax(double val, double min, double max)
   {
      if (min > (max + 1e-10))
      {
         throw new RuntimeException("tried to cap a value " + val + " between a min of " + min + " and a max of " + max
                                    + ". The max value is less than or equal to the min value");
      }

      return (Math.min(max, Math.max(val, min)));
   }
   
   public static int clipToMinMax(int val, int min, int max)
   {
      if (min > max)
      {
         throw new RuntimeException("tried to cap a value " + val + " between a min of " + min + " and a max of " + max
               + ". The max value is less than or equal to the min value");
      }
      
      return (Math.min(max, Math.max(val, min)));
   }

   public static float clipToMinMax(float val, float min, float max)
   {
      if (min > (max + 1e-10))
      {
         throw new RuntimeException("tried to cap a value " + val + " between a min of " + min + " and a max of " + max
                                    + ". The max value is less than or equal to the min value");
      }

      return (Math.min(max, Math.max(val, min)));
   }


   /**
    * Checks to see if val is Inside Bounds of max and min
    *
    * @param val double
    * @param min double
    * @param max double
    * @return boolean
    */
   public static boolean isInsideBoundsExclusive(double val, double min, double max)
   {
      if (min >= max)
      {
         throw new RuntimeException("tried to check bounds on a value " + val + " between a min of " + min + " and a max of " + max
                                    + ". The max value is less than the min value");
      }

      return ((val > min) && (val < max));
   }

   public static boolean isInsideBoundsInclusive(double val, double max)
   {
      return isInsideBoundsInclusive(val, -max, max);
   }

   public static boolean isInsideBoundsExclusive(double val, double max)
   {
      return isInsideBoundsExclusive(val, -max, max);
   }

   public static boolean isInsideBoundsInclusive(double val, double min, double max)
   {
      if (min > max)
      {
         throw new RuntimeException("tried to check bounds on a value " + val + " between a min of " + min + " and a max of " + max
                                    + ". The max value is less than the min value");
      }

      return ((val >= min) && (val <= max));
   }

   public static boolean isInsideBoundsInclusive(long val, long min, long max)
   {
      if (min > max)
      {
         throw new RuntimeException("tried to check bounds on a value " + val + " between a min of " + min + " and a max of " + max
                                    + ". The max value is less than the min value");
      }

      return ((val >= min) && (val <= max));
   }

   /**
    * Sums the integers in a collection
    *
    * @param integers collection of integers
    * @return int the sum
    */
   public static int sumIntegers(Collection<Integer> integers)
   {
      int ret = 0;
      for (int i : integers)
      {
         ret += i;
      }

      return ret;
   }

   /**
    * Sums the integers in an array
    *
    * @param integers array of integers
    * @return int the sum
    */
   public static int sumIntegers(int[] integers)
   {
      int ret = 0;
      for (int i : integers)
      {
         ret += i;
      }

      return ret;
   }

   /**
    * Sums the doubles in a collection
    *
    * @param doubles collection of doubles
    * @return double the sum
    */
   public static double sumDoubles(Collection<Double> doubles)
   {
      double ret = 0.0;
      for (double d : doubles)
      {
         ret += d;
      }

      return ret;
   }

   /**
    * Finds the sum of doubles in an array
    *
    * @param doubles double[]
    * return double
    */
   public static double sumDoubles(double[] doubles)
   {
      double ret = 0.0;
      for (double d : doubles)
      {
         ret += d;
      }

      return ret;
   }

   /**
    * Computes the cumulative sum array for a given array of doubles.
    * For example, the cumulative sum sequence of the input sequence {a, b, c, ...} is {a, a + b, a + b + c, ...}
    * @param doubles input sequence
    * @return cumulative sum sequence
    */
   public static double[] cumulativeSumDoubles(double[] doubles)
   {
      double[] ret = new double[doubles.length];
      double sum = 0.0;
      for (int i = 0; i < doubles.length; i++)
      {
         sum += doubles[i];
         ret[i] = sum;
      }

      return ret;
   }

   /**
    * Finds and returns the min value in an array of doubles
    *
    * @param array double[]
    * @return double
    */
   public static double min(double[] array)
   {
      double ret = Double.MAX_VALUE;
      for (double d : array)
      {
         ret = Math.min(ret, d);
      }

      return ret;
   }

   /**
    * Finds and returns the max value in an array of Doubles
    *
    * @param array double[]
    * @return double
    */
   public static double max(double[] array)
   {
      double ret = -Double.MAX_VALUE;
      for (double d : array)
      {
         ret = Math.max(ret, d);
      }

      return ret;
   }

   /**
    *
    *
    * @param array double[]
    * @return double
    */
   public static double mean(double[] array)
   {
      double tot = 0.0;
      for (double d : array)
      {
         tot += d;
      }

      double ret = tot / (array.length);

      // System.err.println("MathTools::mean: tot : " + tot);
      // System.err.println("MathTools::mean: array.length : " +
      // array.length);
      // System.err.println("MathTools::mean: ret : " + ret);
      return ret;
   }

   /**
    *
    *
    * @param array ArrayList<Double>
    * @return double
    */
   public static double mean(ArrayList<Double> array)
   {
      double tot = 0.0;
      for (double d : array)
      {
         tot += d;
      }

      double ret = tot / (array.size());

      return ret;
   }

   public static void checkIfInRange(double argument, double min, double max)
   {
      if (!isInsideBoundsInclusive(argument, min, max))
      {
         throw new RuntimeException("Argument " + argument + " not in range [" + min + ", " + max + "].");
      }
   }

   public static void checkIfInRange(long argument, long min, long max)
   {
      if (!isInsideBoundsInclusive(argument, min, max))
      {
         throw new RuntimeException("Argument " + argument + " not in range [" + min + ", " + max + "].");
      }
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
   
   public static boolean isPreciselyBoundedByInclusive(double boundaryOne, double boundaryTwo, double number, double precision)
   {
      return isBoundedByInclusive(roundToPrecision(boundaryOne, precision), roundToPrecision(boundaryTwo, precision), roundToPrecision(number, precision));
   }
   
   public static boolean isPreciselyBoundedByExclusive(double boundaryOne, double boundaryTwo, double number, double precision)
   {
      return isBoundedByExclusive(roundToPrecision(boundaryOne, precision), roundToPrecision(boundaryTwo, precision), roundToPrecision(number, precision));
   }
   
   public static boolean isBoundedByInclusive(double boundaryOne, double boundaryTwo, double number)
   {
      if (boundaryOne < boundaryTwo)
      {
         return number >= boundaryOne && number <= boundaryTwo;
      }
      else // (boundaryOne >= boundaryTwo)
      {
         return number >= boundaryTwo && number <= boundaryOne;
      }
   }
   
   public static boolean isBoundedByExclusive(double boundaryOne, double boundaryTwo, double number)
   {
      if (boundaryOne < boundaryTwo)
      {
         return number > boundaryOne && number < boundaryTwo;
      }
      else if (boundaryOne > boundaryTwo)
      {
         return number > boundaryTwo && number < boundaryOne;
      }
      else // (boundaryOne == boundaryTwo)
      {
         return false;
      }
   }

   public static void set(Tuple3DBasics tuple, Direction direction, double value)
   {
      switch (direction)
      {
         case X :
            tuple.setX(value);

            break;

         case Y :
            tuple.setY(value);

            break;

         case Z :
            tuple.setZ(value);

            break;

         default :
            throw new IndexOutOfBoundsException();
      }
   }

   public static double get(Tuple3DBasics tuple, Direction direction)
   {
      switch (direction)
      {
         case X :
            return tuple.getX();

         case Y :
            return tuple.getY();

         case Z :
            return tuple.getZ();

         default :
            throw new IndexOutOfBoundsException();
      }
   }

   public static boolean isFinite(double proposed)
   {
      boolean isNotInfinite = (!Double.isInfinite(proposed));
      boolean isNotInfiniteAndNotNaN = isNotInfinite & (!Double.isNaN(proposed));

      return isNotInfiniteAndNotNaN;
   }

   public static boolean isNumber(double proposed)
   {
      return (!Double.isNaN(proposed));
   }

   public static boolean isFinite(Tuple3DBasics tuple)
   {
      return isFinite(tuple.getX()) && isFinite(tuple.getY()) && isFinite(tuple.getZ());
   }
   
   public static boolean containsNaN(Tuple3DBasics tuple)
   {
      return (Double.isNaN(tuple.getX()) || Double.isNaN(tuple.getY()) || Double.isNaN(tuple.getZ()));
   }

   public static boolean containsNaN(Quaternion quat4d)
   {
      return (Double.isNaN(quat4d.getS()) || Double.isNaN(quat4d.getX()) || Double.isNaN(quat4d.getY()) || Double.isNaN(quat4d.getZ()));
   }

   public static boolean containsNaN(DenseMatrix64F denseMatrix64F)
   {
      int numberOfRows = denseMatrix64F.getNumRows();
      int numberOfColumns = denseMatrix64F.getNumCols();
      
      for (int row = 0; row < numberOfRows; row++)
      {
         for (int column = 0; column < numberOfColumns; column++)
         {
            if (Double.isNaN(denseMatrix64F.get(row, column))) return true;
         }
      }

      return false;
   }

   public static boolean containsNaN(SpatialMotionVector spatialMotionVector)
   {
      if (Double.isNaN(spatialMotionVector.getLinearPartX())) return true;
      if (Double.isNaN(spatialMotionVector.getLinearPartY())) return true;
      if (Double.isNaN(spatialMotionVector.getLinearPartZ())) return true;
      
      if (Double.isNaN(spatialMotionVector.getAngularPartX())) return true;
      if (Double.isNaN(spatialMotionVector.getAngularPartY())) return true;
      if (Double.isNaN(spatialMotionVector.getAngularPartZ())) return true;

      return false;
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

   public static double floorToGivenPrecision(double value, double precisionFactor)
   {
      long longValue = (long) (value / precisionFactor); 
      double roundedValue = ((double) longValue) * precisionFactor;
      return roundedValue;
   }

   public static void floorToGivenPrecision(Tuple3DBasics tuple3d, double precision)
   {
      tuple3d.setX(floorToGivenPrecision(tuple3d.getX(), precision));
      tuple3d.setY(floorToGivenPrecision(tuple3d.getY(), precision));
      tuple3d.setZ(floorToGivenPrecision(tuple3d.getZ(), precision));
      
   }

   public static double roundToPrecision(double value, double precision)
   {
      double adjustmentFactor = (value > 0.0) ? 0.5 * precision : -0.5 * precision;
      long longValue = (long) ((value + adjustmentFactor) / precision);
      double roundedValue = ((double) longValue) * precision;
      return roundedValue;
   }

   public static void roundToGivenPrecision(Tuple3DBasics tuple3d, double precision)
   {
      tuple3d.setX(roundToPrecision(tuple3d.getX(), precision));
      tuple3d.setY(roundToPrecision(tuple3d.getY(), precision));
      tuple3d.setZ(roundToPrecision(tuple3d.getZ(), precision));
   }

   public static double roundToGivenPrecisionForAngle(double angleValue, double precisionFactor)
   {
      double centeredAngleValue = AngleTools.trimAngleMinusPiToPi(angleValue + 0.5 * precisionFactor);
      long longValue = (long) (centeredAngleValue / precisionFactor);
      double roundedValue = ((double) longValue) * precisionFactor;
      return AngleTools.trimAngleMinusPiToPi(roundedValue);
   }

   public static void roundToGivenPrecisionForAngles(Tuple3DBasics tuple3d, double precision)
   {
      tuple3d.setX(roundToGivenPrecisionForAngle(tuple3d.getX(), precision));
      tuple3d.setY(roundToGivenPrecisionForAngle(tuple3d.getY(), precision));
      tuple3d.setZ(roundToGivenPrecisionForAngle(tuple3d.getZ(), precision));
   }
   
   public static double roundToSignificantFigures(double number, int significantFigures)
   {
      if (Math.abs(number) < Double.MIN_VALUE)
      {
         return 0.0;
      }
   
      final double log10 = Math.ceil(Math.log10(Math.abs(number)));
      final int power = significantFigures - (int) log10;
   
      final double magnitude = Math.pow(10, power);
      final long shifted = Math.round(number * magnitude);
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
