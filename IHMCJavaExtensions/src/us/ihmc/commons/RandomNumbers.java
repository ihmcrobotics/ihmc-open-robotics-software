package us.ihmc.commons;

import java.util.Random;

public class RandomNumbers
{
   public static boolean nextBoolean(Random random, double percentChanceTrue)
   {
      double percent = random.nextDouble();
      if (percent < percentChanceTrue)
         return true;
      return false;
   }

   public static int nextInt(Random random, int boundaryOneInclusive, int boundaryTwoInclusive)
   {
      return boundaryOneInclusive + random.nextInt(boundaryTwoInclusive - boundaryOneInclusive + 1);
   }

   public static float nextFloat(Random random, float range1, float range2)
   {
      return range1 + (range2 - range1) * random.nextFloat();
   }

   public static double nextDouble(Random random, double maxAbsolute)
   {
      return nextDouble(random, -maxAbsolute, maxAbsolute);
   }

   public static double nextDouble(Random random, double boundaryOne, double boundaryTwo)
   {
      return boundaryOne + random.nextDouble() * (boundaryTwo - boundaryOne);
   }

   public static <T extends Enum<T>> T nextEnum(Random random, Class<T> enumType)
   {
      int numberOfEnums = enumType.getEnumConstants().length;
      return enumType.getEnumConstants()[random.nextInt(numberOfEnums)];
   }

   public static double[] nextDoubleArray(Random random, int length, double amplitude)
   {
      return nextDoubleArray(random, length, -amplitude / 2.0, amplitude / 2.0);
   }

   public static double[] nextDoubleArray(Random random, int length, double lowerBound, double upperBound)
   {
      double[] ret = new double[length];
      for (int i = 0; i < length; i++)
      {
         double parameter = random.nextDouble();
         ret[i] = parameter * lowerBound + (1.0 - parameter) * upperBound;
      }
   
      return ret;
   }

   public static float[] nextFloatArray(Random random, int length, float amplitude)
   {
      return nextFloatArray(random, length, -amplitude / 2.0f, amplitude / 2.0f);
   }

   public static float[] nextFloatArray(Random random, int length, float lowerBound, float upperBound)
   {
      float[] ret = new float[length];
      for (int i = 0; i < length; i++)
      {
         float parameter = random.nextFloat();
         ret[i] = parameter * lowerBound + (1.0f - parameter) * upperBound;
      }
   
      return ret;
   }

   public static int[] nextIntArray(Random random, int length, int amplitude)
   {
      int[] ret = new int[length];
      for (int i = 0; i < length; i++)
      {
         ret[i] = (int) Math.round((random.nextDouble() - 0.5) * 2.0 * amplitude);
      }
   
      return ret;
   }

   public static int[] nextIntArray(Random random, int length, int lowerBound, int upperBound)
   {
      int[] ret = new int[length];
      for (int i = 0; i < length; i++)
      {
         ret[i] = (int) Math.round(random.nextDouble() * (upperBound - lowerBound) + lowerBound);
      }
   
      return ret;
   }

   public static int nextIntWithEdgeCases(Random random, double probabilityForEdgeCase)
   {
      int totalNumberOfInts = (int) (4 / probabilityForEdgeCase);
   
      int randInt = random.nextInt(totalNumberOfInts);
      switch (randInt)
      {
      case 0:
         return Integer.MIN_VALUE;
      case 1:
         return Integer.MAX_VALUE;
      case 2:
         return -Integer.MIN_VALUE;
      case 3:
         return -Integer.MAX_VALUE;
      default:
         return random.nextInt();
      }
   }

   public static double nextDoubleWithEdgeCases(Random random, double probabilityForEdgeCase, double maxValue)
   {
      int totalNumberOfInts = (int) (7 / probabilityForEdgeCase);
   
      int randInt = random.nextInt(totalNumberOfInts);
      switch (randInt)
      {
      case 0:
         return Double.NaN;
      case 1:
         return Double.MIN_VALUE;
      case 2:
         return Double.MAX_VALUE;
      case 3:
         return -Double.MIN_VALUE;
      case 4:
         return -Double.MAX_VALUE;
      case 5:
         return Double.NEGATIVE_INFINITY;
      case 6:
         return Double.POSITIVE_INFINITY;
      default:
         return (random.nextDouble() * 2 - 1) * maxValue;
      }
   }

   public static double[] nextDoubleArrayWithEdgeCases(Random random, int length, double probabilityForEdgeCase)
   {
      double[] ret = new double[length];
      for (int i = 0; i < length; i++)
      {
         ret[i] = nextDoubleWithEdgeCases(random, probabilityForEdgeCase);
      }
   
      return ret;
   }

   public static double nextDoubleWithEdgeCases(Random random, double probabilityForEdgeCase)
   {
      return nextDoubleWithEdgeCases(random, probabilityForEdgeCase, Double.MAX_VALUE);
   }
}
