package us.ihmc.manipulation.planning.gradientDescent;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.Conversions;

/**
 * Follow the benchmark for Math.Atan2(x, y);
 * Riven atan operation is using lookup-table.
 */
public class ArcTangentBenchmarkTest
{
   @Test
   public void compareMathAtan()
   {
      System.out.println("MathTools.");
      Random random = new Random(0612L);

      final int numberOfTest = (int) 10E7;

      long startTime = System.nanoTime();
      for (int i = 0; i < numberOfTest; i++)
      {
         double xRandom = random.nextDouble();
         double yRandom = random.nextDouble();

         Math.atan2(yRandom, xRandom);
      }

      long endTime = System.nanoTime() - startTime;
      double computingTime = Conversions.nanosecondsToSeconds(endTime);

      System.out.println("computingWithMathTools " + computingTime);
   }

   @Test
   public void compareRivenAtan()
   {
      System.out.println("Riven.");
      Random random = new Random(0612L);

      final int numberOfTest = (int) 10E7;

      long startTime = System.nanoTime();
      for (int i = 0; i < numberOfTest; i++)
      {
         double xRandom = random.nextDouble();
         double yRandom = random.nextDouble();

         Riven.atan2(yRandom, xRandom);
      }

      long endTime = System.nanoTime() - startTime;
      double computingTime = Conversions.nanosecondsToSeconds(endTime);

      System.out.println("computingWithMathTools " + computingTime);
   }

   @Test
   public void compareIceCore()
   {
      System.out.println("IceCore.");
      Random random = new Random(0612L);

      final int numberOfTest = (int) 10E7;

      long startTime = System.nanoTime();
      for (int i = 0; i < numberOfTest; i++)
      {
         double xRandom = random.nextDouble();
         double yRandom = random.nextDouble();

         Icecore.atan2(yRandom, xRandom);
      }

      long endTime = System.nanoTime() - startTime;
      double computingTime = Conversions.nanosecondsToSeconds(endTime);

      System.out.println("computingWithMathTools " + computingTime);
   }

   @Test
   public void compareMathAtan2VersusRivenAtan2()
   {
      System.out.println("MathTools vs Riven.");
      Random random = new Random(0612L);

      final int numberOfTest = (int) 10E6;

      long computingWithMathTools = 0;
      long computingWithRiven = 0;
      double totalErrorRatio = 0;

      long startTime = System.nanoTime();

      for (int i = 0; i < numberOfTest; i++)
      {
         double xRandom = random.nextDouble();
         double yRandom = random.nextDouble();

         long startMathTools = System.nanoTime();
         double atanMathTools = Math.atan2(yRandom, xRandom);
         long timeMathTools = System.nanoTime() - startMathTools;
         computingWithMathTools += timeMathTools;

         long startRiven = System.nanoTime();
         double atanRiven = Riven.atan2(yRandom, xRandom);
         long timeRiven = System.nanoTime() - startRiven;
         computingWithRiven += timeRiven;

         double error = atanRiven - atanMathTools;
         double errorRatio = error / atanMathTools;
         totalErrorRatio += errorRatio;
         assertTrue("Riven solution has big error " + errorRatio, errorRatio < 10E-3);
      }

      double computingTimeWithMathTools = Conversions.nanosecondsToSeconds(computingWithMathTools);
      double computingTimeWithRiven = Conversions.nanosecondsToSeconds(computingWithRiven);

      System.out.println("computingWithMathTools " + computingTimeWithMathTools);
      System.out.println("computingWithRiven     " + computingTimeWithRiven);
      System.out.println("mean error (%)         " + Math.abs(totalErrorRatio / numberOfTest) * 100);
      System.out.println("total test time is     " + Conversions.nanosecondsToSeconds(System.nanoTime() - startTime));
      assertTrue("Java MathTool is faster than Riven. ", computingTimeWithMathTools > computingTimeWithRiven);
   }

   public static final class Riven
   {
      /**
       * @param bitsPerDimension is an accuracy parameter (ATAN2_BITS).
       * The default value is 7.
       * If we use 7 bits per dimension, that means 2^7 (128) values for each dimension.
       * Consequently, 128 * 128 entries are in lookup-table.
       * The accuracy might be around (1/128) in general.
       */
      private static final int ATAN2_BITS = 9;

      private static final int ATAN2_BITS2 = ATAN2_BITS << 1;
      private static final int ATAN2_MASK = ~(-1 << ATAN2_BITS2);
      private static final int ATAN2_COUNT = ATAN2_MASK + 1;
      private static final int ATAN2_DIM = (int) Math.sqrt(ATAN2_COUNT);

      private static final float INV_ATAN2_DIM_MINUS_1 = 1.0f / (ATAN2_DIM - 1);

      private static final float[] atan2 = new float[ATAN2_COUNT];

      static
      {
         for (int i = 0; i < ATAN2_DIM; i++)
         {
            for (int j = 0; j < ATAN2_DIM; j++)
            {
               float x0 = (float) i / ATAN2_DIM;
               float y0 = (float) j / ATAN2_DIM;

               atan2[j * ATAN2_DIM + i] = (float) Math.atan2(y0, x0);
            }
         }
      }

      public static final double atan2(double y, double x)
      {
         double add, mul;

         if (x < 0.0f)
         {
            if (y < 0.0f)
            {
               x = -x;
               y = -y;

               mul = 1.0f;
            }
            else
            {
               x = -x;
               mul = -1.0f;
            }
            add = -3.141592653f;
            //add = -Math.PI;
         }
         else
         {
            if (y < 0.0f)
            {
               y = -y;
               mul = -1.0f;
            }
            else
            {
               mul = 1.0f;
            }
            add = 0.0f;
         }

         double invDiv = 1.0f / (((x < y) ? y : x) * INV_ATAN2_DIM_MINUS_1);

         int xi = (int) (x * invDiv);
         int yi = (int) (y * invDiv);

         return (atan2[yi * ATAN2_DIM + xi] + add) * mul;
      }
   }

   public static final class Icecore
   {
      private static final int Size_Ac = 100000;
      private static final int Size_Ar = Size_Ac + 1;
      private static final float Pi = (float) Math.PI;
      private static final float Pi_H = Pi / 2;

      private static final double Atan2[] = new double[Size_Ar];
      private static final double Atan2_PM[] = new double[Size_Ar];
      private static final double Atan2_MP[] = new double[Size_Ar];
      private static final double Atan2_MM[] = new double[Size_Ar];

      private static final double Atan2_R[] = new double[Size_Ar];
      private static final double Atan2_RPM[] = new double[Size_Ar];
      private static final double Atan2_RMP[] = new double[Size_Ar];
      private static final double Atan2_RMM[] = new double[Size_Ar];

      static
      {
         for (int i = 0; i <= Size_Ac; i++)
         {
            double d = (double) i / Size_Ac;
            double x = 1;
            double y = x * d;
            double v = Math.atan2(y, x);
            Atan2[i] = v;
            Atan2_PM[i] = Pi - v;
            Atan2_MP[i] = -v;
            Atan2_MM[i] = -Pi + v;

            Atan2_R[i] = Pi_H - v;
            Atan2_RPM[i] = Pi_H + v;
            Atan2_RMP[i] = -Pi_H + v;
            Atan2_RMM[i] = -Pi_H - v;
         }
      }

      public static final double atan2(double y, double x)
      {
         if (y < 0)
         {
            if (x < 0)
            {
               if (y < x)
               {
                  return Atan2_RMM[(int) (x / y * Size_Ac)];
               }
               else
               {
                  return Atan2_MM[(int) (y / x * Size_Ac)];
               }
            }
            else
            {
               y = -y;
               if (y > x)
               {
                  return Atan2_RMP[(int) (x / y * Size_Ac)];
               }
               else
               {
                  return Atan2_MP[(int) (y / x * Size_Ac)];
               }
            }
         }
         else
         {
            if (x < 0)
            {
               x = -x;
               if (y > x)
               {
                  return Atan2_RPM[(int) (x / y * Size_Ac)];
               }
               else
               {
                  return Atan2_PM[(int) (y / x * Size_Ac)];
               }
            }
            else
            {
               if (y > x)
               {
                  return Atan2_R[(int) (x / y * Size_Ac)];
               }
               else
               {
                  return Atan2[(int) (y / x * Size_Ac)];
               }
            }
         }
      }
   }
}
