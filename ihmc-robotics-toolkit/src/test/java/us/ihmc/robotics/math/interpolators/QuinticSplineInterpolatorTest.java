package us.ihmc.robotics.math.interpolators;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import java.io.IOException;
import java.util.Random;

import org.junit.jupiter.api.Test;

public class QuinticSplineInterpolatorTest
{

   
   @Test
   public void testUnevenlySpacedTime()
   {
      double[] x = { 0, 0.1, 2.0, 4.1 };
      double[] y = { 0.5, 0.1, 0.3, 0.5 };
      
      double v0 = 0.1;
      double vf = 0.0;
      double a0 = 0.0;
      double af = 0.0;
      
      QuinticSplineInterpolator spline = new QuinticSplineInterpolator("quinticSplineTest", 255, 1, null);
      
      spline.initialize(x.length, x);
      spline.determineCoefficients(0, y, v0, vf, a0, af);
      
      spline.compute(0.0);
      
      assertEquals(y[0], spline.getPosition(0), 1e-6);
      assertEquals(v0, spline.getVelocity(0), 1e-6);
      assertEquals(a0, spline.getAcceleration(0), 1e-6);
   }
   
   
   /**
    * Tests QuinticSplineInterpolator, makes four identical splines and checks if the values correspond
    * to the values I got from the spline interpolation I made in Matlab.
    * 
    * Tests 2D and 5D splines, to test if the resizing works
    */
   @Test
   public void testQuinticSplineInterpolator()
   {
      double[] x = new double[] {0, 3.333, 6, 10.0, 12.0};
      double[] y = new double[] {0, -0.7672, -0.6137, -0.3108, 1.0};

      double v0 = 0.22;
      double vf = -0.3;
      double a0 = 0.14;
      double af = -0.6;

      QuinticSplineInterpolator spline = new QuinticSplineInterpolator("quinticSplineTest", 255, 4, null);
      
      
      
      for(int t = 0; t < 5; t++)
      {
         double[] x2d = new double[] {0.0, 1.0, 0.0, 0.0};
         double[] y2d = new double[] {-0.5, 0.5, 0.0, 0.0};
         double v0_2d = 3.0, vf_2d = -2.0;
         double a0_2d = 10.0, af_2d = -4.0;
   
         spline.initialize(2, x2d);
         spline.determineCoefficients(0, y2d, v0_2d, vf_2d, a0_2d, af_2d);
         spline.determineCoefficients(1, y2d, v0_2d, vf_2d, a0_2d, af_2d);
         spline.determineCoefficients(2, y2d, v0_2d, vf_2d, a0_2d, af_2d);
         spline.determineCoefficients(3, y2d, v0_2d, vf_2d, a0_2d, af_2d);
   
         double delta = 1e-4;
   
         spline.compute(0.0);
         assertEquals(y2d[0], spline.getPosition(0), delta);
         assertEquals(v0_2d, spline.getVelocity(0), delta);
         assertEquals(a0_2d, spline.getAcceleration(0), delta);
   
         spline.compute(1.0);
         assertEquals(y2d[1], spline.getPosition(0), delta);
         assertEquals(vf_2d, spline.getVelocity(0), delta);
         assertEquals(af_2d, spline.getAcceleration(0), delta);
   
         
         spline.initialize(x.length, x);
         spline.determineCoefficients(0, y, v0, vf, a0, af);
         spline.determineCoefficients(1, y, v0, vf, a0, af);
         spline.determineCoefficients(2, y, v0, vf, a0, af);
         spline.determineCoefficients(3, y, v0, vf, a0, af);
   
         for(int i = 0; i < 4; i++)
         {
            spline.compute(12.0);
            assertEquals("Valued at x=12.0 is wrong.", vf, spline.getVelocity(i), 1e-4);
            assertEquals("Valuedd at x=12.0 is wrong.", af, spline.getAcceleration(i), 1e-4);
      
            spline.compute(0.0);
            assertEquals("Valued at x=0 is wrong.", v0, spline.getVelocity(i), 1e-4);
            assertEquals("Valuedd at x=0 is wrong.", a0, spline.getAcceleration(i), 1e-4);
      
            spline.compute(0.0);
            assertEquals("Value at x=0 is wrong.", 0.0, spline.getPosition(i), 1e-4);
      
            spline.compute(1.667);
            assertEquals("Value at x=1.667 is wrong.", -0.0485, spline.getPosition(i), 1e-4);
      
            spline.compute(5.0);
            assertEquals("Value at x=5.0 is wrong.", -0.6613, spline.getPosition(i), 1e-4);
      
            spline.compute(9.009);
            assertEquals("Valuedd at x=9.009 is wrong.", 0.8365, spline.getAcceleration(i), 1e-4);
      
            spline.compute(8.333);
            assertEquals("Value at x=8.333 is wrong.", -1.3500, spline.getPosition(i), 1e-4);
      
            spline.compute(10.0);
            assertEquals("Value at x=10.0 is wrong.", -0.3108, spline.getPosition(i), 1e-4);
      
            spline.compute(12.0);
            assertEquals("Value at x=12.0 is wrong.", 1.0, spline.getPosition(i), 1e-4);
         }
      }
   }

   /*
    * Tests the speed of the the QuinticSplineInterpolator. Makes numberOfSplines loops, each
    * generating a set of 6 splines with the same timing parameters.
    */

   @Test
   public void testQuinticSplineCalculationSpeed() throws IOException
   {

      final int numberOfSplines = 100000;

      double[][] x = new double[numberOfSplines][4];
      double[][][] y = new double[numberOfSplines][6][4];

      double v0[][] = new double[numberOfSplines][6];
      double vf[][] = new double[numberOfSplines][6];
      double a0[][] = new double[numberOfSplines][6];
      double af[][] = new double[numberOfSplines][6];

      QuinticSplineInterpolator spline = new QuinticSplineInterpolator("quinticSplineSpeedTest", 4, 6, null);

      // Generate random setpoints for the spline
      Random random = new Random(842631632);
      for (int i = 0; i < numberOfSplines; i++)
      {
         x[i][0] = random.nextDouble();
         for (int j = 1; j < 4; j++)
            x[i][j] = x[i][j - 1] + random.nextDouble();

         for (int j = 0; j < 6; j++)
         {
            for (int k = 0; k < 4; k++)
            {
               y[i][j][k] = random.nextDouble();
            }
            v0[i][j] = random.nextDouble();
            vf[i][j] = random.nextDouble();
            a0[i][j] = random.nextDouble();
            af[i][j] = random.nextDouble();

         }
      }

      double sum = 0;
      // Calculate a thousand splines and time them
      long startTime = System.nanoTime();
      for (int i = 0; i < numberOfSplines; i++)
      {
         spline.initialize(x[i].length, x[i]);
         for (int j = 0; j < 6; j++)
         {
            spline.determineCoefficients(j, y[i][j], v0[i][j], vf[i][j], a0[i][j], af[i][j]);
         }

         spline.compute(random.nextDouble() * x[i][3]);

         for (int j = 0; j < 6; j++)
         {
            sum += spline.getPosition(j) + spline.getVelocity(j) + spline.getAcceleration(j) + spline.getJerk(j);
         }
      }
      long endTime = System.nanoTime();

      double runTime = ((double) (endTime - startTime)) * 1e-9;
      System.out.println("Calculated " + numberOfSplines + " * 6 splines with random inputs in " + runTime + " seconds, generating a sum of " + sum);

      assertTrue("Spline test took to much time", runTime < ((double) numberOfSplines) / 1e4);
      assertTrue("Hash is not large enough", sum > 1e3);

   }


}
