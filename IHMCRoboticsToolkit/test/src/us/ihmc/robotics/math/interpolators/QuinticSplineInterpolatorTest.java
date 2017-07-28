package us.ihmc.robotics.math.interpolators;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class QuinticSplineInterpolatorTest
{

   /*
    * Tests QuinticSplineInterpolator, makes four identical splines and checks if
    * the values correspond to the values I got from the spline interpolation I
    * made in Matlab.
    */

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testQuinticSplineInterpolator()
   {
      double[] x = new double[] { 0, 3.333, 6, 10.0, 12.0 };
      double[] y = new double[] { 0, -0.7672, -0.6137, -0.3108, 1.0 };

      double v0 = 0.22;
      double vf = -0.3;
      double a0 = 0.14;
      double af = -0.6;

      QuinticSplineInterpolator spline = new QuinticSplineInterpolator("quinticSplineTest", 5, 4, null);
      spline.initialize(x);
      spline.determineCoefficients(0, y, v0, vf, a0, af);
      spline.determineCoefficients(1, y, v0, vf, a0, af);
      spline.determineCoefficients(2, y, v0, vf, a0, af);
      spline.determineCoefficients(3, y, v0, vf, a0, af);

      double ret[][] = new double[4][3];

      spline.compute(12.0, 2, ret);
      assertEquals("Valued at x=12.0 is wrong.", vf, ret[0][1], 1e-4);
      assertEquals("Valuedd at x=12.0 is wrong.", af, ret[1][2], 1e-4);

      spline.compute(0.0, 2, ret);
      assertEquals("Valued at x=0 is wrong.", v0, ret[0][1], 1e-4);
      assertEquals("Valuedd at x=0 is wrong.", a0, ret[0][2], 1e-4);

      spline.compute(0.0, 2, ret);
      assertEquals("Value at x=0 is wrong.", 0.0, ret[0][0], 1e-4);

      spline.compute(1.667, 2, ret);
      assertEquals("Value at x=1.667 is wrong.", -0.0485, ret[0][0], 1e-4);

      spline.compute(5.0, 2, ret);
      assertEquals("Value at x=5.0 is wrong.", -0.6613, ret[0][0], 1e-4);
      
      spline.compute(9.009, 2, ret);
      assertEquals("Valuedd at x=9.009 is wrong.", 0.8365, ret[0][2], 1e-4);

      spline.compute(8.333, 2, ret);
      assertEquals("Value at x=8.333 is wrong.", -1.3500, ret[3][0], 1e-4);

      spline.compute(10.0, 2, ret);
      assertEquals("Value at x=10.0 is wrong.", -0.3108, ret[0][0], 1e-4);
      
      spline.compute(12.0, 2, ret);
      assertEquals("Value at x=12.0 is wrong.", 1.0, ret[0][0], 1e-4);

   }

   /*
    * Tests the speed of the the QuinticSplineInterpolator. Makes
    * numberOfSplines loops, each generating a set of 6 splines with the same
    * timing parameters.
    */

	@ContinuousIntegrationTest(estimatedDuration = 1.0)
	@Test(timeout=300000)
   public void testQuinticSplineCalculationSpeed() throws IOException
   {

      final int numberOfSplines = 100000;

      double[][] x = new double[numberOfSplines][4];
      double[][][] y = new double[numberOfSplines][6][4];

      double v0[][] = new double[numberOfSplines][6];
      double vf[][] = new double[numberOfSplines][6];
      double a0[][] = new double[numberOfSplines][6];
      double af[][] = new double[numberOfSplines][6];

      double[][][] ret = new double[numberOfSplines][6][3];

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

      // Calculate a thousand splines and time them
      long startTime = System.nanoTime();
      for (int i = 0; i < numberOfSplines; i++)
      {
         spline.initialize(x[i]);
         for (int j = 0; j < 6; j++)
         {
            spline.determineCoefficients(j, y[i][j], v0[i][j], vf[i][j], a0[i][j], af[i][j]);
         }

         spline.compute(random.nextDouble() * x[i][3], 2, ret[i]);

      }
      long endTime = System.nanoTime();

      // Do something with ret to prevent the JIT skipping code
      double hash = 0;

      for (int i = 0; i < numberOfSplines; i++)
         for (int j = 0; j < 6; j++)
            for (int k = 0; k < 3; k++)
               hash += ret[i][j][k];

      double runTime = ((double) (endTime - startTime)) * 1e-9;
      System.out.println("Calculated " + numberOfSplines + " * 6 splines with random inputs in " + runTime + " seconds, generating a hash of " + hash);

      assertTrue("Spline test took to much time", runTime < ((double) numberOfSplines) / 1e4);
      assertTrue("Hash is not large enough", hash > 1e3);

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testTwoPointsQuinticSpline()
   {
      double[] x = new double[] { 0.0, 1.0 };
      double[] y = new double[] { -0.5, 0.5 };
      double v0 = 3.0, vf = -2.0;
      double a0 = 10.0, af = -4.0;
      
      QuinticSplineInterpolator spline = new QuinticSplineInterpolator("quinticSplineTestTwo", 2, 3, null);
      spline.initialize(x);
      spline.determineCoefficients(0, y, v0, vf, a0, af);
      spline.determineCoefficients(1, y, v0, vf, a0, af);
      spline.determineCoefficients(2, y, v0, vf, a0, af);
      
      
      double[][] ret = new double[3][3];
      double delta = 1e-4;
      
      spline.compute(0.0, 2, ret);
      assertEquals(y[0], ret[0][0], delta);
      assertEquals(v0, ret[0][1],delta);
      assertEquals(a0, ret[0][2],delta);
      
      spline.compute(1.0, 2, ret);
      assertEquals(y[1], ret[0][0], delta);
      assertEquals(vf, ret[0][1],delta);
      assertEquals(af, ret[0][2],delta);
      
   }

}
