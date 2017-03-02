package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

/**
 * Created by agrabertilton on 3/4/15.
 */
public class DoubleSupportICPEquationsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSingleSupportICPDerivatives()
   {
      Random random = new Random(8723L);
      double w = Math.abs(random.nextDouble()) + 0.1;
      double td = Math.abs(random.nextDouble()) + 0.1;
      double t0 = Math.abs(random.nextDouble());

      double PI = Math.abs(random.nextDouble());
      double PF = PI + 0.1 + Math.abs(random.nextDouble());

      double[] zmpCoefficients = new double[5];
      double lambda = random.nextDouble();

      double t = random.nextDouble();
      double icpDot = calculateICPDotValue(PI, PF, td, t0, zmpCoefficients, t, w, lambda);
      double icp = calculateICPValue(PI, PF, td, t0, zmpCoefficients, t, w, lambda);
      double zmp = calculateZMPValue(PI, PF, td, t0, zmpCoefficients, t);
      assertEquals(icpDot - w * icp, -w * zmp, 1e-14);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCoefficientCalculations()
   {
      Random random = new Random(8723L);
      double w = 1;
      double td = 0.5;
      double t0 = 1;
      double tF = t0 + td;

      double PI = 1;
      double PF = 2;

      double[] zmpCoefficients = {0, 0, 0, 10, -15, 6};
      double[] icpCoefficients = calculateICPCoefficients(zmpCoefficients, w, td);

      double icpF = PF;// + (PF-PI)*(0.1 + 0.9 * random.nextDouble());
      double lambda = calculateLambda(PI, PF, icpF, icpCoefficients, w, td);

      double t = tF;
      double icpDot = calculateICPDotValue(PI, PF, td, t0, icpCoefficients, t, w, lambda);
      double icp = calculateICPValue(PI, PF, td, t0, icpCoefficients, t, w, lambda);
      double zmp = calculateZMPValue(PI, PF, td, t0, zmpCoefficients, t);
      assertEquals(icpDot - w * icp, -w * zmp, 1e-10);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testZMPICPDerivatives()
   {
      Random random = new Random(8723L);
      double w = Math.abs(random.nextDouble()) + 0.1;
      double td = Math.abs(random.nextDouble()) + 0.1;
      double t0 = Math.abs(random.nextDouble());
      double tF = t0 + td;

      double PI = Math.abs(random.nextDouble());
      double PF = PI + 0.1 + Math.abs(random.nextDouble());

      double[] zmpCoefficients = createRandomZMPCoefficients(random, 5);
      double zmpt0 = calculateZMPValue(PI, PF, td, t0, zmpCoefficients, t0);
      assertEquals(zmpt0, PI, 1e-15);
      double zmptF = calculateZMPValue(PI, PF, td, t0, zmpCoefficients, tF);
      assertEquals(zmptF, PF, 1e-15);

      double[] icpCoefficients = calculateICPCoefficients(zmpCoefficients, w, td);
      double icpF = PI + (PF - PI) * (0.1 + 0.9 * random.nextDouble());
      double lambda = calculateLambda(PI, PF, icpF, icpCoefficients, w, td);

      double t = random.nextDouble();
      double icpDot = calculateICPDotValue(PI, PF, td, t0, icpCoefficients, t, w, lambda);
      double icp = calculateICPValue(PI, PF, td, t0, icpCoefficients, t, w, lambda);
      double zmp = calculateZMPValue(PI, PF, td, t0, zmpCoefficients, t);
      assertEquals(icpDot - w * icp, -w * zmp, 1e-10);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSingleCoefficientDetermination()
   {
      Random random = new Random(4575L);
      double w = Math.abs(random.nextDouble()) + 0.1;
      double td = Math.abs(random.nextDouble()) + 0.1;
      double t0 = Math.abs(random.nextDouble());
      double tF = t0 + td;

      double PI = Math.abs(random.nextDouble());
      double PF = PI + 0.1 + Math.abs(random.nextDouble());

      double[] zmpCoefficients = createRandomZMPCoefficients(random, 5);
      double zmpt0 = calculateZMPValue(PI, PF, td, t0, zmpCoefficients, t0);
      assertEquals(zmpt0, PI, 1e-15);
      double zmptF = calculateZMPValue(PI, PF, td, t0, zmpCoefficients, tF);
      assertEquals(zmptF, PF, 1e-15);

      double[] icpCoefficients = calculateICPCoefficients(zmpCoefficients, w, td);
      for (int i = 0; i < icpCoefficients.length; i++)
      {
         double coefficient = calculateICPCoefficient(i, zmpCoefficients, w, td);
         assertEquals(icpCoefficients[i], coefficient, 1e-10);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testICPSumFormula()
   {
      Random random = new Random(4575L);
      double w = Math.abs(random.nextDouble()) + 0.1;
      double td = Math.abs(random.nextDouble()) + 0.1;
      double t0 = Math.abs(random.nextDouble());
      double tF = t0 + td;

      double PI = Math.abs(random.nextDouble());
      double PF = PI + 0.1 + Math.abs(random.nextDouble());

      double[] zmpCoefficients = createRandomZMPCoefficients(random, 5);
      double zmpt0 = calculateZMPValue(PI, PF, td, t0, zmpCoefficients, t0);
      assertEquals(zmpt0, PI, 1e-15);
      double zmptF = calculateZMPValue(PI, PF, td, t0, zmpCoefficients, tF);
      assertEquals(zmptF, PF, 1e-15);

      double icpSum = 0;
      double[] icpCoefficients = calculateICPCoefficients(zmpCoefficients, w, td);
      for (int i = 0; i < icpCoefficients.length; i++)
      {
         icpSum += icpCoefficients[i];
      }
      double matrixCalculation = calculateICPCoefSum(zmpCoefficients, w, td);
      assertEquals(icpSum, matrixCalculation, 1e-10);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testICPICalculation()
   {
      Random random = new Random(8723L);
      double w = Math.abs(random.nextDouble()) + 0.1;
      double td = Math.abs(random.nextDouble()) + 0.1;
      double t0 = Math.abs(random.nextDouble());
      double tF = t0 + td;

      double PI = Math.abs(random.nextDouble());
      double PF = PI + 0.1 + Math.abs(random.nextDouble());

      double[] zmpCoefficients = createRandomZMPCoefficients(random, 5);
      double zmpt0 = calculateZMPValue(PI, PF, td, t0, zmpCoefficients, t0);
      assertEquals(zmpt0, PI, 1e-15);
      double zmptF = calculateZMPValue(PI, PF, td, t0, zmpCoefficients, tF);
      assertEquals(zmptF, PF, 1e-15);

      double[] icpCoefficients = calculateICPCoefficients(zmpCoefficients, w, td);
      double icpF = PI + (PF - PI) * (0.1 + 0.9 * random.nextDouble());
      double lambda = calculateLambda(PI, PF, icpF, icpCoefficients, w, td);

      double t = random.nextDouble();
      double icpDot = calculateICPDotValue(PI, PF, td, t0, icpCoefficients, t, w, lambda);
      double icp = calculateICPValue(PI, PF, td, t0, icpCoefficients, t, w, lambda);
      double zmp = calculateZMPValue(PI, PF, td, t0, zmpCoefficients, t);
      assertEquals(icpDot - w * icp, -w * zmp, 1e-10);

      //test finalICP
      t = tF;
      icp = calculateICPValue(PI, PF, td, t0, icpCoefficients, t, w, lambda);
      assertEquals(icpF, icp, 1e-10);

      //test initialICP
      double icpI = calculateInitialICP(PI, PF, zmpCoefficients, icpF, w, td);
      t = t0;
      icp = calculateICPValue(PI, PF, td, t0, icpCoefficients, t, w, lambda);
      assertEquals(icpI, icp, 1e-10);
   }

   private double[] createRandomZMPCoefficients(Random random, int size)
   {
      double[] zmpCoefficients = new double[size];
      double normalizationFactor = 0;
      for (int i = 0; i < zmpCoefficients.length; i++)
      {
         if (i == 0)
         {
            zmpCoefficients[i] = 0;
         }
         else
         {
            zmpCoefficients[i] = random.nextDouble();
         }
         normalizationFactor += zmpCoefficients[i];
      }

      for (int i = 0; i < zmpCoefficients.length; i++)
      {
         if (normalizationFactor == 0)
         {
            if (i == 0)
            {
               zmpCoefficients[i] = 0;
            }
            else
            {
               zmpCoefficients[i] = 1.0 / zmpCoefficients.length;
            }
         }
         else
         {
            zmpCoefficients[i] /= normalizationFactor;
         }
      }
      return zmpCoefficients;
   }

   private double calculateZMPValue(double PI, double PF, double td, double t0, double[] coefficients, double t)
   {
      //zmp = PI + (PF - PI)*Sum[i](Coef[i] * ((t - to)/td)^i)
      double zmp = PI;
      double timePolynomialValue = 0;
      for (int i = 0; i < coefficients.length; i++)
      {
         timePolynomialValue += coefficients[i] * Math.pow(((t - t0) / td), i);
      }
      zmp += (PF - PI) * timePolynomialValue;
      return zmp;
   }

   private double calculateICPValue(double PI, double PF, double td, double t0, double[] ICPcoefficients, double t, double w, double lambda)
   {
      //ICP = [lamda e^(w(t-t0)) + Sum[i](ICPCoef[i] * ((t - to)/td)^i)](PF-PI) + PI
      double icp = PI;
      double timePolynomialValue = 0;
      for (int i = 0; i < ICPcoefficients.length; i++)
      {
         timePolynomialValue += ICPcoefficients[i] * Math.pow(((t - t0) / td), i);
      }
      timePolynomialValue += lambda * Math.exp(w * (t - t0));
      icp += (PF - PI) * timePolynomialValue;
      return icp;
   }

   private double calculateICPDotValue(double PI, double PF, double td, double t0, double[] ICPcoefficients, double t, double w, double lambda)
   {
      //ICP = [w*lamda e^(w(t-t0)) + Sum[i](ICPCoef[i] * (i)((t - to)/td)^(i-1))](PF-PI)
      double icpDot = 0;
      double timePolynomialValue = 0;
      for (int i = 1; i < ICPcoefficients.length; i++)
      {
         timePolynomialValue += i * ICPcoefficients[i] * Math.pow(((t - t0) / td), (i - 1)) / td;
      }
      timePolynomialValue += w * lambda * Math.exp(w * (t - t0));
      icpDot += (PF - PI) * timePolynomialValue;
      return icpDot;
   }

   private double[] calculateICPCoefficients(double[] zmpCoefficients, double w, double td)
   {
      //calculate the coefficients for the icp
      double[] icpCoefficients = new double[zmpCoefficients.length];
      int n = zmpCoefficients.length - 1;
      for (int i = 0; i < zmpCoefficients.length; i++)
      {
         icpCoefficients[n - i] = zmpCoefficients[n - i];
         if (i != 0)
         {
            icpCoefficients[n - i] += (n - i + 1) * icpCoefficients[n - i + 1] / (w * td);
         }
      }
      return icpCoefficients;
   }

   private double calculateICPCoefficient(int i, double[] zmpCoefficients, double w, double td)
   {
      //calculate the coefficients for the icp
      int n = zmpCoefficients.length - 1;
      double coefficient = 0;
      for (int s = i; s <= n; s++)
      {
         double factorialTerm = factorialTerm(s, i);
         coefficient += zmpCoefficients[s] * factorialTerm / Math.pow(w * td, s - i);
      }
      return coefficient;
   }

   private double calculateLambda(double PI, double PF, double icpF, double[] icpCoefficients, double w, double td)
   {
      double icpCoeffSum = 0;
      for (int i = 0; i < icpCoefficients.length; i++)
      {
         icpCoeffSum += icpCoefficients[i];
      }
      double lambda = ((icpF - PI) / (PF - PI) - icpCoeffSum) * Math.exp(-w * td);
      return lambda;
   }

   private double calculateICPCoefSum(double[] zmpCoefficients, double w, double td)
   {
      double sum = 0;
      double[] multipliers = new double[zmpCoefficients.length];
      for (int i = 0; i < multipliers.length; i++)
      {
         for (int j = 0; j <= i; j++)
         {
            double factorialTerm = factorialTerm(i, i - j);
            multipliers[i] += factorialTerm / Math.pow(w * td, j);
         }
         sum += multipliers[i] * zmpCoefficients[i];
      }
      return sum;
   }

   private double calculateInitialICP(double PI, double PF, double[] zmpCoefficients, double ICPF, double w, double td)
   {
      double finalICPPercent = (ICPF - PI) / (PF - PI);
      double sumICPCoef = calculateICPCoefSum(zmpCoefficients, w, td);
      double icpC0 = calculateICPCoefficient(0, zmpCoefficients, w, td);
      double initialICPPercent = (finalICPPercent - sumICPCoef) * Math.exp(-w * td) + icpC0;
      double initialICP = PI + (PF - PI) * initialICPPercent;
      return initialICP;
   }

   /**
    *
    * @param upper
    * @param lower
    * @return upper!/lower!
    */
   private double factorialTerm(int upper, int lower)
   {
      boolean switched = false;
      if (upper < 0 || lower < 0)
      {
         throw new RuntimeException("Cannot do factorial of a negatice number");
      }

      lower = Math.max(lower, 1);
      upper = Math.max(upper, 1);

      int higherTerm = upper;
      int lowerTerm = lower;

      if (lower > upper)
      {
         switched = true;
         higherTerm = lowerTerm;
         lowerTerm = upper;
      }

      double multiplier = 1.0;
      for (int i = higherTerm; i > lowerTerm; i--)
      {
         multiplier *= i;
      }

      if (switched)
      {
         return 1.0 / multiplier;
      }
      else
      {
         return multiplier;
      }
   }
}
