package us.ihmc.robotics.kinematics.fourbar;

import java.util.Random;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.sqrt;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

/**
 * It makes sure that the fast runner four bar calculator and the calculator used in Beast, Steppr, and Wanderer give the same result
 */
public class FourBarCalculatorsComparisonTest
{
   private static final double eps = 1e-7;

   @DeployableTestMethod
   @Test(timeout = 300000)
   public void equalOuputAnglesForRandomQuadrilatteralTest()
   {
      Random rand = new Random(1986L);
      for (int j = 0; j < 10000; j++)
      {
         // Generate quadrilatteral
         double e = 100 * (rand.nextDouble() + 0.001);
         double k1 = rand.nextDouble();
         double k2 = rand.nextDouble();
         double d1 = e * abs(rand.nextGaussian());
         double d2 = e * abs(rand.nextGaussian());
         double DE = e * k1, DF = e * k2, BE = e * (1 - k1), BF = e * (1 - k2);
         double AE = d1, CF = d2;
         double AD = sqrt(DE * DE + AE * AE), DAE = atan2(DE, AE), ADE = atan2(AE, DE);
         double AB = sqrt(AE * AE + BE * BE), BAE = atan2(BE, AE), ABE = atan2(AE, BE);
         double CD = sqrt(CF * CF + DF * DF), CDF = atan2(CF, DF), DCF = atan2(DF, CF);
         double BC = sqrt(BF * BF + CF * CF), CBF = atan2(CF, BF), BCF = atan2(BF, CF);
         double BAD = DAE + BAE, ABC = ABE + CBF, BCD = BCF + DCF, ADC = ADE + CDF;

         // Test
         testCalculators(AD, AB, BC, CD, BAD);
      }
   }

   private void testCalculators(double AD, double AB, double BC, double CD, double knownAngle)
   {
      // (1) Fast runner calculations
      FourBarCalculatorFromFastRunner fastRunnerCalculator = new FourBarCalculatorFromFastRunner(AD, AB, BC, CD);
      fastRunnerCalculator.solveForAngleDAB(knownAngle);
      double outputFastRunnerCalculator = fastRunnerCalculator.getAngleABC();

      // (2) Other calculations
      FourbarLink outputLink = new FourbarLink(AD);
      FourbarLink groundLink = new FourbarLink(AB);
      FourbarLink inputLink = new FourbarLink(BC);
      FourbarLink floatingLink = new FourbarLink(CD);

      FourbarProperties fourBarProperties = new FourbarProperties()
      {
         @Override
         public boolean isElbowDown()
         {
            // TODO Auto-generated method stub
            return false;
         }

         @Override
         public double getRightLinkageBeta0()
         {
            // TODO Auto-generated method stub
            return 0;
         }

         @Override
         public double getLeftLinkageBeta0()
         {
            // TODO Auto-generated method stub
            return 0;
         }

         @Override
         public FourbarLink getOutputLink()
         {
            return outputLink;
         }

         @Override
         public FourbarLink getInputLink()
         {
            return inputLink;
         }

         @Override
         public FourbarLink getGroundLink()
         {
            return groundLink;
         }

         @Override
         public FourbarLink getFloatingLink()
         {
            return floatingLink;
         }
      };

      FourbarCalculator otherCalculator = new FourbarCalculator(fourBarProperties);
      double outputOtherCalculator = otherCalculator.calculateInputAngleFromOutputAngle(Math.PI - knownAngle);

      // (3) Compare
      assertEquals(outputFastRunnerCalculator, outputOtherCalculator, eps);
   }
}
