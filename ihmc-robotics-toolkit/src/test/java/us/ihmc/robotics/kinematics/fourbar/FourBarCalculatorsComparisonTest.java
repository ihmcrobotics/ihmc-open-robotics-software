package us.ihmc.robotics.kinematics.fourbar;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.sqrt;
import static us.ihmc.robotics.Assert.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;
/**
 * It makes sure that the four bar calculator including derivatives and the calculator used in Beast, Steppr, and Wanderer give the same result
 */
public class FourBarCalculatorsComparisonTest
{
   /*
    * Notation used:
    * 
    *    D--------A
    *    |\      /|
    *    | \e   / |
    *    |  \  /  |
    *    |   \/   |
    *    |   /\   |
    *    |  /  \  |
    *    | /f   \ |
    *    |/      \|
    *    C--------B
    *    
    *    For the calculator that is not from fastRunner:
    *    AD = outputLink
    *    AB = groundLink
    *    BC = inputLink
    *    CD = floatingLink
    *    
    *    It takes as an input the supplementary of angle BAD
    *        
    */
   private static final double eps = 1e-7;

   @Test
   public void equalOuputAnglesForRandomQuadrilatteralTest()
   {
      Random rand = new Random(1986L);
      for (int j = 0; j < 150000; j++) // set the number of quadrilaterals such that the tests runs in about 0.5 seconds
      {
         // Generate quadrilatteral
         double e = 100 * (rand.nextDouble() + 0.001);
         double k1 = rand.nextDouble();
         double k2 = rand.nextDouble();
         double d1 = e * abs(rand.nextGaussian());
         double d2 = e * abs(rand.nextGaussian());
         
         double AE = d1, CF = d2;
         double DE = e * k1, BE = e * (1 - k1); 
         double DF = e * k2, BF = e * (1 - k2);
         
         double AD = sqrt(DE * DE + AE * AE), DAE = atan2(DE, AE);
         double AB = sqrt(AE * AE + BE * BE), BAE = atan2(BE, AE);
         double CD = sqrt(CF * CF + DF * DF);
         double BC = sqrt(BF * BF + CF * CF);
         double BAD = DAE + BAE;

         // Test
         testCalculators(AD, AB, BC, CD, BAD);
      }
   }

   private void testCalculators(double AD, double AB, double BC, double CD, double knownAngle)
   {
      // (1) Fast runner calculations
      FourBarCalculator fastRunnerCalculator = new FourBarCalculator();
      fastRunnerCalculator.setSideLengths(AB, BC, CD, AD);
      fastRunnerCalculator.updateAnglesGivenAngleDAB(knownAngle);
      double outputFastRunnerCalculator = fastRunnerCalculator.getAngleABC();

      // (2) Other calculations
      final OldFourbarLink outputLink = new OldFourbarLink(AD);
      final OldFourbarLink groundLink = new OldFourbarLink(AB);
      final OldFourbarLink inputLink = new OldFourbarLink(BC);
      final OldFourbarLink floatingLink = new OldFourbarLink(CD);

      OldFourbarProperties fourBarProperties = new OldFourbarProperties()
      {
         @Override
         public boolean isElbowDown()
         {
            return false;
         }

         @Override
         public double getRightLinkageBeta0()
         {
            return 0;
         }

         @Override
         public double getLeftLinkageBeta0()
         {
            return 0;
         }

         @Override
         public OldFourbarLink getOutputLink()
         {
            return outputLink;
         }

         @Override
         public OldFourbarLink getInputLink()
         {
            return inputLink;
         }

         @Override
         public OldFourbarLink getGroundLink()
         {
            return groundLink;
         }

         @Override
         public OldFourbarLink getFloatingLink()
         {
            return floatingLink;
         }
      };

      OldFourbarCalculator otherCalculator = new OldFourbarCalculator(fourBarProperties);
      double outputOtherCalculator = otherCalculator.calculateInputAngleFromOutputAngle(Math.PI - knownAngle);

      // (3) Compare
      assertEquals(outputFastRunnerCalculator, outputOtherCalculator, eps);
   }
}
