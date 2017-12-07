package us.ihmc.robotics.kinematics.fourbar;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.sqrt;

import java.util.Random;

/**
 * The goal is to determine which of the two four bar calculators (the one including derivatives and 
 * the one used for Beast) is faster
 */

public class BenchmarkCompareFourBarCalculators
{
   private double startTimeFastRunnerCalculator, totalTimeFastRunnerCalculator;
   private double startTimeOtherCalculator, totalTimeOtherCalculator;
   private Random rand = new Random(1986L);

   private FourbarLink outputLink, groundLink, inputLink, floatingLink;
   private FourbarProperties fourBarProperties;
   
   public BenchmarkCompareFourBarCalculators()
   {
      // FAST RUNNER CALCULATIONS
      startTimeFastRunnerCalculator = System.currentTimeMillis();
      for (int j = 0; j < 500000; j++) // set the number of quadrilaterals such that at least one second goes by 
      {
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

         ConstantSideFourBarCalculatorWithDerivatives fastRunnerCalculator = new ConstantSideFourBarCalculatorWithDerivatives(AD, AB, BC, CD);
         fastRunnerCalculator.updateAnglesGivenAngleDAB(BAD);
      }
      totalTimeFastRunnerCalculator = System.currentTimeMillis() - startTimeFastRunnerCalculator;

      // OTHER CALCULATIONS
      startTimeOtherCalculator = System.currentTimeMillis();
      for (int j = 0; j < 500000; j++)
      {
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

         outputLink = new FourbarLink(AD);
         groundLink = new FourbarLink(AB);
         inputLink = new FourbarLink(BC);
         floatingLink = new FourbarLink(CD);

         fourBarProperties = new FourbarProperties()
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
         otherCalculator.calculateInputAngleFromOutputAngle(Math.PI - BAD);
      }
      totalTimeOtherCalculator = System.currentTimeMillis() - startTimeOtherCalculator;
      
      if (totalTimeFastRunnerCalculator < totalTimeOtherCalculator) System.out.println("The FastRunner calculator is faster");
      else System.out.println("The FastRunner calculator is slower");
      
      System.out.println("500,000 quadrilaterals computed");
      
      System.out.println("\n- Fast Runner time: " + totalTimeFastRunnerCalculator + " (ms)"+ "\n- Other calculator time: " + totalTimeOtherCalculator + " (ms)");
   }
   
   public static void main(String[] args)
   {
      new BenchmarkCompareFourBarCalculators();
   }
}
