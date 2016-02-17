package us.ihmc.robotics.kinematics.fourbar;

import java.util.Random;
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
   public void testJacobianRelationForConsecutiveSideActuation()
   {
      Random random = new Random(58932L);
      int numTests = 10;

      double[] boundsLength = new double[] {0.1, 2.0};
      double[] inputAngleBounds = new double[] {-0.4, 0.4};

      for (int i = 0; i < numTests; i++)
      {
         double L1 = getRandomValFromBounds(boundsLength, random);
         double L2 = getRandomValFromBounds(boundsLength, random);
         double L3 = getRandomValFromBounds(boundsLength, random);
         double L4 = getRandomValFromBounds(boundsLength, random);
         double inputAngle = getRandomValFromBounds(inputAngleBounds, random);

         testCalculators(L1, L2, L3, L4, inputAngle);
      }
   }

   private double getRandomValFromBounds(double[] bounds, Random random)
   {
      return bounds[0] + random.nextDouble() * (bounds[1] - bounds[0]);
   }

   private void testCalculators(double length_DA, double length_AB, double length_BC, double length_CD, double knownAngle)
   {
      // Fast runner calculations
      FourBarCalculatorFromFastRunner fastRunnerCalculator = new FourBarCalculatorFromFastRunner(length_DA, length_AB, length_BC, length_CD);
      fastRunnerCalculator.solveForAngleDAB(knownAngle);
      double outputFastRunnerCalculator = fastRunnerCalculator.getAngleABC();

      // Other calculations
      FourbarLink outputLink = new FourbarLink(length_DA);
      FourbarLink groundLink = new FourbarLink(length_AB);
      FourbarLink inputLink = new FourbarLink(length_BC);
      FourbarLink floatingLink = new FourbarLink(length_CD);

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
      double outputOtherCalculator = otherCalculator.calculateInputAngleFromOutputAngle(Math.PI - knownAngle); //TODO make sure

      // Compare
      assertEquals(outputFastRunnerCalculator, outputOtherCalculator, eps);
   }

}
