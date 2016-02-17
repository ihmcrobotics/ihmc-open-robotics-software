package us.ihmc.robotics.kinematics.fourbar;

import static java.lang.Math.PI;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class FourBarCalculatorTest
{
   private static final double eps = 1e-7;

   @DeployableTestMethod
   @Test(timeout = 300000)
   public void testSquare()
   {
      FourbarLink outputLink = new FourbarLink(1.0);
      FourbarLink groundLink = new FourbarLink(1.0);
      FourbarLink inputLink = new FourbarLink(1.0);
      FourbarLink floatingLink = new FourbarLink(1.0);

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
      double outputOtherCalculator = otherCalculator.calculateInputAngleFromOutputAngle(Math.PI / 2.0);
      assertEquals(PI / 2, outputOtherCalculator, eps);
   }
}
