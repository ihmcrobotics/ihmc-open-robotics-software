package us.ihmc.robotics.kinematics.fourbar;

import static java.lang.Math.PI;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class FourBarCalculatorTest
{
   private static final double eps = 1e-7;

   private FourbarLink outputLink, groundLink, inputLink, floatingLink;
   private FourbarProperties fourBarProperties;
      
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSquare()
   {
      outputLink = new FourbarLink(1.0);
      groundLink = new FourbarLink(1.0);
      inputLink = new FourbarLink(1.0);
      floatingLink = new FourbarLink(1.0);
      
      fourBarProperties = new FourbarProperties()
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
