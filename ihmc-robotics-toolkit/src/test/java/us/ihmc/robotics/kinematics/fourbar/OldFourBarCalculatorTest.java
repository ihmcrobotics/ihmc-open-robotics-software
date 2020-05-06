package us.ihmc.robotics.kinematics.fourbar;

import static java.lang.Math.PI;
import static us.ihmc.robotics.Assert.assertEquals;

import org.junit.jupiter.api.Test;

public class OldFourBarCalculatorTest
{
   private static final double eps = 1e-7;

   private OldFourbarLink outputLink, groundLink, inputLink, floatingLink;
   private OldFourbarProperties fourBarProperties;

   @Test
   public void testSquare()
   {
      outputLink = new OldFourbarLink(1.0);
      groundLink = new OldFourbarLink(1.0);
      inputLink = new OldFourbarLink(1.0);
      floatingLink = new OldFourbarLink(1.0);

      fourBarProperties = new OldFourbarProperties()
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
      double outputOtherCalculator = otherCalculator.calculateInputAngleFromOutputAngle(Math.PI / 2.0);
      assertEquals(PI / 2, outputOtherCalculator, eps);
   }
}
