package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;

import static org.junit.Assert.assertEquals;

public class ContactMotionTest
{
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCoefficientNumber()
   {
      assertEquals(3, ContactMotion.CONSTANT.getNumberOfCoefficients());
      assertEquals(4, ContactMotion.LINEAR.getNumberOfCoefficients());
   }
}
