package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.junit.jupiter.api.Test;

import static org.junit.Assert.assertEquals;

public class ContactMotionTest
{
   @Test
   public void testCoefficientNumber()
   {
      assertEquals(3, ContactMotion.CONSTANT.getNumberOfCoefficients());
      assertEquals(4, ContactMotion.LINEAR.getNumberOfCoefficients());
   }
}
