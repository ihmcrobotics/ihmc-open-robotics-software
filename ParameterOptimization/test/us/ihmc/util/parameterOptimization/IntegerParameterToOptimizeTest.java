package us.ihmc.util.parameterOptimization;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class IntegerParameterToOptimizeTest
{
   @Test
   public void testGetBitsOfResolution()
   {
      testNBit(0, 0, 1);
      testNBit(0, 1, 1);
      testNBit(0, 2, 2);
      testNBit(0, 3, 2);
      testNBit(0, 4, 3);
      testNBit(0, 126, 7);
      testNBit(0, 127, 7);
      testNBit(0, 128, 8);
      
      testNBit(55, 55+254, 8);
      testNBit(55, 55+255, 8);
      testNBit(55, 55+256, 9);
   }

   
   private void testNBit(int min, int max, int expectedNumberOfBits)
   {
      String name = "test";
      ListOfParametersToOptimize listOfParametersToOptimize = new ListOfParametersToOptimize();
      
      IntegerParameterToOptimize integerParameterToOptimize = new IntegerParameterToOptimize(name, min, max, listOfParametersToOptimize);

      assertEquals(expectedNumberOfBits, integerParameterToOptimize.getBitsOfResolution());
   }

}
