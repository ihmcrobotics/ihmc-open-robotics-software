package us.ihmc.utilities.parameterOptimization;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class IntegerParameterToOptimizeTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testZeroToOneConversions()
   {
      testZeroToOneConversion(0, 100);
      testZeroToOneConversion(-100, 1000);
      testZeroToOneConversion(1, 1);
      testZeroToOneConversion(100, -100);
      
   }
   
   private void testZeroToOneConversion(int min, int max)
   {
      String name = "test";

      ListOfParametersToOptimize listOfParametersToOptimize = new ListOfParametersToOptimize();
      IntegerParameterToOptimize integerParameterToOptimize = new IntegerParameterToOptimize(name, min, max, listOfParametersToOptimize);

      integerParameterToOptimize.setCurrentValueGivenZeroToOne(0.0);
      int value = integerParameterToOptimize.getCurrentValue();
      assertEquals(min, value);
      
      integerParameterToOptimize.setCurrentValueGivenZeroToOne(1.0);
      value = integerParameterToOptimize.getCurrentValue();
      assertEquals(max, value);
      

      
      for (double zeroToOne = 0.0; zeroToOne<1.0; zeroToOne = zeroToOne + 0.001)
      {
         integerParameterToOptimize.setCurrentValueGivenZeroToOne(zeroToOne);
         int integerValue = integerParameterToOptimize.getCurrentValue();
         
         double zeroToOneAgain = integerParameterToOptimize.getCurrentValueFromZeroToOne();
         integerParameterToOptimize.setCurrentValueGivenZeroToOne(zeroToOneAgain);
         int integerValueAgain = integerParameterToOptimize.getCurrentValue();

         double zeroToOneYetAgain = integerParameterToOptimize.getCurrentValueFromZeroToOne();

         assertEquals(zeroToOneAgain, zeroToOneYetAgain, 1e-7);
         assertEquals(integerValue, integerValueAgain);
      }      

   }

}
