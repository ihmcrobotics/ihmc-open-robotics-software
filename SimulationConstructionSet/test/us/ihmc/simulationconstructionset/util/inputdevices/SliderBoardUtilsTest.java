package us.ihmc.simulationconstructionset.util.inputdevices;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.random.RandomTools;

public class SliderBoardUtilsTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSimpleConversion()
   {
      int sliderBoardMax = 128;
      int sliderValue = 64;
      double max = 1.0;
      double min = 0.0;
      double exponent = 1.0;
      double hires = 0.5;
      
      double value = SliderBoardUtils.valueRatioConvertToDoubleWithExponents(sliderValue, sliderBoardMax, max, min, exponent, hires);
      assertEquals(0.5, value, 1e-7);
      
      int intValue = SliderBoardUtils.valueRatioConvertToIntWithExponents(sliderBoardMax, max, min, exponent, value);
      assertEquals(intValue, sliderValue);
      
      sliderValue = 0;

      value = SliderBoardUtils.valueRatioConvertToDoubleWithExponents(sliderValue, sliderBoardMax, max, min, exponent, hires);
      assertEquals(0.0, value, 1e-7);
      
      intValue = SliderBoardUtils.valueRatioConvertToIntWithExponents(sliderBoardMax, max, min, exponent, value);
      assertEquals(intValue, sliderValue);
      
      sliderValue = 128;

      value = SliderBoardUtils.valueRatioConvertToDoubleWithExponents(sliderValue, sliderBoardMax, max, min, exponent, hires);
      assertEquals(1.0, value, 1e-7);
      
      intValue = SliderBoardUtils.valueRatioConvertToIntWithExponents(sliderBoardMax, max, min, exponent, value);
      assertEquals(intValue, sliderValue);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSimpleSquareConversion()
   {
      int sliderBoardMax = 128;
      double max = 1.0;
      double min = 0.0;
      double exponent = 2.0;
      double hires = 0.5;
      
      int sliderValue = 64;
      double value = SliderBoardUtils.valueRatioConvertToDoubleWithExponents(sliderValue, sliderBoardMax, max, min, exponent, hires);
      assertEquals(0.5, value, 1e-7);
      
      int intValue = SliderBoardUtils.valueRatioConvertToIntWithExponents(sliderBoardMax, max, min, exponent, hires, value);
      assertEquals(intValue, sliderValue);
      
      sliderValue = 0;

      value = SliderBoardUtils.valueRatioConvertToDoubleWithExponents(sliderValue, sliderBoardMax, max, min, exponent, hires);
      assertEquals(0.0, value, 1e-7);
      
      intValue = SliderBoardUtils.valueRatioConvertToIntWithExponents(sliderBoardMax, max, min, exponent, hires, value);
      assertEquals(intValue, sliderValue);
      
      sliderValue = 128;

      value = SliderBoardUtils.valueRatioConvertToDoubleWithExponents(sliderValue, sliderBoardMax, max, min, exponent, hires);
      assertEquals(1.0, value, 1e-7);
      
      intValue = SliderBoardUtils.valueRatioConvertToIntWithExponents(sliderBoardMax, max, min, exponent, hires, value);
      assertEquals(intValue, sliderValue);
      
      
      sliderValue = (64 + 128)/2;

      value = SliderBoardUtils.valueRatioConvertToDoubleWithExponents(sliderValue, sliderBoardMax, max, min, exponent, hires);
      assertEquals(0.5 + (0.5*0.5)*0.5, value, 1e-7);
      
      intValue = SliderBoardUtils.valueRatioConvertToIntWithExponents(sliderBoardMax, max, min, exponent, hires, value);
      assertEquals(intValue, sliderValue);
      
      sliderValue = 32;

      value = SliderBoardUtils.valueRatioConvertToDoubleWithExponents(sliderValue, sliderBoardMax, max, min, exponent, hires);
      assertEquals(0.5 - (0.5*0.5)*0.5, value, 1e-7);
      
      intValue = SliderBoardUtils.valueRatioConvertToIntWithExponents(sliderBoardMax, max, min, exponent, hires, value);
      assertEquals(intValue, sliderValue);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testRandomConversions()
   {
      Random random = new Random(1776L);
      
      int sliderBoardMax = random.nextInt(1000); 
      double max = RandomTools.generateRandomDouble(random, 0.5, 10.0);
      double min = RandomTools.generateRandomDouble(random, -10.0, 0.4);
      double exponent = RandomTools.generateRandomDouble(random, 0.1, 10.0);
      
      
      for (int sliderValue=0; sliderValue<sliderBoardMax; sliderValue++)
      {
         double value = SliderBoardUtils.valueRatioConvertToDoubleWithExponents(sliderValue, sliderBoardMax, max, min, exponent);
         int intValue = SliderBoardUtils.valueRatioConvertToIntWithExponents(sliderBoardMax, max, min, exponent, value);
         assertEquals(intValue, sliderValue);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testRandomConversionsWithHires()
   {
      Random random = new Random(1776L);
      
      int sliderBoardMax = random.nextInt(1000); 
      double max = RandomTools.generateRandomDouble(random, 0.5, 10.0);
      double min = RandomTools.generateRandomDouble(random, -10.0, 0.4);
      double exponent = RandomTools.generateRandomDouble(random, 0.1, 10.0);
      double hires = RandomTools.generateRandomDouble(random, min, max);
      
      for (int sliderValue=0; sliderValue<sliderBoardMax; sliderValue++)
      {
         double value = SliderBoardUtils.valueRatioConvertToDoubleWithExponents(sliderValue, sliderBoardMax, max, min, exponent, hires);
         int intValue = SliderBoardUtils.valueRatioConvertToIntWithExponents(sliderBoardMax, max, min, exponent, hires, value);
         assertEquals(intValue, sliderValue);
      }
   }

}
