package com.yobotics.simulationconstructionset.util.math.functionGenerator;

import static org.junit.Assert.assertEquals;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class YoFunctionGeneratorTest
{
	YoFunctionGenerator yoFunctionGenerator;
   @Before
   public void setUp() throws Exception
   {
	   YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
	   yoFunctionGenerator = new YoFunctionGenerator("test", registry);
   }

   @After
   public void tearDown() throws Exception
   {
   }


   @Test
   public void testZeroFrequencyDC()
   {
	   yoFunctionGenerator.setMode(YoFunctionGeneratorMode.DC);

	   double amplitude = 1.0;
	   yoFunctionGenerator.setAmplitude(amplitude);
	   yoFunctionGenerator.setFrequency(0.0);

	   for(double time = 0.0; time< 10.0; time+=0.01)
	   {
		   assertEquals(amplitude, yoFunctionGenerator.getValue(time), 1e-10);

	   }
   }

   @Test
   public void testZeroFrequencySine()
   {
	   yoFunctionGenerator.setMode(YoFunctionGeneratorMode.SINE);

	   double amplitude = 1.0;
	   yoFunctionGenerator.setAmplitude(amplitude);
	   yoFunctionGenerator.setFrequency(0.0);
	   yoFunctionGenerator.setPhase(Math.PI/2.0);

	   for(double time = 0.0; time< 10.0; time+=0.01)
	   {
		  assertEquals(amplitude, yoFunctionGenerator.getValue(time), 1e-10);

	   }
   }

}
