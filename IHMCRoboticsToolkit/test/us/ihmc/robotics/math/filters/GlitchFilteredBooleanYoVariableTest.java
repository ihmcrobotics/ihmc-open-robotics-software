package us.ihmc.robotics.math.filters;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;

public class GlitchFilteredBooleanYoVariableTest
{
   public static final int WINDOW_SIZE = 10;
   private YoVariableRegistry registry;
   private BooleanYoVariable yoVariableToFilter;
   private GlitchFilteredBooleanYoVariable filteredVariable;

   @Before
   public void setUp()
   {
      registry = new YoVariableRegistry("testRegistry");
      yoVariableToFilter = new BooleanYoVariable("variableToFilter", registry);
      filteredVariable = new GlitchFilteredBooleanYoVariable("filteredVariable", registry, yoVariableToFilter, WINDOW_SIZE);
   }

   @After
   public void tearDown()
   {
      registry = null;
      yoVariableToFilter = null;
      filteredVariable = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testConstructors_Set_Get()
   {
      GlitchFilteredBooleanYoVariable number1 = new GlitchFilteredBooleanYoVariable("stringInt", WINDOW_SIZE);
      GlitchFilteredBooleanYoVariable number2 = new GlitchFilteredBooleanYoVariable("stringYoVariableRegistryInt", registry, WINDOW_SIZE);
      GlitchFilteredBooleanYoVariable number3 = new GlitchFilteredBooleanYoVariable("stringBooleanYoVariableInt", yoVariableToFilter, WINDOW_SIZE);
      GlitchFilteredBooleanYoVariable number4 = new GlitchFilteredBooleanYoVariable("stringYoVariableRegistryBooleanYoVariableInt", registry,
            yoVariableToFilter, WINDOW_SIZE);

      GlitchFilteredBooleanYoVariable array[] = { number1, number2, number3, number4 };

      for (int i = 0; i < array.length; i++)
      {
         assertFalse(array[i].getBooleanValue());
         //assertFalse(array[i].getPreviousBooleanValue());

         array[i].set(true);
         assertTrue(array[i].getBooleanValue());
         //assertTrue(array[i].getPreviousBooleanValue());

         array[i].set(false);
         assertFalse(array[i].getBooleanValue());
         //assertFalse(array[i].getPreviousBooleanValue());
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testUpdate()
   {
      int windowSize = 3;
      filteredVariable.set(true);
      filteredVariable.setWindowSize(windowSize);
      int numberOfSets = windowSize * 20;

//      for (int i = 0; i < numberOfSets; i++)
//      {
//         if (i % 2 == 0)
//         {
//            filteredVariable.update(false);
//            assertFalse(filteredVariable.getPreviousBooleanValue());
//         }
//         else
//         {
//            filteredVariable.update(true);
//            assertTrue(filteredVariable.getPreviousBooleanValue());
//         }
//         assertTrue(filteredVariable.getBooleanValue());
//      }

      
      int counter = 0;
      for (int i = 0; i < (windowSize + 10); i++)
      {

         filteredVariable.update(false);
         counter++;
         
         //assertFalse(filteredVariable.getPreviousBooleanValue());

         if (counter < windowSize )
         {
            assertTrue(filteredVariable.getBooleanValue());
         }
         else
         {
            assertFalse(filteredVariable.getBooleanValue());
         }
      }

      filteredVariable = null;
      try
      {
         filteredVariable.update();
         fail();
      }
      catch (RuntimeException rte)
      {
         //do nothing
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCounter()
   {
      int windowSize = 10;
      filteredVariable.set(true);
      filteredVariable.setWindowSize(windowSize);
      
      
      for (int i = 0; i < ( (int)(windowSize/2.0)); i++)
      {
    	  filteredVariable.update(filteredVariable.getBooleanValue());
    	  assertEquals(filteredVariable.counter.getIntegerValue(), 0);
      }
      
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testFiltering()
   {
      yoVariableToFilter.set(true);

      for (int i = 0; i < WINDOW_SIZE / 2; i++)
      {
         filteredVariable.update();
      }

      assertFalse(yoVariableToFilter.getBooleanValue() == filteredVariable.getBooleanValue());

      for (int i = 0; i < WINDOW_SIZE / 2; i++)
      {
         filteredVariable.update();
      }

      assertTrue(yoVariableToFilter.getBooleanValue() == filteredVariable.getBooleanValue());
   }
}