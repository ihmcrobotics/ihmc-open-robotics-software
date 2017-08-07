package us.ihmc.robotics.math.filters;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class GlitchFilteredYoBooleanTest
{
   public static final int WINDOW_SIZE = 10;
   private YoVariableRegistry registry;
   private YoBoolean yoVariableToFilter;
   private GlitchFilteredYoBoolean filteredVariable;

   @Before
   public void setUp()
   {
      registry = new YoVariableRegistry("testRegistry");
      yoVariableToFilter = new YoBoolean("variableToFilter", registry);
      filteredVariable = new GlitchFilteredYoBoolean("filteredVariable", registry, yoVariableToFilter, WINDOW_SIZE);
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
      GlitchFilteredYoBoolean number1 = new GlitchFilteredYoBoolean("stringInt", WINDOW_SIZE);
      GlitchFilteredYoBoolean number2 = new GlitchFilteredYoBoolean("stringYoVariableRegistryInt", registry, WINDOW_SIZE);
      GlitchFilteredYoBoolean number3 = new GlitchFilteredYoBoolean("stringBooleanYoVariableInt", yoVariableToFilter, WINDOW_SIZE);
      GlitchFilteredYoBoolean number4 = new GlitchFilteredYoBoolean("stringYoVariableRegistryBooleanYoVariableInt", registry,
            yoVariableToFilter, WINDOW_SIZE);

      GlitchFilteredYoBoolean array[] = { number1, number2, number3, number4 };

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