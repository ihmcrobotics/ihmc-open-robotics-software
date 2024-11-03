package us.ihmc.robotics.math.filters;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class GlitchFilteredYoBooleanTest
{
   public static final int WINDOW_SIZE = 10;
   private YoRegistry registry;
   private YoBoolean yoVariableToFilter;
   private GlitchFilteredYoBoolean filteredVariable;

   @BeforeEach
   public void setUp()
   {
      registry = new YoRegistry("testRegistry");
      yoVariableToFilter = new YoBoolean("variableToFilter", registry);
      filteredVariable = new GlitchFilteredYoBoolean("filteredVariable", registry, yoVariableToFilter, WINDOW_SIZE);
   }

   @AfterEach
   public void tearDown()
   {
      registry = null;
      yoVariableToFilter = null;
      filteredVariable = null;
   }

	@Test
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

	@Test
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

	@Test
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

	@Test
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