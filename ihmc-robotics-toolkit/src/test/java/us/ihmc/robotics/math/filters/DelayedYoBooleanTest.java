package us.ihmc.robotics.math.filters;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class DelayedYoBooleanTest
{
   private static YoRegistry registry;
   private static YoBoolean variableToDelay;
   private static Boolean DEBUG = false;
   private int ticksToDelay;
   
   @BeforeEach
   public void setUp()
   {
      registry = new YoRegistry("registry");
      variableToDelay = new YoBoolean("variableToDelay", registry);
   }
   
   @AfterEach
   public void tearDown()
   {
      registry = null;
      variableToDelay = null;
   }

	@Test
   public void testDelayedYoVariableMultipleTickDelays()
   {
      for (ticksToDelay = 0; ticksToDelay < 10; ticksToDelay++)
      {
         variableToDelay.set(true);

         DelayedYoBoolean delayedYoVariable = new DelayedYoBoolean("delayedVariable" + ticksToDelay, "", variableToDelay, ticksToDelay,
               registry);

         int ticksToTest = 100;
         boolean[] valuesToSet = new boolean[ticksToTest];

         for (int i = 0; i < valuesToSet.length; i++)
         {
            if (Math.random() < .5)
               valuesToSet[i] = true;
            else
               valuesToSet[i] = false;
         }

         assertEquals(delayedYoVariable.getBooleanValue(), true);

         for (int i = 0; i < ticksToTest; i++)
         {
            variableToDelay.set(valuesToSet[i]);
            delayedYoVariable.update(); 

            if (i < ticksToDelay)
            {
               assertEquals(delayedYoVariable.getBooleanValue(), true);
            }
            else
            {
               assertEquals(delayedYoVariable.getBooleanValue(), valuesToSet[i - ticksToDelay]);
            }
         }
      }
   }

	@Test
   public void testDelayedYoVariableOneTickDelay()
   {
      ticksToDelay = 1;

      variableToDelay.set(false);
      DelayedYoBoolean delayedYoVariable = new DelayedYoBoolean("delayedVariable" + ticksToDelay, "", variableToDelay, ticksToDelay, registry);
      assertEquals(delayedYoVariable.getBooleanValue(), false);

      variableToDelay.set(true);
      delayedYoVariable.update();
      assertEquals(delayedYoVariable.getBooleanValue(), false);

      variableToDelay.set(false);
      delayedYoVariable.update();
      assertEquals(delayedYoVariable.getBooleanValue(), true);

      variableToDelay.set(true);
      delayedYoVariable.update();
      assertEquals(delayedYoVariable.getBooleanValue(), false);

      delayedYoVariable.update();
      assertEquals(delayedYoVariable.getBooleanValue(), true);

      delayedYoVariable.update();
      assertEquals(delayedYoVariable.getBooleanValue(), true);

      delayedYoVariable.update();
      assertEquals(delayedYoVariable.getBooleanValue(), true);
   }

	@Test
   public void testDelayedYoVariableZeroTickDelay()
   {
      ticksToDelay = 0;

      variableToDelay.set(false);
      DelayedYoBoolean delayedYoVariable = new DelayedYoBoolean("delayedVariable" + ticksToDelay, "", variableToDelay, ticksToDelay, registry);
      assertEquals(delayedYoVariable.getBooleanValue(), false);

      variableToDelay.set(true);
      delayedYoVariable.update();
      assertEquals(delayedYoVariable.getBooleanValue(), true);

      variableToDelay.set(false);
      delayedYoVariable.update();
      assertEquals(delayedYoVariable.getBooleanValue(), false);

      variableToDelay.set(true);
      delayedYoVariable.update();
      assertEquals(delayedYoVariable.getBooleanValue(), true);

      delayedYoVariable.update();
      assertEquals(delayedYoVariable.getBooleanValue(), true);

      delayedYoVariable.update();
      assertEquals(delayedYoVariable.getBooleanValue(), true);

      delayedYoVariable.update();
      assertEquals(delayedYoVariable.getBooleanValue(), true);
   }

	@Test
   public void testUpdateWithZero()
   {
      ticksToDelay = 0;
      variableToDelay.set(false);
      DelayedYoBoolean delayedYoVariable = new DelayedYoBoolean("delayedVariable" + ticksToDelay, "", variableToDelay, ticksToDelay, registry);
      delayedYoVariable.getInternalState("Should be all false", DEBUG);      
      
      variableToDelay.set(true);
      delayedYoVariable.update();
      delayedYoVariable.getInternalState("Should be all true", DEBUG);
      
      assertEquals(delayedYoVariable.getBooleanValue(), true);
      delayedYoVariable.update();
      assertEquals(delayedYoVariable.getBooleanValue(), true);
   }

	@Test
   public void testReset()
   {  
      ticksToDelay = 10;
      variableToDelay.set(false);
      DelayedYoBoolean delayedYoVariable = new DelayedYoBoolean("delayedVariable" + ticksToDelay, "", variableToDelay, ticksToDelay, registry);
      
      for(int i = 0; i < ticksToDelay; i++)
      {
         assertEquals(delayedYoVariable.getBooleanValue(), false);
         delayedYoVariable.update();
      }
      
      delayedYoVariable.getInternalState("Should be all false", DEBUG);
      
      variableToDelay.set(true);
      delayedYoVariable.update();  
      
      for(int i = 0; i < ticksToDelay; i++)
      {
         assertEquals(delayedYoVariable.getBooleanValue(), false);
         delayedYoVariable.update();  
      }
      assertEquals(delayedYoVariable.getBooleanValue(), true);

      variableToDelay.set(false);
      delayedYoVariable.update(); 
      delayedYoVariable.getInternalState("Should be all true, except for the end", DEBUG);
      
      delayedYoVariable.reset();
      delayedYoVariable.update();  

      for(int i = 0; i < ticksToDelay; i++)
      {
         assertEquals(delayedYoVariable.getBooleanValue(), false);
         delayedYoVariable.update();  
      }
      delayedYoVariable.getInternalState("Should be all false", DEBUG);
   }
}