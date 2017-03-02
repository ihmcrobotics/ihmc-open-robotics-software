package us.ihmc.robotics.math.filters;

import static org.junit.Assert.assertEquals;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;


public class DelayedBooleanYoVariableTest
{
   private static YoVariableRegistry registry;
   private static BooleanYoVariable variableToDelay;
   private static Boolean DEBUG = false;
   private int ticksToDelay;
   
   @Before
   public void setUp()
   {
      registry = new YoVariableRegistry("registry");
      variableToDelay = new BooleanYoVariable("variableToDelay", registry);
   }
   
   @After
   public void tearDown()
   {
      registry = null;
      variableToDelay = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testDelayedYoVariableMultipleTickDelays()
   {
      for (ticksToDelay = 0; ticksToDelay < 10; ticksToDelay++)
      {
         variableToDelay.set(true);

         DelayedBooleanYoVariable delayedYoVariable = new DelayedBooleanYoVariable("delayedVariable" + ticksToDelay, "", variableToDelay, ticksToDelay,
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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testDelayedYoVariableOneTickDelay()
   {
      ticksToDelay = 1;

      variableToDelay.set(false);
      DelayedBooleanYoVariable delayedYoVariable = new DelayedBooleanYoVariable("delayedVariable" + ticksToDelay, "", variableToDelay, ticksToDelay, registry);
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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testDelayedYoVariableZeroTickDelay()
   {
      ticksToDelay = 0;

      variableToDelay.set(false);
      DelayedBooleanYoVariable delayedYoVariable = new DelayedBooleanYoVariable("delayedVariable" + ticksToDelay, "", variableToDelay, ticksToDelay, registry);
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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testUpdateWithZero()
   {
      ticksToDelay = 0;
      variableToDelay.set(false);
      DelayedBooleanYoVariable delayedYoVariable = new DelayedBooleanYoVariable("delayedVariable" + ticksToDelay, "", variableToDelay, ticksToDelay, registry);
      delayedYoVariable.getInternalState("Should be all false", DEBUG);      
      
      variableToDelay.set(true);
      delayedYoVariable.update();
      delayedYoVariable.getInternalState("Should be all true", DEBUG);
      
      assertEquals(delayedYoVariable.getBooleanValue(), true);
      delayedYoVariable.update();
      assertEquals(delayedYoVariable.getBooleanValue(), true);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testReset()
   {  
      ticksToDelay = 10;
      variableToDelay.set(false);
      DelayedBooleanYoVariable delayedYoVariable = new DelayedBooleanYoVariable("delayedVariable" + ticksToDelay, "", variableToDelay, ticksToDelay, registry);
      
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