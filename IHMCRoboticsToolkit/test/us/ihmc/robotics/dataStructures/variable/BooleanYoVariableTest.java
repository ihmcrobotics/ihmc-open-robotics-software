package us.ihmc.robotics.dataStructures.variable;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;


public class BooleanYoVariableTest
{
   private BooleanYoVariable booleanYoVariable;
   private YoVariableRegistry registry;
   private static final double EPSILON = 1e-10;

   @Before
   public void setUp()
   {
      registry = new YoVariableRegistry("testRegistry");
      booleanYoVariable = new BooleanYoVariable("booleanVariable", registry);
   }

   @After
   public void tearDown()
   {
      booleanYoVariable = null;
      registry = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testBooleanYoVariable()
   {
      assertEquals("booleanVariable", booleanYoVariable.getName());
      assertEquals("testRegistry", booleanYoVariable.getYoVariableRegistry().getName());
      assertEquals(false, booleanYoVariable.getBooleanValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testValueEquals()
   {
      booleanYoVariable.set(false);
      assertFalse(booleanYoVariable.valueEquals(true));

      booleanYoVariable.set(true);
      assertTrue(booleanYoVariable.valueEquals(true));

      booleanYoVariable.set(false);
      assertTrue(booleanYoVariable.valueEquals(false));

      booleanYoVariable.set(true);
      assertFalse(booleanYoVariable.valueEquals(false));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetAndSetBooleanYoVariable()
   {
      booleanYoVariable.set(false);
      assertFalse(booleanYoVariable.getBooleanValue());

      booleanYoVariable.set(true);
      assertTrue(booleanYoVariable.getBooleanValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSet_boolean_boolean()
   {
      assertFalse(booleanYoVariable.getBooleanValue());

      boolean value = true;
      boolean notifyListeners = true;
      boolean result = booleanYoVariable.set(value, notifyListeners);
      assertTrue(booleanYoVariable.getBooleanValue());
      assertTrue(result);

      result = booleanYoVariable.set(value, notifyListeners);
      assertFalse(result);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetAndSetAsDouble()
   {
      Random rng = new Random();
      double testRandom = 0;

      booleanYoVariable.setValueFromDouble(0.0);
      assertFalse(booleanYoVariable.getBooleanValue());

      testRandom = (double) rng.nextInt(23000) / (double) rng.nextInt(5);

      booleanYoVariable.setValueFromDouble(10000);
      assertTrue(booleanYoVariable.getBooleanValue());

      booleanYoVariable.setValueFromDouble(-0.4);
      assertFalse(booleanYoVariable.getBooleanValue());
      
      for(int counter = 0; counter < 100; counter++)
      {
	      testRandom = rng.nextDouble();
	      booleanYoVariable.setValueFromDouble(testRandom);
	      if (testRandom >= 0.5)
	         assertTrue(booleanYoVariable.getBooleanValue());
	      else
	         assertFalse(booleanYoVariable.getBooleanValue());
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetValueAsDouble()
   {
      assertFalse(booleanYoVariable.getBooleanValue());
      double result = booleanYoVariable.getValueAsDouble();
      assertEquals(0.0, result, EPSILON);

      booleanYoVariable.set(true);
      assertTrue(booleanYoVariable.getBooleanValue());
      result = booleanYoVariable.getValueAsDouble();
      assertEquals(1.0, result, EPSILON);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testToString()
   {
      booleanYoVariable.set(false);
      assertEquals(booleanYoVariable.toString(), "booleanVariable: false");
      booleanYoVariable.set(true);
      assertEquals(booleanYoVariable.toString(), "booleanVariable: true");
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetValueWithStringBuffer()
   {
      StringBuffer testStringBuffer = new StringBuffer();
      testStringBuffer.append("Test Case: ");
      booleanYoVariable.set(false);
      booleanYoVariable.getValueString(testStringBuffer);
      testStringBuffer.append(" ");
      booleanYoVariable.getValueStringFromDouble(testStringBuffer, 1.0);

      StringBuffer expectedTestStringBuffer = new StringBuffer();
      expectedTestStringBuffer.append("Test Case: false true");

      assertEquals(testStringBuffer.toString(), expectedTestStringBuffer.toString());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetAndSetValueAsLongBits()
   {
      assertFalse(booleanYoVariable.getBooleanValue());
      long value = booleanYoVariable.getValueAsLongBits();
      assertEquals(0, value);

      booleanYoVariable.set(true);
      assertTrue(booleanYoVariable.getBooleanValue());
      value = booleanYoVariable.getValueAsLongBits();
      assertEquals(1, value);

      boolean notifyListeners = true;
      booleanYoVariable.setValueFromLongBits(value, notifyListeners);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testYoVariableType()
   {
      assertEquals(booleanYoVariable.getYoVariableType(), YoVariableType.BOOLEAN);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testDuplicate()
   {
	   YoVariableRegistry newRegistry = new YoVariableRegistry("newTestRegistry");
	   BooleanYoVariable val, testVal;
	   
	   booleanYoVariable.set(true);
	   val = booleanYoVariable.duplicate(newRegistry);
	   testVal = (BooleanYoVariable)newRegistry.getAllVariables().get(0);
	   assertEquals(booleanYoVariable.getBooleanValue(), val.getBooleanValue());
	   assertEquals(booleanYoVariable.getBooleanValue(), testVal.getBooleanValue());
	   assertTrue(val.getYoVariableRegistry().areEqual(newRegistry));
	   assertTrue(val.getYoVariableRegistry().areEqual(testVal.getYoVariableRegistry()));
	   newRegistry = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetValue()
   {
	   boolean valSet = false;
	   YoVariableRegistry newRegistry = new YoVariableRegistry("newTestRegistry");
	   BooleanYoVariable testBoolean = new BooleanYoVariable("testBooleanVariable", newRegistry);
	   testBoolean.set(true);
	   valSet = booleanYoVariable.setValue(testBoolean, false);
	   assertTrue(valSet);
	   assertEquals(booleanYoVariable.getBooleanValue(), testBoolean.getBooleanValue());
	   assertTrue(booleanYoVariable.getYoVariableRegistry().areEqual(registry));
   }
   
}