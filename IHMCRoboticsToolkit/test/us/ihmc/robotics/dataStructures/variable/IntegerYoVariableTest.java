package us.ihmc.robotics.dataStructures.variable;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class IntegerYoVariableTest
{
   private YoVariableRegistry registry;
   private Random random;
   private IntegerYoVariable integerYoVariable;
   private static final double EPSILON = 1e-10;
   
   @Before
   public void setUp()
   {
      registry = new YoVariableRegistry("testRegistry");
      random = new Random(1776L);
      integerYoVariable = new IntegerYoVariable("test", registry);
   }
   
   @After
   public void tearDown()
   {
      registry = null;
      integerYoVariable = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetAndGet()
   {
      for (int i = 0; i < 100; i++)
      {
         int value = random.nextInt(Integer.MAX_VALUE);
         integerYoVariable.set(value);
         assertEquals(value, integerYoVariable.getIntegerValue());
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testIncrementDecrementAddSubtract()
   {
      int value = random.nextInt();
      integerYoVariable.set(value);
      
      integerYoVariable.increment();
      assertEquals(value + 1, integerYoVariable.getIntegerValue());
      
      integerYoVariable.decrement();
      assertEquals(value, integerYoVariable.getIntegerValue());
      
      integerYoVariable.add(value);
      assertEquals(value * 2, integerYoVariable.getIntegerValue());
      
      integerYoVariable.subtract(value);
      assertEquals(value, integerYoVariable.getIntegerValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testLargeValue()
   {
      int value = Integer.MAX_VALUE - 2;
      integerYoVariable.set(value);
      assertEquals(value, integerYoVariable.getIntegerValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testValueEquals()
   {
      boolean result = integerYoVariable.valueEquals(0);
      assertTrue(result);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetFinal()
   {
      assertEquals(0, integerYoVariable.getIntegerValue());
      integerYoVariable.set(0);
      assertEquals(0, integerYoVariable.getIntegerValue());
      
      int value = random.nextInt() + 1;
      integerYoVariable.set(value);
      assertEquals(value, integerYoVariable.getIntegerValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetValueFromDouble()
   {
      double doubleValue = random.nextDouble();
      int intValue = (int) Math.round(doubleValue);
      boolean notifyListeners = true;
      integerYoVariable.setValueFromDouble(doubleValue, notifyListeners);
      assertEquals(intValue, integerYoVariable.getIntegerValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetValueAsDouble()
   {
      int value = 15;
      assertEquals(0, integerYoVariable.getIntegerValue());
      integerYoVariable.set(value);
      double result = integerYoVariable.getValueAsDouble();
      assertEquals(15.0, result, EPSILON);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
public void testToString()
{
   assertEquals(integerYoVariable.getName() + ": " + integerYoVariable.getIntegerValue(), integerYoVariable.toString());
}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
public void testGetValueString()
{
   assertEquals(0, integerYoVariable.getIntegerValue());
   int value = random.nextInt();
   integerYoVariable.set(value);
   StringBuffer stringBuffer = new StringBuffer();
   integerYoVariable.getValueString(stringBuffer);
   assertEquals("" + value, stringBuffer.toString());
}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
public void testGetValueStringFromDouble()
{
   assertEquals(0, integerYoVariable.getIntegerValue());
   double doubleValue = random.nextDouble();
   int value = (int) Math.round(doubleValue);
   integerYoVariable.setValueFromDouble(doubleValue);
   StringBuffer stringBuffer = new StringBuffer();
   integerYoVariable.getValueStringFromDouble(stringBuffer, doubleValue);
   assertEquals("" + value, stringBuffer.toString()); 
}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
public void testGetYoVariableType()
{
   assertEquals(YoVariableType.INTEGER, integerYoVariable.getYoVariableType());
}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
public void testGetAndSetValueAsLongBits()
{
   int value = 57;
   integerYoVariable.set(value);
   long result = integerYoVariable.getValueAsLongBits();
   assertEquals(value, result);
   
   long longValue = 12345;
   boolean notifyListeners = true;
   integerYoVariable.setValueFromLongBits(longValue, notifyListeners);
   assertEquals(longValue, integerYoVariable.getValueAsLongBits());
}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
public void testDuplicate()
{
   IntegerYoVariable integerYoVariable2 = new IntegerYoVariable("var2", "descriptionTest", registry);
   YoVariableRegistry newRegistry = new YoVariableRegistry("newRegistry");
   IntegerYoVariable duplicate = integerYoVariable2.duplicate(newRegistry);
   assertEquals(integerYoVariable2.getName(), duplicate.getName());
   assertEquals(integerYoVariable2.getDescription(), duplicate.getDescription());
   assertEquals(integerYoVariable2.getManualScalingMin(), duplicate.getManualScalingMin(), EPSILON);
   assertEquals(integerYoVariable2.getManualScalingMax(), duplicate.getManualScalingMax(), EPSILON);
}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
public void testSetValue()
{
   IntegerYoVariable integerYoVariable2 = new IntegerYoVariable("var2", "descriptionTest", registry);
   boolean notifyListeners = true;
   integerYoVariable.setValue(integerYoVariable2, notifyListeners);
   assertEquals(integerYoVariable2.getIntegerValue(), integerYoVariable.getIntegerValue());
}

}
