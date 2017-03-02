package us.ihmc.robotics.dataStructures.variable;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class LongYoVariableTest
{
   private YoVariableRegistry registry;
   private Random random;
   private LongYoVariable longYoVariable;
   private static final double EPSILON = 1e-10;

   @Before
   public void setUp()
   {
      registry = new YoVariableRegistry("testRegistry");
      random = new Random(1776L);
      longYoVariable = new LongYoVariable("test", registry);
   }

   @After
   public void tearDown()
   {
      registry = null;
      longYoVariable = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetAndGet()
   {
      for (int i = 0; i < 100; i++)
      {
         long value = random.nextLong();
         boolean notify = random.nextBoolean();
         longYoVariable.set(value, notify);
         assertEquals(value, longYoVariable.getLongValue());
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testIncrementDecrementAddSubtract()
   {
      long value = random.nextLong();
      longYoVariable.set(value);

      longYoVariable.increment();
      assertEquals(value + 1, longYoVariable.getLongValue());

      longYoVariable.decrement();
      assertEquals(value, longYoVariable.getLongValue());

      longYoVariable.add(value);
      assertEquals(value * 2, longYoVariable.getLongValue());

      longYoVariable.subtract(value);
      assertEquals(value, longYoVariable.getLongValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testLargeValue()
   {
      long value = Long.MAX_VALUE - 2;
      longYoVariable.set(value);
      assertEquals(value, longYoVariable.getLongValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testValueEquals()
   {
      assertTrue(longYoVariable.valueEquals(0));

      long number = random.nextLong();
      longYoVariable.set(number);
      assertTrue(longYoVariable.valueEquals(number));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetFinal()
   {
      assertEquals(0, longYoVariable.getLongValue());
      longYoVariable.set(0);
      assertEquals(0, longYoVariable.getLongValue());

      int value = random.nextInt() + 1;
      longYoVariable.set(value);
      assertEquals(value, longYoVariable.getLongValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetValueFromDouble()
   {
      double doubleValue = random.nextDouble();
      int intValue = (int) Math.round(doubleValue);
      boolean notifyListeners = true;
      longYoVariable.setValueFromDouble(doubleValue, notifyListeners);
      assertEquals(intValue, longYoVariable.getLongValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetValueAsDouble()
   {
      assertEquals(0.0, longYoVariable.getValueAsDouble(), EPSILON);
      long value = 15;
      longYoVariable.set(value);
      double result = (double) longYoVariable.getValueAsDouble();
      assertEquals(15.0, result, EPSILON);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testToString()
   {
      assertEquals(longYoVariable.getName() + ": " + longYoVariable.getLongValue(), longYoVariable.toString());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetValueString()
   {
      assertEquals(0, longYoVariable.getLongValue());
      int value = random.nextInt();
      longYoVariable.set(value);
      StringBuffer stringBuffer = new StringBuffer();
      longYoVariable.getValueString(stringBuffer);
      assertEquals("" + value, stringBuffer.toString());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetValueStringFromDouble()
   {
      assertEquals(0, longYoVariable.getLongValue());
      double doubleValue = random.nextDouble();
      int value = (int) Math.round(doubleValue);
      longYoVariable.setValueFromDouble(doubleValue);
      StringBuffer stringBuffer = new StringBuffer();
      longYoVariable.getValueStringFromDouble(stringBuffer, doubleValue);
      assertEquals("" + value, stringBuffer.toString());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetYoVariableType()
   {
      assertEquals(YoVariableType.LONG, longYoVariable.getYoVariableType());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetAndSetValueAsLongBits()
   {
      int value = 57;
      longYoVariable.set(value);
      long result = longYoVariable.getValueAsLongBits();
      assertEquals(value, result);

      long longValue = 12345;
      boolean notifyListeners = true;
      longYoVariable.setValueFromLongBits(longValue, notifyListeners);
      assertEquals(longValue, longYoVariable.getValueAsLongBits());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testDuplicate()
   {
      LongYoVariable longYoVariable2 = new LongYoVariable("var2", "descriptionTest", registry);
      YoVariableRegistry newRegistry = new YoVariableRegistry("newRegistry");
      LongYoVariable duplicate = longYoVariable2.duplicate(newRegistry);
      assertEquals(longYoVariable2.getName(), duplicate.getName());
      assertEquals(longYoVariable2.getDescription(), duplicate.getDescription());
      assertEquals(longYoVariable2.getManualScalingMin(), duplicate.getManualScalingMin(), EPSILON);
      assertEquals(longYoVariable2.getManualScalingMax(), duplicate.getManualScalingMax(), EPSILON);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetValue()
   {
      LongYoVariable longYoVariable2 = new LongYoVariable("var2", "descriptionTest", registry);
      boolean notifyListeners = true;
      longYoVariable.setValue(longYoVariable2, notifyListeners);
      assertEquals(longYoVariable2.getLongValue(), longYoVariable.getLongValue());
   }
}