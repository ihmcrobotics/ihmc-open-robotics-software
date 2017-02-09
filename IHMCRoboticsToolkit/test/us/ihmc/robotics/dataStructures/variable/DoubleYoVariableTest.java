package us.ihmc.robotics.dataStructures.variable;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.text.DecimalFormat;
import java.text.FieldPosition;
import java.text.NumberFormat;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class DoubleYoVariableTest
{

   private YoVariableRegistry registry;
   private DoubleYoVariable doubleYoVariable1;
   private DoubleYoVariable doubleYoVariable2;
   private static final double EPSILON = 1e-10;
   private Random random = new Random(345345L);

   @Before
   public void setUp()
   {
      registry = new YoVariableRegistry("testRegistry");
      doubleYoVariable1 = new DoubleYoVariable("doubleYoVariable1", registry);
      doubleYoVariable2 = new DoubleYoVariable("doubleYoVariable2", "description2", registry, 0.0, 10.0);
   }

   @After
   public void tearDown()
   {
      doubleYoVariable1 = null;
      registry = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testDoubleYoVariableConstructorWithoutDescription()
   {
      assertTrue(doubleYoVariable1.getDoubleValue() == 0.0);
      assertEquals(doubleYoVariable1.getName(), "doubleYoVariable1");
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testDoubleYoVariableConstructorWithDescription()
   {
      String testDescription = "This is a test description.";
      DoubleYoVariable doubleYoVariableWithDescription = new DoubleYoVariable("doubleYoVariableWithDescription", testDescription, registry);
      assertTrue(doubleYoVariableWithDescription.getDoubleValue() == 0.0);
      assertEquals(doubleYoVariableWithDescription.getName(), "doubleYoVariableWithDescription");
      assertTrue(doubleYoVariableWithDescription.getDescription() == testDescription);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testToString()
   {
      double randomNumber = Math.random();
      doubleYoVariable1.set(randomNumber);
      assertEquals(doubleYoVariable1.toString(), "doubleYoVariable1: " + randomNumber);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testIsNaN()
   {
      assertFalse(doubleYoVariable2.isNaN());
      doubleYoVariable2.set(Double.NaN);
      assertTrue(doubleYoVariable2.isNaN());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAdditionWithDoubles()
   {
      double randomNumber1 = Math.random();
      double randomNumber2 = Math.random();

      doubleYoVariable1.set(randomNumber1);
      doubleYoVariable1.add(randomNumber2);
      double expectedSum = randomNumber1 + randomNumber2;
      assertTrue(doubleYoVariable1.getDoubleValue() == expectedSum);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSubtractionWithDoubles()
   {
      double randomNumber1 = Math.random();
      double randomNumber2 = Math.random();

      doubleYoVariable1.set(randomNumber1);
      doubleYoVariable1.sub(randomNumber2);
      double expectedDifference = randomNumber1 - randomNumber2;
      assertTrue(doubleYoVariable1.getDoubleValue() == expectedDifference);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testMultiplicationWithDoubles()
   {
      double randomNumber1 = Math.random();
      double randomNumber2 = Math.random();

      doubleYoVariable1.set(randomNumber1);
      doubleYoVariable1.mul(randomNumber2);
      double expectedProduct = randomNumber1 * randomNumber2;
      assertTrue(doubleYoVariable1.getDoubleValue() == expectedProduct);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAdditionWithDoubleYoVariables()
   {
      double randomNumber1 = Math.random();
      double randomNumber2 = Math.random();

      doubleYoVariable1.set(randomNumber1);
      doubleYoVariable2.set(randomNumber2);
      double expectedSum = doubleYoVariable1.getDoubleValue() + doubleYoVariable2.getDoubleValue();
      doubleYoVariable1.add(doubleYoVariable2);
      assertTrue(doubleYoVariable1.getDoubleValue() == expectedSum);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSubtractionWithDoubleYoVariables()
   {
      double randomNumber1 = Math.random();
      double randomNumber2 = Math.random();

      doubleYoVariable1.set(randomNumber1);
      doubleYoVariable2.set(randomNumber2);
      double expectedDifference = doubleYoVariable1.getDoubleValue() - doubleYoVariable2.getDoubleValue();
      doubleYoVariable1.sub(doubleYoVariable2);
      assertTrue(doubleYoVariable1.getDoubleValue() == expectedDifference);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testMultiplicationWithDoubleYoVariables()
   {
      double randomNumber1 = Math.random();
      double randomNumber2 = Math.random();

      doubleYoVariable1.set(randomNumber1);
      doubleYoVariable2.set(randomNumber2);
      double expectedProduct = doubleYoVariable1.getDoubleValue() * doubleYoVariable2.getDoubleValue();
      doubleYoVariable1.mul(doubleYoVariable2);
      assertTrue(doubleYoVariable1.getDoubleValue() == expectedProduct);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testValueEquals()
   {
      double randomNumber = Math.random();
      doubleYoVariable1.set(randomNumber);
      doubleYoVariable1.valueEquals(randomNumber);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetValueWithStringBuffer()
   {
      FieldPosition fieldPosition = new FieldPosition(NumberFormat.INTEGER_FIELD);
      NumberFormat doubleFormat = new DecimalFormat(" 0.00000;-0.00000");

      double randomNumber = 100000 * Math.random();
      randomNumber = Math.round(randomNumber);
      randomNumber /= 100000; //five decimal format as determined by doubleFormat under <p>getValue</p> method.
      StringBuffer stringBufferTest = new StringBuffer();
      stringBufferTest.append("Test case:");

      doubleYoVariable1.set(randomNumber);
      doubleYoVariable1.getValueString(stringBufferTest);

      StringBuffer expectedStringBuffer = new StringBuffer();
      expectedStringBuffer.append("Test case:");
      doubleFormat.format(randomNumber, expectedStringBuffer, fieldPosition);
      //         expectedStringBuffer.append(" " + randomNumber); //added space that occurs in <p>getValue</p> method.

      System.out.println("Expected String Buffer: " + expectedStringBuffer.toString());
      System.out.println("String Buffer: " + stringBufferTest.toString());

      assertEquals(expectedStringBuffer.toString(), stringBufferTest.toString());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetAndSetMethods()
   {
      double randomNumber = Math.random();
      doubleYoVariable1.set(randomNumber);
      doubleYoVariable2.set(doubleYoVariable1.getDoubleValue());
      assertTrue(doubleYoVariable1.getDoubleValue() == doubleYoVariable2.getDoubleValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetAndSetDoubleValue()
   {
      double randomNumber = Math.random();
      doubleYoVariable1.setValueFromDouble(randomNumber);
      doubleYoVariable2.setValueFromDouble(doubleYoVariable1.getValueAsDouble());
      assertTrue(doubleYoVariable1.getValueAsDouble() == doubleYoVariable2.getValueAsDouble());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetFinal()
   {
      doubleYoVariable1.set(0.0);
      doubleYoVariable1.set(0.0);
      
      doubleYoVariable1.set(10.0);
      assertEquals(10.0, doubleYoVariable1.getDoubleValue(), EPSILON); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetValueAsLongBitsAndSetValueFromLongBits()
   {
      long longValue = random.nextLong();
      doubleYoVariable1.setValueFromLongBits(longValue);
      assertEquals(longValue, doubleYoVariable1.getValueAsLongBits());
      
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("doubleYo", registry);
      doubleYoVariable.set(Double.NEGATIVE_INFINITY);
      
      doubleYoVariable1.setValue(doubleYoVariable, true);
      assertEquals(0xfff0000000000000L, doubleYoVariable1.getValueAsLongBits());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetYoVariableType()
   {
      assertTrue(doubleYoVariable1.getYoVariableType() == YoVariableType.DOUBLE);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testDuplicate()
   {
      String newName = "registry2000";
      double value = random.nextDouble();
      doubleYoVariable2.set(value);
      YoVariableRegistry newRegistry = new YoVariableRegistry(newName);
      DoubleYoVariable duplicate = doubleYoVariable2.duplicate(newRegistry);
      assertEquals(doubleYoVariable2.getDoubleValue(), duplicate.getDoubleValue(), EPSILON);
   }
}