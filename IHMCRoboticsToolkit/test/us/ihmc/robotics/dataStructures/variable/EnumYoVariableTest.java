package us.ihmc.robotics.dataStructures.variable;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class EnumYoVariableTest
{
   private enum EnumYoVariableTestEnums
   {
      ONE, TWO;
   }

   private EnumYoVariableTestEnums enumValue;

   private YoVariableRegistry registry = null;
   private static final double EPSILON = 1e-10;

   @Before
   public void setUp()
   {
      registry = new YoVariableRegistry("testRegistry");
   }

   @After
   public void tearDown()
   {
      registry = null;
   }

   @SuppressWarnings("deprecation")

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testConstructorNoDescription()
   {
      EnumYoVariable<EnumYoVariableTestEnums> enumYoVariable = new EnumYoVariable<EnumYoVariableTest.EnumYoVariableTestEnums>("enumYoVariable", registry,
            EnumYoVariableTestEnums.class);
      assertFalse(enumYoVariable == null);
      assertTrue(registry.getAllVariables().size() == 1);
      assertTrue(enumYoVariable.getName().equals("enumYoVariable"));
      assertTrue(registry.getVariable("enumYoVariable").equals(enumYoVariable));
   }

   @SuppressWarnings("deprecation")

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testConstructorWithDescription()
   {
      EnumYoVariable<EnumYoVariableTestEnums> enumYoVariable = new EnumYoVariable<EnumYoVariableTestEnums>("enumYoVariable", "enumYoVariable with description",
            registry, EnumYoVariableTestEnums.class, false);
      assertFalse(enumYoVariable == null);
      assertTrue(registry.getAllVariables().size() == 1);
      assertTrue(enumYoVariable.getName().equals("enumYoVariable"));
      assertTrue(registry.getVariable("enumYoVariable").equals(enumYoVariable));
      assertTrue(enumYoVariable.getDescription().equals("enumYoVariable with description"));
   }

   @SuppressWarnings("deprecation")

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCreateNoDescription()
   {
      EnumYoVariable<EnumYoVariableTestEnums> enumYoVariable = EnumYoVariable.create("enumYoVariable", EnumYoVariableTestEnums.class, registry);
      assertFalse(enumYoVariable == null);
      assertTrue(registry.getAllVariables().size() == 1);
      assertTrue(enumYoVariable.getName().equals("enumYoVariable"));
      assertTrue(registry.getVariable("enumYoVariable").equals(enumYoVariable));
   }

   @SuppressWarnings("deprecation")

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCreateWithDescription()
   {
      EnumYoVariable<EnumYoVariableTestEnums> enumYoVariable = EnumYoVariable.create("enumYoVariable", "enumYoVariable with description",
            EnumYoVariableTestEnums.class, registry, false);
      assertFalse(enumYoVariable == null);
      assertTrue(registry.getAllVariables().size() == 1);
      assertTrue(enumYoVariable.getName().equals("enumYoVariable"));
      assertTrue(registry.getVariable("enumYoVariable").equals(enumYoVariable));
      
      assertTrue(enumYoVariable.getDescription().equals("enumYoVariable with description"));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetAndValueEquals()
   {
      EnumYoVariable<EnumYoVariableTestEnums> enumYoVariable = new EnumYoVariable<EnumYoVariableTest.EnumYoVariableTestEnums>("enumYoVariable", registry,
            EnumYoVariableTestEnums.class, true);
      enumYoVariable.set(EnumYoVariableTestEnums.ONE);
      assertTrue(enumYoVariable.valueEquals(EnumYoVariableTestEnums.ONE));
      enumYoVariable.set(EnumYoVariableTestEnums.TWO);
      assertTrue(enumYoVariable.valueEquals(EnumYoVariableTestEnums.TWO));

      enumYoVariable.set(null);
      assertTrue(enumYoVariable.valueEquals(null));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetAndGet()
   {
      EnumYoVariable<EnumYoVariableTestEnums> enumYoVariable = new EnumYoVariable<EnumYoVariableTest.EnumYoVariableTestEnums>("enumYoVariable", registry,
            EnumYoVariableTestEnums.class);
      enumYoVariable.set(EnumYoVariableTestEnums.ONE);
      assertEquals(EnumYoVariableTestEnums.ONE, enumYoVariable.getEnumValue());
      enumYoVariable.set(EnumYoVariableTestEnums.TWO);
      assertEquals(EnumYoVariableTestEnums.TWO, enumYoVariable.getEnumValue());
      enumYoVariable.set(EnumYoVariableTestEnums.ONE,false);
      assertEquals(EnumYoVariableTestEnums.ONE, enumYoVariable.getEnumValue());
      enumYoVariable.set(EnumYoVariableTestEnums.TWO,false);
      assertEquals(EnumYoVariableTestEnums.TWO, enumYoVariable.getEnumValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetValues()
   {
      EnumYoVariable<EnumYoVariableTestEnums> enumYoVariable = new EnumYoVariable<EnumYoVariableTest.EnumYoVariableTestEnums>("enumYoVariable", registry,
            EnumYoVariableTestEnums.class);
      EnumYoVariableTestEnums[] enumTypeArray = enumYoVariable.getEnumValues();
      assertEquals(2, enumTypeArray.length);
      assertEquals(EnumYoVariableTestEnums.ONE, enumTypeArray[0]);
      assertEquals(EnumYoVariableTestEnums.TWO, enumTypeArray[1]);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetValueAsDoublePositiveNumber()
   {
      EnumYoVariable<EnumYoVariableTestEnums> enumYoVariable = new EnumYoVariable<EnumYoVariableTest.EnumYoVariableTestEnums>("enumYoVariable", registry,
            EnumYoVariableTestEnums.class);
      enumYoVariable.setValueFromDouble(0.0);
      assertEquals(EnumYoVariableTestEnums.ONE, enumYoVariable.getEnumValue());
      enumYoVariable.setValueFromDouble(0.25);
      assertEquals(EnumYoVariableTestEnums.ONE, enumYoVariable.getEnumValue());
      enumYoVariable.setValueFromDouble(0.5);
      assertEquals(EnumYoVariableTestEnums.TWO, enumYoVariable.getEnumValue());
      enumYoVariable.setValueFromDouble(1.0);
      assertEquals(EnumYoVariableTestEnums.TWO, enumYoVariable.getEnumValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetValueAsDoubleOutOfBoundsJustIgnoresIt()
   {
      EnumYoVariable<EnumYoVariableTestEnums> enumYoVariable = new EnumYoVariable<EnumYoVariableTest.EnumYoVariableTestEnums>("enumYoVariable", registry,
            EnumYoVariableTestEnums.class);
      EnumYoVariableTestEnums originalValue = enumYoVariable.getEnumValue();

      enumYoVariable.setValueFromDouble(2.0);
      assertEquals(originalValue, enumYoVariable.getEnumValue());

      enumYoVariable.setValueFromDouble(-100.0);
      assertEquals(originalValue, enumYoVariable.getEnumValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testForNull()
   {
      EnumYoVariable<EnumYoVariableTestEnums> enumYoVariable = new EnumYoVariable<EnumYoVariableTest.EnumYoVariableTestEnums>("enumYoVariable", "", registry,
            EnumYoVariableTestEnums.class, true);
      enumYoVariable.setValueFromDouble(EnumYoVariable.NULL_VALUE);
      assertEquals(enumYoVariable.getEnumValue(), null);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000,expected = RuntimeException.class)
   public void testNotAllowNull()
   {
      EnumYoVariable<EnumYoVariableTestEnums> enumYoVariable = new EnumYoVariable<EnumYoVariableTest.EnumYoVariableTestEnums>("enumYoVariable", registry,
            EnumYoVariableTestEnums.class);
      enumYoVariable.set(null);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAllowNull()
   {
      EnumYoVariable<EnumYoVariableTestEnums> enumYoVariable = new EnumYoVariable<EnumYoVariableTest.EnumYoVariableTestEnums>("enumYoVariable", "", registry,
            EnumYoVariableTestEnums.class, true);
      enumYoVariable.set(null);
      assertEquals(enumYoVariable.getEnumValue(), null);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetValueAsDouble()
   {
      EnumYoVariable<EnumYoVariableTestEnums> enumYoVariable = new EnumYoVariable<EnumYoVariableTest.EnumYoVariableTestEnums>("enumYoVariable", registry,
            EnumYoVariableTestEnums.class, true);
      enumYoVariable.set(EnumYoVariableTestEnums.ONE);
      assertEquals(0.0, enumYoVariable.getValueAsDouble(), EPSILON);
      enumYoVariable.set(EnumYoVariableTestEnums.TWO);
      assertEquals(1.0, enumYoVariable.getValueAsDouble(), EPSILON);
      enumYoVariable.set(null);
      assertEquals(-1, enumYoVariable.getValueAsDouble(), EPSILON);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testToString()
   {
      EnumYoVariable<EnumYoVariableTestEnums> enumYoVariable = new EnumYoVariable<EnumYoVariableTest.EnumYoVariableTestEnums>("enumYoVariable", registry,
            EnumYoVariableTestEnums.class);
      enumYoVariable.set(EnumYoVariableTestEnums.ONE);
      assertEquals("enumYoVariable: ONE", enumYoVariable.toString());
      enumYoVariable.set(EnumYoVariableTestEnums.TWO);
      assertEquals("enumYoVariable: TWO", enumYoVariable.toString());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetValueStringBufferWithNullValue()
   {
      EnumYoVariable<EnumYoVariableTestEnums> enumYoVariable = new EnumYoVariable<EnumYoVariableTest.EnumYoVariableTestEnums>("enumYoVariable", "", registry,
            EnumYoVariableTestEnums.class, true);
      enumYoVariable.set(null);

      StringBuffer valueBuffer = new StringBuffer();
      enumYoVariable.getValueString(valueBuffer);

      assertEquals("null", valueBuffer.toString());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetValueStringBuffer()
   {
      StringBuffer valueBuffer = new StringBuffer();
      EnumYoVariable<EnumYoVariableTestEnums> enumYoVariable = new EnumYoVariable<EnumYoVariableTest.EnumYoVariableTestEnums>("enumYoVariable", registry,
            EnumYoVariableTestEnums.class);

      enumYoVariable.set(EnumYoVariableTestEnums.ONE);
      enumYoVariable.getValueString(valueBuffer);
      assertEquals("ONE", valueBuffer.toString());

      enumYoVariable.set(EnumYoVariableTestEnums.TWO);
      enumYoVariable.getValueString(valueBuffer);
      assertEquals("ONETWO", valueBuffer.toString());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetYoVariableType()
   {
      EnumYoVariable<EnumYoVariableTestEnums> enumYoVariable = new EnumYoVariable<EnumYoVariableTest.EnumYoVariableTestEnums>("enumYoVariable", registry,
            EnumYoVariableTestEnums.class);
      assertEquals(YoVariableType.ENUM, enumYoVariable.getYoVariableType());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetValueAsLongBitsAndSetValueFromLongBits()
   {
      boolean notifyListeners = true;
      long longValue = 1;
      EnumYoVariable<EnumYoVariableTestEnums> enumYoVariable = new EnumYoVariable<EnumYoVariableTest.EnumYoVariableTestEnums>("enumYoVariable", registry,
            EnumYoVariableTestEnums.class, true);
      enumYoVariable.setValueFromLongBits(longValue, notifyListeners);
      assertEquals(longValue, enumYoVariable.getValueAsLongBits());

      try
      {
         longValue = 2;
         enumYoVariable.setValueFromLongBits(longValue);
         fail();
      }
      catch (RuntimeException rte)
      {
         //do nothing
      }
      
      enumYoVariable.set(null);
      assertEquals(-1, enumYoVariable.getValueAsLongBits());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetEnumType()
   {
      enumValue = EnumYoVariableTestEnums.ONE;
      EnumYoVariable<EnumYoVariableTestEnums> enumYoVariable = new EnumYoVariable<EnumYoVariableTestEnums>("enumYoVariable", registry,
            EnumYoVariableTestEnums.class);
      enumYoVariable.set(enumValue);

      assertEquals(enumValue.getClass(), enumYoVariable.getEnumType());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetValueStringFromDouble()
   {
      EnumYoVariable<EnumYoVariableTestEnums> enumYoVariable = new EnumYoVariable<EnumYoVariableTestEnums>("enumYoVariable", registry,
            EnumYoVariableTestEnums.class, true);
      StringBuffer stringBuffer = new StringBuffer();
      double doubleValue;

      doubleValue = -1.0;
      enumYoVariable.getValueStringFromDouble(stringBuffer, doubleValue);
      assertTrue(stringBuffer.toString().equals("null"));

      doubleValue = 0.0;
      enumYoVariable.getValueStringFromDouble(stringBuffer, doubleValue);
      assertTrue(stringBuffer.toString().equals("null" + "ONE"));

      doubleValue = 0.49;
      enumYoVariable.getValueStringFromDouble(stringBuffer, doubleValue);
      assertTrue(stringBuffer.toString().equals("null" + "ONE" + "ONE"));

      doubleValue = 0.5;
      enumYoVariable.getValueStringFromDouble(stringBuffer, doubleValue);
      assertTrue(stringBuffer.toString().equals("null" + "ONE" + "ONE" + "TWO"));

      doubleValue = 1.0;
      enumYoVariable.getValueStringFromDouble(stringBuffer, doubleValue);
      assertTrue(stringBuffer.toString().equals("null" + "ONE" + "ONE" + "TWO" + "TWO"));

      try
      {
         doubleValue = 1.5;
         enumYoVariable.getValueStringFromDouble(stringBuffer, doubleValue);
         fail();
      }
      catch (RuntimeException rte)
      {
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testDuplicate()
   {
      YoVariableRegistry newRegistry = new YoVariableRegistry("newRegistry");
      EnumYoVariable<EnumYoVariableTestEnums> enumYoVariable = new EnumYoVariable<EnumYoVariableTestEnums>("enumYoVariable", registry,
            EnumYoVariableTestEnums.class);
      EnumYoVariable<EnumYoVariableTestEnums> enumYoVariable2 = enumYoVariable.duplicate(newRegistry);

      assertTrue(enumYoVariable2.getName().equals(enumYoVariable.getName()));
      assertTrue(enumYoVariable2.getDescription().equals(enumYoVariable.getDescription()));
      assertTrue(enumYoVariable2.getEnumType().equals(enumYoVariable.getEnumType()));
      assertEquals(enumYoVariable2.getAllowNullValue(), enumYoVariable.getAllowNullValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetValue()
   {
      YoVariableRegistry newRegistry = new YoVariableRegistry("newRegistry2");
      boolean notifyListeners = true;
      EnumYoVariable<EnumYoVariableTestEnums> enumYoVariable = new EnumYoVariable<EnumYoVariableTestEnums>("enumYoVariable", registry,
            EnumYoVariableTestEnums.class);
      EnumYoVariable<EnumYoVariableTestEnums> enumYoVariable2 = new EnumYoVariable<EnumYoVariableTestEnums>("enumYoVariable", newRegistry,
            EnumYoVariableTestEnums.class);

      enumValue = EnumYoVariableTestEnums.TWO;
      enumYoVariable2.set(enumValue);
      enumYoVariable.setValue(enumYoVariable2, notifyListeners);
      assertEquals(enumValue, enumYoVariable.getEnumValue());

      enumValue = EnumYoVariableTestEnums.ONE;
      enumYoVariable2.set(enumValue);
      enumYoVariable.setValue(enumYoVariable2, notifyListeners);
      assertEquals(enumValue, enumYoVariable.getEnumValue());
   }
}