package us.ihmc.robotics.dataStructures.variable;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.NoSuchElementException;

import org.apache.commons.lang3.builder.ReflectionToStringBuilder;
import org.apache.commons.lang3.builder.StandardToStringStyle;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class YoVariableTest
{
   private YoVariable<?> yoVariable = null;
   private YoVariableRegistry registry = null;
   private ArrayList<TestVariableChangedListener> variableChangedListeners = null;


   public YoVariableTest()
   {
   }

   @Before
   public void setUp()
   {
      YoVariableRegistry robotRegistry = new YoVariableRegistry("robot");
      
      registry = new YoVariableRegistry("testRegistry");
      robotRegistry.addChild(registry);
      
      yoVariable = new DoubleYoVariable("variableOne", registry);
      variableChangedListeners = new ArrayList<TestVariableChangedListener>();
   }

   @After
   public void tearDown()
   {
      yoVariable = null;
      registry = null;
      variableChangedListeners = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testFullNameEndsWith()
   {
      assertTrue(yoVariable.fullNameEndsWithCaseInsensitive("robot.testRegistry.variableOne"));
      assertTrue(yoVariable.fullNameEndsWithCaseInsensitive("testRegistry.variableOne"));
      assertTrue(yoVariable.fullNameEndsWithCaseInsensitive("variableOne"));

      assertTrue(!yoVariable.fullNameEndsWithCaseInsensitive("bot.testRegistry.variableOne"));
      assertTrue(!yoVariable.fullNameEndsWithCaseInsensitive(".testRegistry.variableOne"));
      assertTrue(!yoVariable.fullNameEndsWithCaseInsensitive("gistry.variableOne"));
      assertTrue(!yoVariable.fullNameEndsWithCaseInsensitive("ableOne"));
      assertTrue(!yoVariable.fullNameEndsWithCaseInsensitive("robot.testRegistr"));
      
      assertTrue(yoVariable.fullNameEndsWithCaseInsensitive("robot.testRegistry.VARIABLEONE"));
      assertFalse(yoVariable.fullNameEndsWithCaseInsensitive("Robot.testRegistry.variableOne"));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testValidVariable()
   {
      new DoubleYoVariable("foobar","",null);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000,expected = RuntimeException.class)
   public void testCantHaveADot()
   {
      new DoubleYoVariable("foo.bar", "", null);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000,expected = RuntimeException.class)
   public void testCantHaveAComma()
   {
      new DoubleYoVariable("foo,bar", "", null);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000,expected = RuntimeException.class)
   public void testCantHaveACarrot()
   {
      new DoubleYoVariable("foo^bar", "", null);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCanHaveAClosingBracket()
   {
      new DoubleYoVariable("foo]bar", "", null);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCanHaveAnOpeningBracket()
   {
      new DoubleYoVariable("foo[bar", "", null);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000,expected = RuntimeException.class)
   public void testCantHaveABackSlash()
   {
      new DoubleYoVariable("foo\\bar", "", null);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000,expected = RuntimeException.class)
   public void testCantHaveAQuote()
   {
      new DoubleYoVariable("foo\"bar", "", null);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000,expected = RuntimeException.class)
   public void testCantHaveASpace()
   {
      new DoubleYoVariable("foo bar", "", null);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000,expected = RuntimeException.class)
   public void testCantHaveASlash()
   {
      new DoubleYoVariable("foo/bar", "", null);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetBooleanValue()
   {
      BooleanYoVariable booleanVariable = new BooleanYoVariable("booleanVar", registry);
      booleanVariable.set(true);

      assert booleanVariable.getBooleanValue();
      booleanVariable.set(false);
      assert !booleanVariable.getBooleanValue();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetDescription()
   {
      DoubleYoVariable descrVariable = new DoubleYoVariable("booleanVar", "Description", registry);

      assertEquals(descrVariable.getDescription(), "Description");
      assertNotNull(yoVariable.getDescription());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetDoubleValue()
   {
      DoubleYoVariable doubleVariable = new DoubleYoVariable("doubleVar", registry);
      doubleVariable.set(15.6);

      assertEquals(doubleVariable.getDoubleValue(), 15.6, 1e-7);
   }

   private enum FooEnum {ONE, TWO, THREE;}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetEnumValue()
   {
      EnumYoVariable<FooEnum> enumVariable = EnumYoVariable.create("booleanVar", FooEnum.class, registry);
      enumVariable.set(FooEnum.TWO);

      assertEquals(enumVariable.getEnumValue(), FooEnum.TWO);
      assertFalse(enumVariable.getEnumValue() == FooEnum.ONE);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetFullNameWithNameSpace()
   {
      assertEquals(yoVariable.getFullNameWithNameSpace(), "robot.testRegistry.variableOne");
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetIntegerValue()
   {
      IntegerYoVariable integerVariable = new IntegerYoVariable("integerVariable", registry);
      integerVariable.set(5);

      assertEquals(integerVariable.getIntegerValue(), 5);
   }

//   @Test(timeout=300000)
//   public void testGetManualScalingMax()
//   {
//   }
//
//   @Test(timeout=300000)
//   public void testGetManualScalingMin()
//   {
//   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetName()
   {
      assertEquals(yoVariable.getName(), "variableOne");
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetName1()
   {
      StringBuffer buffer = new StringBuffer();
      yoVariable.getName(buffer);
      assertEquals(buffer.toString(), "variableOne");
   }

//   public void testGetNameAndValue()
//   {
//      // Don't test. Just for GUI.
//   }
//
//   public void testGetShortName()
//   {
//      // Don't test. Just for GUI.
//   }
//
//   public void testGetValue()
//   {
//      // Don't test. Just for GUI.
//   }


	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetYoVariableRegistry()
   {
      YoVariableRegistry registry = yoVariable.getYoVariableRegistry();
      assertNotNull(registry);
      assertEquals(registry, this.registry);
      assertEquals(registry.getVariable(yoVariable.getName()), yoVariable);
   }

//   @Test(timeout=300000)
//   public void testHasSameFullName()
//   {
//      // Not testing. Just used for check of repeat variables.
//   }

// public void testSet()
// {
//    // Already tested normal sets with testDouble, Int, Enum.
//    BooleanYoVariable booleanVariable = new BooleanYoVariable("booleanVar", registry);
//    DoubleYoVariable doubleVariable = new DoubleYoVariable("doubleVariable",  registry);
//    IntYoVariable intVariable = new IntYoVariable("intVariable", registry);
//    EnumYoVariable enumVariable = EnumYoVariable.create("enumVariable", FooEnum.class, registry);
//
//    boolean testPassed = true;
// }


//   public void testSetManualScalingMinMax()
//   {
//      // Not testing
//   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testToString()
   {
      EnumYoVariable<FooEnum> enumyoVariable = EnumYoVariable.create("enumYoVariable", FooEnum.class, registry);
      enumyoVariable.set(FooEnum.THREE);
      assertEquals("enumYoVariable: THREE", enumyoVariable.toString());

      IntegerYoVariable intyoVariable = new IntegerYoVariable("intYoVariable", registry);
      intyoVariable.set(1);
      assertEquals("intYoVariable: 1", intyoVariable.toString());

      DoubleYoVariable yoVariable = new DoubleYoVariable("doubleYoVariable", registry);
      yoVariable.set(0.0112);
      assertEquals("doubleYoVariable: 0.0112", yoVariable.toString());

      BooleanYoVariable booleanyoVariable = new BooleanYoVariable("booleanYoVariable", registry);
      booleanyoVariable.set(false);
      assertEquals("booleanYoVariable: false", booleanyoVariable.toString());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testValueEquals()
   {
      BooleanYoVariable booleanVariable = new BooleanYoVariable("booleanVar", registry);
      DoubleYoVariable doubleVariable = new DoubleYoVariable("doubleVariable", registry);
      IntegerYoVariable intVariable = new IntegerYoVariable("intVariable", registry);
      EnumYoVariable<FooEnum> enumVariable = EnumYoVariable.create("enumVariable", FooEnum.class, registry);

      booleanVariable.set(true);
      doubleVariable.set(1.4);
      intVariable.set(7);
      enumVariable.set(FooEnum.THREE);

      assert booleanVariable.valueEquals(true);
      assert doubleVariable.valueEquals(1.4);
      assert intVariable.valueEquals(7);
      assert enumVariable.valueEquals(FooEnum.THREE);

      booleanVariable.set(false);
      doubleVariable.set(4.5);
      intVariable.set(9);
      enumVariable.set(FooEnum.TWO);

      assert booleanVariable.valueEquals(false);
      assert doubleVariable.valueEquals(4.5);
      assert intVariable.valueEquals(9);
      assert enumVariable.valueEquals(FooEnum.TWO);

      assert !booleanVariable.valueEquals(true);
   }

//   public void testYoVariable1()
//   {
//      // Did a lot of constructing already. Not testing constructors.
//   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testNotifyVaribaleChangeListeners()
   {
      // create a bunch of Observers
      @SuppressWarnings("unused") int nObservers = 5;
      createVariableChangeListeners(5);

      // add them to the YoVariable
      yoVariable.removeAllVariableChangedListeners();
      addAllListenersToYoVariable();

      // create an observer that's not supposed to be notified.
      TestVariableChangedListener hearNoEvil = new TestVariableChangedListener();

      // make sure there's no event stored in the observers before we set
      for (TestVariableChangedListener listener : variableChangedListeners)
      {
         assertNull(listener.getLastVariableChanged());
      }

      assertNull(hearNoEvil.getLastVariableChanged());

      // now notify and check if the observers catch on

      yoVariable.notifyVariableChangedListeners();

      for (TestVariableChangedListener observer : variableChangedListeners)
      {
         assertEquals(observer.getLastVariableChanged(), yoVariable);
      }

      // make sure hearNoEvil is unaware
      assertNull(hearNoEvil.getLastVariableChanged());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAddVariableChangeListener()
   {
      // remove all observers, then add one new observer, and check if it can be removed without exceptions.
      yoVariable.removeAllVariableChangedListeners();
      TestVariableChangedListener listener = new TestVariableChangedListener();
      yoVariable.addVariableChangedListener(listener);
      yoVariable.removeVariableChangedListener(listener);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testRemoveAllVariableChangeListeners()
   {
      // create some observers, add them to yoVariable
      int nObservers = 5;
      createVariableChangeListeners(nObservers);
      addAllListenersToYoVariable();

      // let yoVariable notify observers. Assert that they got the event.

      yoVariable.notifyVariableChangedListeners();

      for (TestVariableChangedListener observer : variableChangedListeners)
      {
         assertEquals(observer.getLastVariableChanged(), yoVariable);
      }

      // Remove and reset observers. Let yoVariable notify observers again. Assert that the observers didn't notice anything.
      yoVariable.removeAllVariableChangedListeners();
      resetAllObservers();
      yoVariable.notifyVariableChangedListeners();

      for (TestVariableChangedListener observer : this.variableChangedListeners)
      {
         assertNull(observer.getLastVariableChanged());
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testRemoveObserver()
   {
      // create some observers, add them to yoVariable
      int nObservers = 5;
      createVariableChangeListeners(nObservers);
      addAllListenersToYoVariable();

      // let yoVariable notify observers. Assert that they got the event.
      yoVariable.notifyVariableChangedListeners();

      for (TestVariableChangedListener observer : variableChangedListeners)
      {
         assertEquals(observer.getLastVariableChanged(), yoVariable);
      }

      // Remove and reset observers. Let yoVariable notify observers again. Assert that the observers didn't notice anything.
      for (VariableChangedListener observer : variableChangedListeners)
      {
         yoVariable.removeVariableChangedListener(observer);
      }

      resetAllObservers();
      yoVariable.notifyVariableChangedListeners();

      for (TestVariableChangedListener observer : this.variableChangedListeners)
      {
         assertNull(observer.getLastVariableChanged());
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testRemoveObserverNonExistent1()
   {
      // make sure removing an observer that wasn't added throws an exception.
      try
      {
         yoVariable.removeVariableChangedListener(new TestVariableChangedListener());
         fail();
      }
      catch (NoSuchElementException e)
      {
         // pass.
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000,expected = NoSuchElementException.class)
   public void testRemoveObserverNonExistent2()
   {
      // make sure removing an observer that wasn't added throws an exception.
//      try
      {
         createVariableChangeListeners(5);
         addAllListenersToYoVariable();
         yoVariable.removeVariableChangedListener(new TestVariableChangedListener());
         //fail();
      }
//      catch (NoSuchElementException e)
      {
         // pass.
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testRecursiveCompareYoVariables() throws IllegalArgumentException, IllegalAccessException, SecurityException, NoSuchFieldException
   {
      YoVariableRegistry root0 = new YoVariableRegistry("root");
      DoubleYoVariable variable01 = new DoubleYoVariable("variableOne", root0);
      DoubleYoVariable variable02 = new DoubleYoVariable("variableTwo", root0);
      
      YoVariableRegistry root1 = new YoVariableRegistry("root");
      DoubleYoVariable variable11 = new DoubleYoVariable("variableOne", root1);
      DoubleYoVariable variable12 = new DoubleYoVariable("variableTwo", root1);
      
      YoVariableComparer comparer = new YoVariableComparer(Integer.MAX_VALUE, Integer.MAX_VALUE);
      
      assertTrue(comparer.compare(root0, root1));
      assertTrue(comparer.compare(variable01, variable11));
      assertTrue(comparer.compare(variable02, variable12));
      
      variable01.set(4.4);

      // If a comparer returns false, next time it might return true since it stores list of everything that has been compared. So you need to be careful!
      assertFalse(comparer.compare(root0, root1));

      assertFalse(comparer.compare(variable01, variable11));
      assertTrue(comparer.compare(variable02, variable12));
      
      variable11.set(4.4);
      
      assertTrue(comparer.compare(root0, root1));
      assertTrue(comparer.compare(variable01, variable11));
      assertTrue(comparer.compare(variable02, variable12));
      
      variable02.set(99.9);
      assertTrue(comparer.compare(variable01, variable11));
      assertFalse(comparer.compare(variable02, variable12));
      assertFalse(comparer.compare(root0, root1));
      
      variable12.set(99.9);
      
      assertTrue(comparer.compare(variable01, variable11));
      assertTrue(comparer.compare(variable02, variable12));
      assertTrue(comparer.compare(root0, root1));

   }

   private void createVariableChangeListeners(int numberOfListeners)
   {
      for (int i = 0; i < numberOfListeners; i++)
      {
         TestVariableChangedListener listener = new TestVariableChangedListener();
         this.variableChangedListeners.add(listener);
      }
   }

   private void addAllListenersToYoVariable()
   {
      for (TestVariableChangedListener observer : variableChangedListeners)
      {
         yoVariable.addVariableChangedListener(observer);
      }
   }

   private void resetAllObservers()
   {
      for (TestVariableChangedListener observer : variableChangedListeners)
      {
         observer.reset();
      }
   }


   /**
    * An VariableChangedListener that stores the most recent event and source, to be used for tests
    *
    * @author Twan Koolen
    *
    */
   private class TestVariableChangedListener implements VariableChangedListener
   {
      private YoVariable<?> lastVariableChanged = null;

      public void variableChanged(YoVariable<?> v)
      {
         lastVariableChanged = v;
      }

      public YoVariable<?> getLastVariableChanged()
      {
         return lastVariableChanged;
      }

      public void reset()
      {
         lastVariableChanged = null;
      }
   }
   
   private class YoVariableComparer
   {
      public YoVariableComparer(int maxDepth, int maxSize) throws NoSuchFieldException, SecurityException
      {
      }


      public boolean compare(DoubleYoVariable variable01, DoubleYoVariable variable11)
      {
         return ReflectionToStringBuilder.toString(variable01, StandardToStringStyle.NO_CLASS_NAME_STYLE).equals(ReflectionToStringBuilder.toString(variable11, StandardToStringStyle.NO_CLASS_NAME_STYLE));

      }

      public boolean compare(YoVariableRegistry root0, YoVariableRegistry root1)
      {
         return ReflectionToStringBuilder.toString(root0, StandardToStringStyle.NO_CLASS_NAME_STYLE).equals(ReflectionToStringBuilder.toString(root1, StandardToStringStyle.NO_CLASS_NAME_STYLE));
      }
   }
}
