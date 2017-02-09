package us.ihmc.robotics.dataStructures.variable;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class YoVariableListTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCommonUsage()
   {
      YoVariableList varList = new YoVariableList("listOne");
      YoVariableRegistry registryOne = new YoVariableRegistry("registryOne");
      YoVariableRegistry registryTwo = new YoVariableRegistry("registryTwo");

      BooleanYoVariable booleanOne = new BooleanYoVariable("booleanOne", registryOne);
      DoubleYoVariable doubleOne = new DoubleYoVariable("doubleOne", registryOne);
      varList.addVariable(booleanOne);
      varList.addVariable(doubleOne);

      BooleanYoVariable booleanTwo = new BooleanYoVariable("booleanTwo", registryTwo);
      DoubleYoVariable doubleTwo = new DoubleYoVariable("doubleTwo", registryTwo);
      varList.addVariable(booleanTwo);
      varList.addVariable(doubleTwo);

      // Here's the tricky case. Same name variable booleanOne but with a
      // different name space.
      // Design question is should varList.getVariable("booleanOne") return
      // the first one, both, or throw an exception?
      // Right now it returns the first one. This has problems with
      // graphGroups and such and should be fixed later
      BooleanYoVariable repeatBooleanOne = new BooleanYoVariable("booleanOne", registryTwo);
      varList.addVariable(repeatBooleanOne);

      BooleanYoVariable notIncluded = new BooleanYoVariable("notIncluded", registryTwo);

      assertEquals("listOne", varList.getName());
      assertEquals(0, varList.getIndexOfVariable(booleanOne));
      assertEquals(1, varList.getIndexOfVariable(doubleOne));
      assertEquals(2, varList.getIndexOfVariable(booleanTwo));
      assertEquals(3, varList.getIndexOfVariable(doubleTwo));
      assertEquals(4, varList.getIndexOfVariable(repeatBooleanOne));

      assertEquals(-1, varList.getIndexOfVariable(notIncluded));

      assertTrue(varList.containsVariable(booleanOne));
      assertTrue(varList.containsVariable(doubleOne));
      assertTrue(varList.containsVariable(booleanTwo));
      assertTrue(varList.containsVariable(doubleTwo));
      assertTrue(varList.containsVariable(repeatBooleanOne));
      assertFalse(varList.containsVariable(notIncluded));

      assertTrue(null == varList.getVariable(-1));
      assertTrue(booleanOne == varList.getVariable(0));
      assertTrue(doubleOne == varList.getVariable(1));
      assertTrue(booleanTwo == varList.getVariable(2));
      assertTrue(doubleTwo == varList.getVariable(3));
      assertTrue(repeatBooleanOne == varList.getVariable(4));
      assertTrue(null == varList.getVariable(5));

      assertTrue(varList.hasVariableWithName("booleanOne"));
      assertTrue(varList.hasVariableWithName("registryOne.booleanOne"));
      assertTrue(varList.hasVariableWithName("doubleOne"));
      assertTrue(varList.hasVariableWithName("registryOne.doubleOne"));
      assertTrue(varList.hasVariableWithName("booleanTwo"));
      assertTrue(varList.hasVariableWithName("registryTwo.booleanTwo"));
      assertTrue(varList.hasVariableWithName("doubleTwo"));
      assertTrue(varList.hasVariableWithName("registryTwo.doubleTwo"));
      assertTrue(varList.hasVariableWithName("registryTwo.booleanOne"));

      assertFalse(varList.hasVariableWithName("notIncluded"));
      assertFalse(varList.hasVariableWithName("registryOne.doubleTwo"));

      assertTrue(booleanOne == varList.getVariable("booleanOne"));
      assertTrue(doubleOne == varList.getVariable("doubleOne"));
      assertTrue(booleanTwo == varList.getVariable("booleanTwo"));
      assertTrue(doubleTwo == varList.getVariable("doubleTwo"));
      assertTrue(booleanOne == varList.getVariable("registryOne.booleanOne"));
      assertTrue(doubleOne == varList.getVariable("registryOne.doubleOne"));
      assertTrue(booleanTwo == varList.getVariable("registryTwo.booleanTwo"));
      assertTrue(doubleTwo == varList.getVariable("registryTwo.doubleTwo"));

      assertTrue(repeatBooleanOne == varList.getVariable("registryTwo.booleanOne"));

      assertTrue(null == varList.getVariable("registryOne.doubleTwo"));
      assertTrue(null == varList.getVariable("notIncluded"));

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.5)
	@Test(timeout = 30000)
   public void testGetPerformanceInLargeList()
   {
      // Test should take O(n) or O(n lg n) approximately. Was taking O(n^2)
      // with old implementation, which made data file reading really slow.
      // Automatic test somewhat tests the speed of this but need to run fast
      // so use small number.
      // Have tested manually by using numberOfVariables = 20000 and it takes
      // less than 5 seconds.

      YoVariableRegistry rootRegistry = new YoVariableRegistry("rootRegistry");
      YoVariableRegistry registryOne = new YoVariableRegistry("registryOne");
      YoVariableRegistry registryTwo = new YoVariableRegistry("registryTwo");
      YoVariableRegistry registryThree = new YoVariableRegistry("registryThree");

      rootRegistry.addChild(registryOne);
      registryOne.addChild(registryTwo);
      registryTwo.addChild(registryThree);

      DoubleYoVariable t = new DoubleYoVariable("t", registryThree);
      DoubleYoVariable time = new DoubleYoVariable("time", registryThree);
      t.set(1.1);
      time.set(2.2);

      int numberOfVariables = 4000;
      ArrayList<YoVariable<?>> variables = new ArrayList<YoVariable<?>>();
      YoVariableList varList = new YoVariableList("test");

      for (int i = 0; i < numberOfVariables; i++)
      {
         DoubleYoVariable variableA = new DoubleYoVariable("variable" + i, registryThree);
         DoubleYoVariable variableB = new DoubleYoVariable("variable" + i, registryTwo);
         variableA.set(Math.random());
         variableB.set(Math.random());

         variables.add(variableA);
         variables.add(variableB);
         varList.addVariable(variableA);
         varList.addVariable(variableB);
      }

      assertEquals(variables.size(), varList.size());

      for (int i = 0; i < variables.size(); i++)
      {
         YoVariable<?> yoVariable = variables.get(i);

         assertTrue(varList.hasVariableWithName(yoVariable.getFullNameWithNameSpace()));
         assertTrue(yoVariable == varList.getVariable(yoVariable.getFullNameWithNameSpace()));
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testToString()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoVariableList list = new YoVariableList("list");

      DoubleYoVariable a = new DoubleYoVariable("a", registry);
      DoubleYoVariable b = new DoubleYoVariable("b", registry);
      DoubleYoVariable c = new DoubleYoVariable("c", registry);

      list.addVariable(a);
      list.addVariable(b);
      list.addVariable(c);

      assertEquals(a.toString() + "\n" + b.toString() + "\n" + c.toString() + "\n", list.toString());

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAddVariables()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoVariableList list = new YoVariableList("list");
      YoVariableList listTwo = new YoVariableList("listTwo");
      YoVariableList listThree = new YoVariableList("listThree");
      YoVariableList listFour = new YoVariableList("listFour");

      DoubleYoVariable a = new DoubleYoVariable("a", registry);
      DoubleYoVariable b = new DoubleYoVariable("b", registry);
      DoubleYoVariable c = new DoubleYoVariable("c", registry);

      //YoVariableList
      list.addVariable(a);
      assertEquals(1, list.size());
      list.addVariable(a); // Ignores if added twice.
      assertEquals(1, list.size());

      listTwo.addVariable(b);
      listTwo.addVariable(c);
      assertEquals(2, listTwo.size());

      list.addVariables(listTwo);
      assertEquals(3, list.size());

      //YoVariable[]
      listThree.addVariable(a);

      YoVariable<?>[] array = new YoVariable[2];
      array[0] = b;
      array[1] = c;

      listThree.addVariables(array);

      //ArrayList
      ArrayList<YoVariable<?>> arrayList = new ArrayList<YoVariable<?>>();
      arrayList.add(b);
      arrayList.add(c);

      listFour.addVariable(a);

      listFour.addVariables(arrayList);

      //assertions
      for (int i = 0; i < list.size(); i++)
      {
         assertEquals(list.getVariable(i), listThree.getVariable(i));
         assertEquals(list.getVariable(i), listFour.getVariable(i));
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCommonUsageTwo()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoVariableList list = new YoVariableList("list");
      YoVariableList listTwo = new YoVariableList("listTwo");
      YoVariableList listThree = new YoVariableList("listThree");
      YoVariableList List = new YoVariableList("List");

      assertTrue(list.isEmpty());

      DoubleYoVariable a = new DoubleYoVariable("a", registry);
      DoubleYoVariable b = new DoubleYoVariable("b", registry);
      DoubleYoVariable c = new DoubleYoVariable("c", registry);
      DoubleYoVariable f = new DoubleYoVariable("f", registry); //variable will not be added to list

      list.addVariable(a);
      list.addVariable(b);
      list.addVariable(c);

      list.removeVariable(b);
      list.removeVariable(f); //attempt to remove variable not in list

      listTwo.addVariable(a);
      listTwo.addVariable(c);

      assertTrue(list.compareTo(listThree) < 0);
      assertTrue(listThree.compareTo(list) > 0);
      assertTrue(list.compareTo(List) == 0);

      assertEquals(list.toString(), listTwo.toString());

      String[] names = list.getVariableNames();
      for (int i = 0; i < names.length; i++)
      {
         YoVariable<?> variable = list.getVariable(i);
         String name = variable.toString();
         assertEquals(names[i], name.substring(0, 1));
      }

      assertFalse(list.isEmpty());

      list.removeAllVariables();
      assertEquals(list.toString(), listThree.toString());

      YoVariable<?>[] allVariables = listTwo.getAllVariables();

      for (int i = 0; i < listTwo.size(); i++)
      {
         assertEquals(allVariables[i].toString(), listTwo.getVariable(i).toString());
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetMatchingVariables()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoVariableList list = new YoVariableList("list");

      assertTrue(list.isEmpty());

      DoubleYoVariable a = new DoubleYoVariable("a_arm", registry);
      DoubleYoVariable b = new DoubleYoVariable("b_arm", registry);
      DoubleYoVariable c = new DoubleYoVariable("c_arm", registry);
      DoubleYoVariable f = new DoubleYoVariable("f_arm", registry); //variable will not be added to list

      list.addVariable(a);
      list.addVariable(b);
      list.addVariable(c);

      String[] names = new String[2];
      names[0] = "a_arm";
      names[1] = "b_arm";

      List<YoVariable<?>> matchedName = list.getMatchingVariables(names, null);

      assertTrue(matchedName.contains(a));
      assertTrue(matchedName.contains(b));
      assertFalse(matchedName.contains(c));
      assertFalse(matchedName.contains(f));

      String[] regularExpressions = { ".*" };

      List<YoVariable<?>> matchedAll = list.getMatchingVariables(null, regularExpressions);

      assertTrue(matchedAll.contains(a));
      assertTrue(matchedAll.contains(b));
      assertTrue(matchedAll.contains(c));
      assertFalse(matchedAll.contains(f));

      String regexpStartWithC[] = { "c.*" };
      List<YoVariable<?>> matchedStartWithC = list.getMatchingVariables(new String[0], regexpStartWithC);

      assertFalse(matchedStartWithC.contains(a));
      assertFalse(matchedStartWithC.contains(b));
      assertTrue(matchedStartWithC.contains(c));
      assertFalse(matchedStartWithC.contains(f));

      List<YoVariable<?>> namesOrStartWithC = list.getMatchingVariables(names, regexpStartWithC);

      assertTrue(namesOrStartWithC.contains(a));
      assertTrue(namesOrStartWithC.contains(b));
      assertTrue(namesOrStartWithC.contains(c));
      assertFalse(namesOrStartWithC.contains(f));
      
      // Return empty list when none match.
      String[] namesThatAreNotInList = new String[2];
      namesThatAreNotInList[0] = "foo";
      namesThatAreNotInList[1] = "bar";
      ArrayList<YoVariable<?>> matchedNameShouldBeEmpty = list.getMatchingVariables(namesThatAreNotInList, null);
      assertTrue(matchedNameShouldBeEmpty.isEmpty());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAddChangeListener()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoVariableList list = new YoVariableList("list");
            
      ChangeListenerForTest listener = new ChangeListenerForTest();
      list.addChangeListener(listener);

      assertFalse(listener.getWasCalled());
      
      BooleanYoVariable variable = new BooleanYoVariable("testBoolean", registry);
      list.addVariable(variable);

      assertTrue(listener.getWasCalled());
   }
   
   private class ChangeListenerForTest implements ChangeListener
   {
      private boolean wasCalled = false;
      
      public void stateChanged(ChangeEvent e)
      {
         this.wasCalled = true;
      }
      
      public boolean getWasCalled()
      {
         return wasCalled;
      }
      
   }
}
