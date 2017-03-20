package us.ihmc.robotics.dataStructures.registry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.listener.YoVariableRegistryChangedListener;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariableList;


public class YoVariableRegistryTest
{
   private static final int N_VARS_IN_ROOT = 4;
   private YoVariableRegistry robotRegistry = null;
   private YoVariableRegistry controllerRegistry = null;
   private YoVariableRegistry testRegistry = null;

   private YoVariableRegistryChangedListener listener = null;

   private YoVariable<?> lastRegisteredVariable = null;
   private YoVariableRegistry lastAddedRegistry = null;
   private YoVariableRegistry lastClearedRegistry = null;
   
   @Before
   public void setUp() 
   {
      robotRegistry = new YoVariableRegistry("robot");
      controllerRegistry = new YoVariableRegistry("controller");
      testRegistry = new YoVariableRegistry("testRegistry");
      
      robotRegistry.addChild(controllerRegistry);
      controllerRegistry.addChild(testRegistry);
      
//      yoVariableRegistry = new YoVariableRegistry("robot.controller.testRegistry");

      new DoubleYoVariable("robotVariable", robotRegistry);
      new DoubleYoVariable("controlVariable", controllerRegistry);
      
      createAndAddNYoVariables(N_VARS_IN_ROOT, testRegistry);
      
      listener = new YoVariableRegistryChangedListener()
      {
         public void yoVariableWasRegistered(YoVariableRegistry registry, YoVariable<?> variable)
         {            
            lastRegisteredVariable = variable;
         }
         
         public void yoVariableRegistryWasCleared(YoVariableRegistry yoVariableRegistry)
         {        
            lastClearedRegistry = yoVariableRegistry;
         }

         public void yoVariableRegistryWasAdded(YoVariableRegistry addedYoVariableRegistry)
         {          
            lastAddedRegistry = addedYoVariableRegistry;
         }
      };
   }

   private void createAndAddNYoVariables(int numberVariablesToAdd, YoVariableRegistry registry)
   {
      if (numberVariablesToAdd >= 1) new DoubleYoVariable("variableOne", registry);
      if (numberVariablesToAdd >= 2) new DoubleYoVariable("variableTwo", registry);
      if (numberVariablesToAdd >= 3) new DoubleYoVariable("variableThree", registry);
      if (numberVariablesToAdd >= 4) new DoubleYoVariable("variableFour", registry);
   }

   @After
   public void tearDown() 
   {
      robotRegistry = null;
      controllerRegistry = null;
      testRegistry = null;
      
      listener = null;
      
      lastRegisteredVariable = null;
      lastAddedRegistry = null;
      lastClearedRegistry = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000,expected = RuntimeException.class)
   public void testCantAddChildWithSameName()
   {
      String name = "sameName";
      YoVariableRegistry child1 = new YoVariableRegistry(name);
      YoVariableRegistry child2 = new YoVariableRegistry(name);
      
      testRegistry.addChild(child1);
      testRegistry.addChild(child2);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetName()
   {      
      assertEquals("robot", robotRegistry.getName());
      assertEquals("controller", controllerRegistry.getName());
      assertEquals("testRegistry", testRegistry.getName());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCreateVarList()
   {
      YoVariableList varList = testRegistry.createVarList();
      assertTrue(varList.size() == 4);
      assertTrue(varList.getName() == testRegistry.getNameSpace().getName());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetAllVariables()
   {
      ArrayList<YoVariable<?>> allVars = testRegistry.getAllVariablesIncludingDescendants();
      assertTrue(allVars.size() == 4);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetNameSpace()
   {
      NameSpace expectedReturn = new NameSpace("robot.controller.testRegistry");
      NameSpace actualReturn = testRegistry.getNameSpace();
      assertEquals("return value", expectedReturn, actualReturn);
   }
   
   @SuppressWarnings("deprecation")

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVariable()
   {
      YoVariable<?> variableOne = testRegistry.getVariable("variableOne");
      YoVariable<?> variableTwo = testRegistry.getVariable("variableTwo");
      YoVariable<?> variableThree = testRegistry.getVariable("variableThree");
      YoVariable<?> variableFour = testRegistry.getVariable("variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      YoVariable<?> doesntExist = testRegistry.getVariable("fooy");
      assertTrue(doesntExist == null);

      variableOne = testRegistry.getVariable("robot.controller.testRegistry.variableOne");
      variableTwo = testRegistry.getVariable("robot.controller.testRegistry.variableTwo");
      variableThree = testRegistry.getVariable("robot.controller.testRegistry.variableThree");
      variableFour = testRegistry.getVariable("robot.controller.testRegistry.variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      variableOne = testRegistry.getVariable("testRegistry.variableOne");
      variableTwo = testRegistry.getVariable("controller.testRegistry.variableTwo");
      variableThree = testRegistry.getVariable("testRegistry.variableThree");
      variableFour = testRegistry.getVariable("controller.testRegistry.variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));


      variableOne = testRegistry.getVariable("robot.controller.variableOne");
      variableTwo = testRegistry.getVariable("robot.testRegistry.variableTwo");
      variableThree = testRegistry.getVariable("bot.controller.testRegistry.variableThree");
      variableFour = testRegistry.getVariable("robot.controller.testRegis.variableFour");

      assertNull(variableOne);
      assertNull(variableTwo);
      assertNull(variableThree);
      assertNull(variableFour);
   }
   
   
   @SuppressWarnings("deprecation")

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCaseInsensitivityToNameButNotNamespace()
   {
      YoVariable<?> variableOne = testRegistry.getVariable("variableone");
      YoVariable<?> variableTwo = testRegistry.getVariable("variableTWO");
      YoVariable<?> variableThree = testRegistry.getVariable("VAriableThree");
      YoVariable<?> variableFour = testRegistry.getVariable("variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      variableOne = testRegistry.getVariable("robot.controller.testRegistry.variableONE");
      variableTwo = testRegistry.getVariable("robot.controller.testRegistry.variableTwo");
      variableThree = testRegistry.getVariable("robot.controller.testRegistry.variableTHREe");
      variableFour = testRegistry.getVariable("robot.controller.testRegistry.VAriableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      variableOne = testRegistry.getVariable("testRegistry.variableONE");
      variableTwo = testRegistry.getVariable("controller.testRegistry.variableTWO");
      variableThree = testRegistry.getVariable("testRegistry.variableThREE");
      variableFour = testRegistry.getVariable("controller.testRegistry.VAriableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      variableOne = testRegistry.getVariable("Robot.controller.testRegistry.variableOne");
      variableTwo = testRegistry.getVariable("robot.coNtroller.testRegistry.variableTwo");
      variableThree = testRegistry.getVariable("robot.controller.TestRegistry.variableThree");
      variableFour = testRegistry.getVariable("robot.controller.testRegistrY.variableFour");

      assertNull(variableOne);
      assertNull(variableTwo);
      assertNull(variableThree);
      assertNull(variableFour);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVariable1()
   {
      String nameSpace = "robot.controller.testRegistry";

      YoVariable<?> variableOne = testRegistry.getVariable(nameSpace, "variableOne");
      YoVariable<?> variableTwo = testRegistry.getVariable(nameSpace, "variableTwo");
      YoVariable<?> variableThree = testRegistry.getVariable(nameSpace, "variableThree");
      YoVariable<?> variableFour = testRegistry.getVariable(nameSpace, "variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      nameSpace = "controller.testRegistry";

      variableOne = testRegistry.getVariable(nameSpace, "variableOne");
      variableTwo = testRegistry.getVariable(nameSpace, "variableTwo");
      variableThree = testRegistry.getVariable(nameSpace, "variableThree");
      variableFour = testRegistry.getVariable(nameSpace, "variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      nameSpace = "testRegistry";

      variableOne = testRegistry.getVariable(nameSpace, "variableOne");
      variableTwo = testRegistry.getVariable(nameSpace, "variableTwo");
      variableThree = testRegistry.getVariable(nameSpace, "variableThree");
      variableFour = testRegistry.getVariable(nameSpace, "variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));


      nameSpace = ".testRegistry";

      variableOne = testRegistry.getVariable(nameSpace, "variableOne");
      variableTwo = testRegistry.getVariable(nameSpace, "variableTwo");
      variableThree = testRegistry.getVariable(nameSpace, "variableThree");
      variableFour = testRegistry.getVariable(nameSpace, "variableFour");

      assertNull(variableOne);
      assertNull(variableTwo);
      assertNull(variableThree);
      assertNull(variableFour);

      boolean testPassed = true;
      try
      {
         testRegistry.getVariable(nameSpace, "foo.variableOne");
         testPassed = false;
      }
      catch (RuntimeException e)
      {
      }

      assertTrue(testPassed);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVariables1()
   {
      ArrayList<YoVariable<?>> variables = testRegistry.getVariables("variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getVariables("variableTwo");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getVariables("variableThree");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getVariables("variableFour");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getVariables("variable");
      assertTrue(variables.size() == 0);

      variables = testRegistry.getVariables("robot.controller.testRegistry.variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getVariables("controller.testRegistry.variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getVariables("testRegistry.variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getVariables("estRegistry.variableOne");
      assertTrue(variables.size() == 0);

      variables = testRegistry.getVariables("foo.robot.controller.testRegistry.variableOne");
      assertTrue(variables.size() == 0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVariables2()
   {
      ArrayList<YoVariable<?>> variables = testRegistry.getVariables("robot.controller.testRegistry", "variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getVariables("robot.controller.testRegistry", "variableTwo");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getVariables("robot.controller.testRegistry", "variableThree");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getVariables("robot.controller.testRegistry", "variableFour");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getVariables("robot.controller.testRegistry", "variable");
      assertTrue(variables.size() == 0);

      variables = testRegistry.getVariables("controller.testRegistry", "variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getVariables("testRegistry", "variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getVariables("estRegistry", "variableOne");
      assertTrue(variables.size() == 0);

      variables = testRegistry.getVariables("foo.robot.controller.testRegistry", "variableOne");
      assertTrue(variables.size() == 0);

      boolean testPassed = true;
      try
      {
         variables = testRegistry.getVariables("robot.controller.testRegistry", "robot.controller.testRegistry.variableOne");
         testPassed = false;
      }
      catch (Exception e)
      {
      }

      assertTrue(testPassed);

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testHasUniqueVariable()
   {
      String name = "";
      boolean expectedReturn = false;
      boolean actualReturn = testRegistry.hasUniqueVariable(name);
      assertEquals("return value", expectedReturn, actualReturn);

      assertTrue(testRegistry.hasUniqueVariable("variableOne"));
      assertFalse(testRegistry.hasUniqueVariable("dontHaveMeVariable"));
      
      assertTrue(testRegistry.hasUniqueVariable("robot.controller.testRegistry", "variableTwo"));
      assertTrue(testRegistry.hasUniqueVariable("controller.testRegistry", "variableTwo"));
      assertFalse(testRegistry.hasUniqueVariable("robot.controller", "variableTwo"));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testHasUniqueVariable1()
   {
      String nameSpace = "";
      String name = "";
      boolean expectedReturn = false;
      boolean actualReturn = testRegistry.hasUniqueVariable(nameSpace, name);
      assertEquals("return value", expectedReturn, actualReturn);

      /** @todo fill in the test code */
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testRegisterVariable()
   {
      boolean testPassed = true;
      try
      {
         testRegistry.registerVariable(null);
         testPassed = false;
      }
      catch (NullPointerException e)
      {
      }

      assertTrue(testPassed);

      testRegistry.registerVariable(new DoubleYoVariable("variableFive", null));

      assertTrue(testRegistry.hasUniqueVariable("variableFive"));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000,expected = RuntimeException.class)
   public void testCannotRegisterSameVariableName()
   {
      DoubleYoVariable variableFiveOnce = new DoubleYoVariable("variableFive", null);
      DoubleYoVariable variableFiveTwice = new DoubleYoVariable("variableFive", null);

      testRegistry.registerVariable(variableFiveOnce);
      assertTrue(testRegistry.hasUniqueVariable("variableFive"));
      testRegistry.registerVariable(variableFiveTwice);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetAllVariablesInThisListOnly()
   {
      ArrayList<YoVariable<?>> robotVariablesOnly = robotRegistry.getAllVariablesInThisListOnly();
      ArrayList<YoVariable<?>> controlVariablesOnly = controllerRegistry.getAllVariablesInThisListOnly();
      ArrayList<YoVariable<?>> testRegistryVariablesOnly = testRegistry.getAllVariablesInThisListOnly();
      
      assertEquals(1, robotVariablesOnly.size());
      assertEquals(1, controlVariablesOnly.size());
      assertEquals(4, testRegistryVariablesOnly.size());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAddChildAndGetParentAndGetChildren()
   {
      assertEquals(robotRegistry.getParent(), null);
      
      YoVariableRegistry childOne = new YoVariableRegistry("childOne");
      assertEquals(childOne.getParent(), null);

      testRegistry.addChild(childOne);
      
      boolean testPassed = true;
      try
      {
         YoVariableRegistry childOneRepeat = new YoVariableRegistry("childOne");

         testRegistry.addChild(childOneRepeat);
         testPassed = false;
      }
      catch (Exception e)
      {
      }

      assertTrue(testPassed);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000,expected = RuntimeException.class)
   public void testDontLetAChildGetAddedToTwoRegistries()
   {
      YoVariableRegistry root1 = new YoVariableRegistry("root1");
      YoVariableRegistry root2 = new YoVariableRegistry("root2");
      
      YoVariableRegistry child = new YoVariableRegistry("child");
      root1.addChild(child);
      root2.addChild(child);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000,expected = RuntimeException.class)
   public void testIllegalName1()
   {
      testRegistry = new YoVariableRegistry("foo..foo");
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000,expected = RuntimeException.class)
   public void testIllegalName2()
   {
      testRegistry = new YoVariableRegistry("foo.");
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000,expected = RuntimeException.class)
   public void testNoDotsAllowed()
   {
      testRegistry = new YoVariableRegistry("foo.bar");
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000,expected = RuntimeException.class)
   public void testIllegalAddChild()
   {
      YoVariableRegistry childOne = new YoVariableRegistry("childOne");
      childOne.addChild(childOne);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetAllVariablesIncludingDescendants()
   {
      YoVariableRegistry childOne = new YoVariableRegistry("childOne");
      int nVarsChildOne = 3;
      createAndAddNYoVariables(nVarsChildOne, childOne);

      YoVariableRegistry childTwo = new YoVariableRegistry("childTwo");
      int nVarsChildTwo = 2;
      createAndAddNYoVariables(nVarsChildTwo, childTwo);

      testRegistry.addChild(childOne);
      testRegistry.addChild(childTwo);

      int nVarsExpected = nVarsChildOne + nVarsChildTwo + N_VARS_IN_ROOT;
      
      ArrayList<YoVariable<?>> allVariables = testRegistry.getAllVariablesIncludingDescendants();
      assertEquals(nVarsExpected, allVariables.size());
      YoVariable<?>[] allVariablesArray = testRegistry.getAllVariablesArray();
      assertEquals(nVarsExpected, allVariablesArray.length);

      assertEquals(nVarsChildTwo, childTwo.getAllVariablesIncludingDescendants().size());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testFamilyRelations()
   {
      YoVariableRegistry childOne = new YoVariableRegistry("childOne");
      YoVariableRegistry childTwo = new YoVariableRegistry("childTwo");
      
      testRegistry.addChild(childOne);
      testRegistry.addChild(childTwo);

      assertEquals(childOne.getParent(), testRegistry);
      assertEquals(childTwo.getParent(), testRegistry);

      ArrayList<YoVariableRegistry> children = testRegistry.getChildren();

      int childrenSize = children.size();

      assertEquals(2, childrenSize);
      assertTrue(children.contains(childOne));
      assertTrue(children.contains(childTwo));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000,expected = RuntimeException.class)
   public void testCantAddDuplicateSubnames()
   {
      YoVariableRegistry childOne = new YoVariableRegistry("childOne");
      testRegistry.addChild(childOne);

      YoVariableRegistry grandChildOne = new YoVariableRegistry(childOne.getParent().getNameSpace().getRootName());
      childOne.addChild(grandChildOne);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testNullChild()
   {

      YoVariableRegistry nullChild = null;
      YoVariableRegistry testNullChild = new YoVariableRegistry("TestNullChild");

      testNullChild.addChild(nullChild);
      assertEquals(0, testNullChild.getChildren().size());
   }

   @SuppressWarnings("deprecation")

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testRegistryTree()
   {
      YoVariableRegistry root = new YoVariableRegistry("root");
      
      YoVariableRegistry registry0 = new YoVariableRegistry("registry0");
      assertEquals("registry0", registry0.getNameSpace().getName());
      YoVariableRegistry registry1 = new YoVariableRegistry("registry1");
      YoVariableRegistry registry2 = new YoVariableRegistry("registry2");
      root.addChild(registry0);
      assertEquals("root.registry0", registry0.getNameSpace().getName());
      assertEquals("registry0", registry0.getNameSpace().getShortName());
      root.addChild(registry2);
      
      YoVariableRegistry registry00 = new YoVariableRegistry("registry00");
      YoVariableRegistry registry01 = new YoVariableRegistry("registry01");
      registry0.addChild(registry00);
      registry0.addChild(registry01);
      
      YoVariableRegistry registry10 = new YoVariableRegistry("registry10");
      registry1.addChild(registry10);
      
      YoVariableRegistry registry010 = new YoVariableRegistry("registry010");
      YoVariableRegistry registry011 = new YoVariableRegistry("registry011");
      registry01.addChild(registry010);
      
      DoubleYoVariable variable0_A = new DoubleYoVariable("variable0_A", registry0);
      DoubleYoVariable variable0_B = new DoubleYoVariable("variable0_B", registry0);
      DoubleYoVariable variable10_A = new DoubleYoVariable("variable10_A", registry10);
      DoubleYoVariable variable011_A = new DoubleYoVariable("variable011_A", registry011);
      
      DoubleYoVariable repeatedVariable_root = new DoubleYoVariable("repeatedVariable", root);
      DoubleYoVariable repeatedVariable_registry0 = new DoubleYoVariable("repeatedVariable", registry0);
      DoubleYoVariable repeatedVariable_registry01 = new DoubleYoVariable("repeatedVariable", registry01);
      DoubleYoVariable repeatedVariable_registry010 = new DoubleYoVariable("repeatedVariable", registry010);

      
      // Do some of the addChilds out of order to make sure they work correctly when done out of order.
      root.addChild(registry1);
      registry01.addChild(registry011);
      assertEquals("root.registry0.registry01.registry011", registry011.getNameSpace().getName());

      assertEquals("root.registry0.variable0_A", variable0_A.getFullNameWithNameSpace());
      assertEquals("root.registry0.variable0_B", variable0_B.getFullNameWithNameSpace());
      assertEquals("root.registry1.registry10.variable10_A", variable10_A.getFullNameWithNameSpace());
      assertEquals("root.registry0.registry01.registry011.variable011_A", variable011_A.getFullNameWithNameSpace());
      
      ArrayList<YoVariable<?>> allRootVariables = root.getAllVariablesIncludingDescendants();
      assertEquals(8, allRootVariables.size());
      
      assertTrue(registry10.hasUniqueVariable("root.registry1.registry10.variable10_A"));
      
      assertEquals(variable10_A, registry10.getVariable("root.registry1.registry10.variable10_A"));
      assertEquals(variable10_A, registry10.getVariable("registry10.variable10_A"));
      assertEquals(variable10_A, registry10.getVariable("variable10_A"));
      
      assertTrue(root.hasUniqueVariable("root.registry1.registry10.variable10_A"));
      assertTrue(root.hasUniqueVariable("registry1.registry10.variable10_A"));
      assertTrue(root.hasUniqueVariable("registry10.variable10_A"));
      assertTrue(root.hasUniqueVariable("variable10_A"));

      assertFalse(root.hasUniqueVariable("repeatedVariable"));
      assertFalse(registry0.hasUniqueVariable("repeatedVariable"));
      assertFalse(registry01.hasUniqueVariable("repeatedVariable"));
      assertTrue(registry010.hasUniqueVariable("repeatedVariable"));

      assertTrue(root.hasUniqueVariable("registry0.repeatedVariable"));
      assertTrue(root.hasUniqueVariable("registry0", "repeatedVariable"));
      assertFalse(root.hasUniqueVariable("registry0.noWay"));
      assertFalse(root.hasUniqueVariable("noWay.repeatedVariable"));
      assertFalse(root.hasUniqueVariable("noWay", "repeatedVariable"));

      assertEquals(variable10_A, registry1.getVariable("variable10_A"));
      assertEquals(variable10_A, root.getVariable("variable10_A"));
      assertEquals(variable011_A, root.getVariable("variable011_A"));
      assertEquals(variable011_A, registry0.getVariable("variable011_A"));
      assertEquals(variable011_A, registry01.getVariable("variable011_A"));
      assertEquals(variable011_A, registry011.getVariable("variable011_A"));

      assertEquals(repeatedVariable_root, root.getVariable("repeatedVariable"));
      assertEquals(repeatedVariable_registry0, registry0.getVariable("repeatedVariable"));
      assertEquals(repeatedVariable_registry01, registry01.getVariable("repeatedVariable"));
      assertEquals(repeatedVariable_registry010, registry010.getVariable("repeatedVariable"));

      assertEquals(4, root.getVariables("repeatedVariable").size());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000,expected = RuntimeException.class)
   public void testDontAllowRepeatRegistryNames()
   {
      YoVariableRegistry root = new YoVariableRegistry("root");

      YoVariableRegistry levelOne = new YoVariableRegistry("levelOne");

      YoVariableRegistry registryOne = new YoVariableRegistry("registryOne");
      YoVariableRegistry registryOneRepeat = new YoVariableRegistry("registryOne");

//      DoubleYoVariable variableOne = new DoubleYoVariable("variableOne", registryOne);
////      DoubleYoVariable variableOneRepeat = new DoubleYoVariable("variableOne", registryOneRepeat);
//      DoubleYoVariable variableTwo = new DoubleYoVariable("variableTwo", registryOneRepeat);
      
      root.addChild(levelOne);
      
      levelOne.addChild(registryOne);
      levelOne.addChild(registryOneRepeat);
   }
   
   
   @SuppressWarnings("deprecation")

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetOrCreateAndAddRegistry()
   {
      YoVariableRegistry root = new YoVariableRegistry("root");
      
      YoVariableRegistry registry000 = root.getOrCreateAndAddRegistry(new NameSpace("root.registry0.registry00.registry000"));
      NameSpace nameSpaceCheck = registry000.getNameSpace();
      assertEquals(new NameSpace("root.registry0.registry00.registry000"), nameSpaceCheck);
      
      DoubleYoVariable foo = new DoubleYoVariable("foo", registry000);
      assertEquals("root.registry0.registry00.registry000.foo", foo.getFullNameWithNameSpace());
      
      YoVariableRegistry registry010 = root.getOrCreateAndAddRegistry(new NameSpace("root.registry0.registry01.registry010"));
      DoubleYoVariable bar = new DoubleYoVariable("bar", registry010);
      assertEquals("root.registry0.registry01.registry010.bar", bar.getFullNameWithNameSpace());
      
      assertEquals(foo, root.getVariable("foo"));
      assertEquals(bar, root.getVariable("bar"));
      
      
      assertEquals(registry000, root.getOrCreateAndAddRegistry(new NameSpace("root.registry0.registry00.registry000")));
      assertEquals(registry010, root.getOrCreateAndAddRegistry(new NameSpace("root.registry0.registry01.registry010")));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testNullNameSpace()
   {
      YoVariableRegistry root = new YoVariableRegistry("");
      assertEquals(null, root.getNameSpace());

      YoVariableRegistry registry000 = root.getOrCreateAndAddRegistry(new NameSpace("root.registry0.registry00.registry000"));
      NameSpace nameSpaceCheck = registry000.getNameSpace();
      assertEquals(new NameSpace("root.registry0.registry00.registry000"), nameSpaceCheck);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000,expected = RuntimeException.class)
   public void testCantAddAChildWithANullNamespace()
   {
      YoVariableRegistry root = new YoVariableRegistry("root");
      YoVariableRegistry child = new YoVariableRegistry("");
      root.addChild(child);

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCreateVarListIncludingChildren()
   {
      ArrayList<YoVariableList> varLists = robotRegistry.createVarListsIncludingChildren();
      assertEquals(3, varLists.size());
            
      assertContainsListWithNameAndVariables(varLists, "robot", 1);
      assertContainsListWithNameAndVariables(varLists, "robot.controller", 1);
      assertContainsListWithNameAndVariables(varLists, "robot.controller.testRegistry", 4);
      
   }
   
   private void assertContainsListWithNameAndVariables(ArrayList<YoVariableList> varLists, String name, int numVariables)
   {
      int containsName = 0;
      
      for (YoVariableList varList : varLists)
      {
         if (varList.getName().equals(name))
         {
            containsName++;
            assertEquals(numVariables, varList.getVariables().size());
         }
      }
      
      assertEquals(1, containsName);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetAllRegistriesIncludingChildren()
   {
      ArrayList<YoVariableRegistry> registries = robotRegistry.getAllRegistriesIncludingChildren();
      
      assertEquals(3, registries.size());
      assertTrue(registries.contains(robotRegistry));
      assertTrue(registries.contains(controllerRegistry));
      assertTrue(registries.contains(testRegistry));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000) 
   public void testGetRegistry()
   {
      assertEquals(robotRegistry, robotRegistry.getRegistry(new NameSpace("robot")));
      assertEquals(controllerRegistry, robotRegistry.getRegistry(new NameSpace("robot.controller")));
      assertEquals(testRegistry, robotRegistry.getRegistry(new NameSpace("robot.controller.testRegistry")));
      
      assertEquals(controllerRegistry, controllerRegistry.getRegistry(new NameSpace("robot.controller")));
      assertEquals(testRegistry, controllerRegistry.getRegistry(new NameSpace("robot.controller.testRegistry")));

      assertEquals(testRegistry, testRegistry.getRegistry(new NameSpace("robot.controller.testRegistry")));

      assertTrue(testRegistry != robotRegistry.getRegistry(new NameSpace("testRegistry")));
      assertTrue(testRegistry != robotRegistry.getRegistry(new NameSpace("controller.testRegistry")));
      
      assertTrue(robotRegistry != controllerRegistry.getRegistry(new NameSpace("robot")));

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testLoggingAndSending()
   {
      robotRegistry.setLoggingIncludingDescendants(false);
      robotRegistry.setSendingIncludingDescendants(false);
      
      assertFalse(robotRegistry.isLogged());
      assertFalse(controllerRegistry.isLogged());
      assertFalse(testRegistry.isLogged());
      
      assertFalse(robotRegistry.isSent());
      assertFalse(controllerRegistry.isSent());
      assertFalse(testRegistry.isSent());
      
      controllerRegistry.setLogging(true);
      assertFalse(robotRegistry.isLogged());
      assertTrue(controllerRegistry.isLogged());
      assertFalse(testRegistry.isLogged());
      
      assertFalse(robotRegistry.isSent());
      assertFalse(controllerRegistry.isSent());
      assertFalse(testRegistry.isSent());
      
      controllerRegistry.setSendingIncludingDescendants(true);
      assertFalse(robotRegistry.isLogged());
      assertTrue(controllerRegistry.isLogged());
      assertFalse(testRegistry.isLogged());
      
      assertFalse(robotRegistry.isSent());
      assertTrue(controllerRegistry.isSent());
      assertTrue(testRegistry.isSent());
      
      robotRegistry.setLoggingIncludingDescendants(false);
      robotRegistry.setSendingIncludingDescendants(true);
      
      assertFalse(robotRegistry.isLogged());
      assertFalse(controllerRegistry.isLogged());
      assertFalse(testRegistry.isLogged());
      
      assertTrue(robotRegistry.isSent());
      assertTrue(controllerRegistry.isSent());
      assertTrue(testRegistry.isSent());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetNumberOfVariables()
   {
      assertEquals(1, robotRegistry.getNumberOfYoVariables());
      assertEquals(1, controllerRegistry.getNumberOfYoVariables());
      assertEquals(4, testRegistry.getNumberOfYoVariables());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testListenersOne()
   {
      this.robotRegistry.attachYoVariableRegistryChangedListener(listener);
      
      assertNull(lastRegisteredVariable);
      DoubleYoVariable addedYoVariable = new DoubleYoVariable("addedLater", controllerRegistry);
      assertEquals(addedYoVariable, lastRegisteredVariable);
      
      assertNull(lastAddedRegistry);
      YoVariableRegistry addedRegistry = new YoVariableRegistry("addedRegistry");
      testRegistry.addChild(addedRegistry);
      assertEquals(addedRegistry, lastAddedRegistry);

      assertNull(lastClearedRegistry);
      testRegistry.clear();
      assertEquals(testRegistry, lastClearedRegistry);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000,expected = RuntimeException.class)
   public void testThrowExceptionIfAttachListenerToNonRoot()
   {
      this.controllerRegistry.attachYoVariableRegistryChangedListener(listener);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testThrowExceptionIfAddChildRegistryWithAListener()
   {
      YoVariableRegistry childRegistry = new YoVariableRegistry("childToAdd");
      childRegistry.attachYoVariableRegistryChangedListener(listener);
      
      try
      {
         this.controllerRegistry.addChild(childRegistry);
         fail("Should not get here!");
      }
      catch(RuntimeException runtimeException)
      {
      }
   }
   
}
