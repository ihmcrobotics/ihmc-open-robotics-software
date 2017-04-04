package us.ihmc.simulationconstructionset;

import static org.junit.Assert.*;

import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.NameSpace;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

public class YoVariableHolderImplementationTest
{
   private YoVariableHolderImplementation yoVariableHolderImplementation = null;

   public YoVariableHolderImplementationTest()
   {
   }

   @Before
   public void setUp()
   {
      yoVariableHolderImplementation = new YoVariableHolderImplementation();

      YoVariableRegistry robotRegistry = new YoVariableRegistry("robot");
      YoVariableRegistry robot2Registry = new YoVariableRegistry("robot2");

      YoVariableRegistry registryA = new YoVariableRegistry("registryA");
      YoVariableRegistry registryB = new YoVariableRegistry("registryB");
      YoVariableRegistry registryC = new YoVariableRegistry("registryC");
      robotRegistry.addChild(registryA);
      robotRegistry.addChild(registryB);
      robotRegistry.addChild(registryC);

      YoVariableRegistry registryC2 = new YoVariableRegistry("registryC");
      robot2Registry.addChild(registryC2);

      DoubleYoVariable variableOneA = new DoubleYoVariable("variableOne", registryA);
      DoubleYoVariable variableOneB = new DoubleYoVariable("variableOne", registryB);
      DoubleYoVariable variableOneC = new DoubleYoVariable("variableOne", registryC);
      DoubleYoVariable variableOneC2 = new DoubleYoVariable("variableOne", registryC2);

      DoubleYoVariable variableTwoA = new DoubleYoVariable("variableTwo", registryA);
      DoubleYoVariable variableTwoB = new DoubleYoVariable("variableTwo", registryB);
      DoubleYoVariable variableTwoC = new DoubleYoVariable("variableTwo", registryC);
      DoubleYoVariable variableTwoC2 = new DoubleYoVariable("variableTwo", registryC2);

      DoubleYoVariable variableThreeA = new DoubleYoVariable("variableThree", registryA);
      DoubleYoVariable variableThreeB = new DoubleYoVariable("variableThree", registryB);
      DoubleYoVariable variableThreeC = new DoubleYoVariable("variableThree", registryC);
      DoubleYoVariable variableThreeC2 = new DoubleYoVariable("variableThree", registryC2);

      yoVariableHolderImplementation.addVariableToHolder(variableOneA);
      yoVariableHolderImplementation.addVariableToHolder(variableOneB);
      yoVariableHolderImplementation.addVariableToHolder(variableOneC);
      yoVariableHolderImplementation.addVariableToHolder(variableOneC2);

      yoVariableHolderImplementation.addVariableToHolder(variableTwoA);
      yoVariableHolderImplementation.addVariableToHolder(variableTwoB);
      yoVariableHolderImplementation.addVariableToHolder(variableTwoC);
      yoVariableHolderImplementation.addVariableToHolder(variableTwoC2);

      yoVariableHolderImplementation.addVariableToHolder(variableThreeA);
      yoVariableHolderImplementation.addVariableToHolder(variableThreeB);
      yoVariableHolderImplementation.addVariableToHolder(variableThreeC);
      yoVariableHolderImplementation.addVariableToHolder(variableThreeC2);
   }

   @After
   public void tearDown()
   {
      yoVariableHolderImplementation = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVariable()
   {
      YoVariable<?> variable = yoVariableHolderImplementation.getVariable("robot.registryA.variableOne");
      assertEquals(variable.getName(), "variableOne");

      variable = yoVariableHolderImplementation.getVariable("registryA.variableOne");
      assertEquals(variable.getName(), "variableOne");

      variable = yoVariableHolderImplementation.getVariable("robot.registryA.variableOne");
      assertEquals(variable.getName(), "variableOne");

      variable = yoVariableHolderImplementation.getVariable("istryA.variableOne");
      assertNull(variable);

      variable = yoVariableHolderImplementation.getVariable("robot.registryA.variableTwo");
      assertEquals(variable.getName(), "variableTwo");

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVariable1()
   {
      YoVariable<?> variable = yoVariableHolderImplementation.getVariable("robot.registryA", "variableOne");
      assertEquals(variable.getName(), "variableOne");
      assertEquals(variable.getFullNameWithNameSpace(), "robot.registryA.variableOne");

      variable = yoVariableHolderImplementation.getVariable("robot.registryB", "variableOne");
      assertEquals(variable.getName(), "variableOne");
      assertEquals(variable.getFullNameWithNameSpace(), "robot.registryB.variableOne");

      variable = yoVariableHolderImplementation.getVariable("robot.registryC", "variableOne");
      assertEquals(variable.getName(), "variableOne");
      assertEquals(variable.getFullNameWithNameSpace(), "robot.registryC.variableOne");

      variable = yoVariableHolderImplementation.getVariable("registryA", "variableOne");
      assertEquals(variable.getName(), "variableOne");
      assertEquals(variable.getFullNameWithNameSpace(), "robot.registryA.variableOne");

      variable = yoVariableHolderImplementation.getVariable("registryB", "variableTwo");
      assertEquals(variable.getName(), "variableTwo");
      assertEquals(variable.getFullNameWithNameSpace(), "robot.registryB.variableTwo");

      boolean testPassed = true;

      try
      {
         variable = yoVariableHolderImplementation.getVariable("registryC", "variableOne");
         testPassed = false;
      } catch (Exception e)
      {
      }

      assert testPassed;

      try
      {
         variable = yoVariableHolderImplementation.getVariable("registryC", "variableTwo");
         testPassed = false;
      } catch (Exception e)
      {
      }

      assert testPassed;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVariables()
   {
      NameSpace nameSpace = new NameSpace("robot.registryA");
      ArrayList<YoVariable<?>> variables = yoVariableHolderImplementation.getVariables(nameSpace);
      assertEquals(3, variables.size());

      nameSpace = new NameSpace("robot.registryB");
      variables = yoVariableHolderImplementation.getVariables(nameSpace);
      assertEquals(3, variables.size());

      nameSpace = new NameSpace("robot.registryC");
      variables = yoVariableHolderImplementation.getVariables(nameSpace);
      assertEquals(3, variables.size());

      nameSpace = new NameSpace("robot2.registryC");
      variables = yoVariableHolderImplementation.getVariables(nameSpace);
      assertEquals(3, variables.size());

      nameSpace = new NameSpace("robot");
      variables = yoVariableHolderImplementation.getVariables(nameSpace);
      assertEquals(0, variables.size());

      nameSpace = new NameSpace("registryA");
      variables = yoVariableHolderImplementation.getVariables(nameSpace);
      assertEquals(0, variables.size());

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVariables1()
   {
      ArrayList<YoVariable<?>> variables = yoVariableHolderImplementation.getVariables("variableOne");
      boolean aFound = false, bFound = false, cFound = false, c2Found = false;

      for (YoVariable<?> variable : variables)
      {
         if (variable.getFullNameWithNameSpace().equals("robot.registryA.variableOne"))
            aFound = true;
         if (variable.getFullNameWithNameSpace().equals("robot.registryB.variableOne"))
            bFound = true;
         if (variable.getFullNameWithNameSpace().equals("robot.registryC.variableOne"))
            cFound = true;
         if (variable.getFullNameWithNameSpace().equals("robot2.registryC.variableOne"))
            c2Found = true;
      }

      assert (aFound && bFound && cFound && c2Found);
      assertEquals(4, variables.size());

      variables = yoVariableHolderImplementation.getVariables("variableTwo");
      assertEquals(4, variables.size());

      variables = yoVariableHolderImplementation.getVariables("variableThree");
      assertEquals(4, variables.size());

      variables = yoVariableHolderImplementation.getVariables("var");
      assertEquals(0, variables.size());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVariables2()
   {
      ArrayList<YoVariable<?>> variables = yoVariableHolderImplementation.getVariables("robot.registryA", "variableOne");
      assertEquals(1, variables.size());

      variables = yoVariableHolderImplementation.getVariables("robot", "variableOne");
      assertEquals(0, variables.size());

      variables = yoVariableHolderImplementation.getVariables("registryC", "variableOne");
      assertEquals(2, variables.size());
      boolean cFound = false, c2Found = false, testPassed = true;

      for (YoVariable<?> variable : variables)
      {
         if (variable.getFullNameWithNameSpace().equals("robot.registryC.variableOne"))
            cFound = true;
         if (variable.getFullNameWithNameSpace().equals("robot2.registryC.variableOne"))
            c2Found = true;
      }

      assert (cFound && c2Found);

      try
      {
         variables = yoVariableHolderImplementation.getVariables("robot", "registryC.variableOne");
         testPassed = false;
      } catch (Exception e)
      {
      }

      assert testPassed;

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testHasUniqueVariable()
   {
      
      assertTrue(!yoVariableHolderImplementation.hasUniqueVariable("variableOne"));

      assertTrue(yoVariableHolderImplementation.hasUniqueVariable("robot.registryA.variableOne"));

      assertTrue(yoVariableHolderImplementation.hasUniqueVariable("registryA.variableOne"));

      assertTrue(yoVariableHolderImplementation.hasUniqueVariable("robot.registryA.variableOne"));

      assertTrue(!yoVariableHolderImplementation.hasUniqueVariable("istryA.variableOne"));

      assertTrue(yoVariableHolderImplementation.hasUniqueVariable("robot.registryA.variableTwo"));

      assertTrue(!yoVariableHolderImplementation.hasUniqueVariable("registryC.variableTwo"));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testHasUniqueVariable1()
   {
      assert yoVariableHolderImplementation.hasUniqueVariable("robot.registryA", "variableOne");

      assert yoVariableHolderImplementation.hasUniqueVariable("registryA", "variableOne");

      assert !yoVariableHolderImplementation.hasUniqueVariable("registryC", "variableTwo");

      assert !yoVariableHolderImplementation.hasUniqueVariable("istryA", "variableOne");

      assert yoVariableHolderImplementation.hasUniqueVariable("robot.registryA", "variableTwo");

      assert !yoVariableHolderImplementation.hasUniqueVariable("robot", "variableOne");

      boolean testPassed = true;
      try
      {
         yoVariableHolderImplementation.hasUniqueVariable("robot", "registryC.variableOne");
         testPassed = false;
      } catch (Exception e)
      {
      }

      assert testPassed;

   }

}
