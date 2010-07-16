package com.yobotics.simulationconstructionset;

import java.util.ArrayList;

import junit.framework.TestCase;

/**
 * <p>Title: SimulationConstructionSet</p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2000</p>
 *
 * <p>Company: Yobotics, Inc.</p>
 *
 * @author not attributable
 * @version 1.0
 */
public class TestYoVariableHolderImplementation extends TestCase
{
   private YoVariableHolderImplementation yoVariableHolderImplementation = null;

   public TestYoVariableHolderImplementation(String name)
   {
      super(name);
   }

   protected void setUp() throws Exception
   {
      super.setUp();
      yoVariableHolderImplementation = new YoVariableHolderImplementation();

      YoVariableRegistry registryA = new YoVariableRegistry("robot.registryA");
      YoVariableRegistry registryB = new YoVariableRegistry("robot.registryB");
      YoVariableRegistry registryC = new YoVariableRegistry("robot.registryC");
      YoVariableRegistry registryC2 = new YoVariableRegistry("robot2.registryC");

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

   protected void tearDown() throws Exception
   {
      yoVariableHolderImplementation = null;
      super.tearDown();
   }

   public void testAddVariableToHolder()
   {
   }

   public void testGetVariable()
   {
      boolean testPass = true;
      try
      {
         yoVariableHolderImplementation.getVariable("variableOne");
         testPass = false;
      }
      catch (RuntimeException e)
      {
      }

      assert testPass;

      AbstractYoVariable variable = yoVariableHolderImplementation.getVariable("robot.registryA.variableOne");
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

   public void testGetVariable1()
   {
	   AbstractYoVariable variable = yoVariableHolderImplementation.getVariable("robot.registryA", "variableOne");
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
      }
      catch (Exception e)
      {
      }

      assert testPassed;


      try
      {
         variable = yoVariableHolderImplementation.getVariable("registryC", "variableTwo");
         testPassed = false;
      }
      catch (Exception e)
      {
      }

      assert testPassed;
   }

   public void testGetVariables()
   {
      NameSpace nameSpace = new NameSpace("robot.registryA");
      ArrayList<AbstractYoVariable> variables = yoVariableHolderImplementation.getVariables(nameSpace);
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

   public void testGetVariables1()
   {
      ArrayList<AbstractYoVariable> variables = yoVariableHolderImplementation.getVariables("variableOne");
      boolean aFound = false, bFound = false, cFound = false, c2Found = false;

      for (AbstractYoVariable variable : variables)
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

      assert(aFound && bFound && cFound && c2Found);
      assertEquals(4, variables.size());

      variables = yoVariableHolderImplementation.getVariables("variableTwo");
      assertEquals(4, variables.size());

      variables = yoVariableHolderImplementation.getVariables("variableThree");
      assertEquals(4, variables.size());

      variables = yoVariableHolderImplementation.getVariables("var");
      assertEquals(0, variables.size());
   }

   public void testGetVariables2()
   {
      ArrayList<AbstractYoVariable> variables = yoVariableHolderImplementation.getVariables("robot.registryA", "variableOne");
      assertEquals(1, variables.size());

      variables = yoVariableHolderImplementation.getVariables("robot", "variableOne");
      assertEquals(0, variables.size());

      variables = yoVariableHolderImplementation.getVariables("registryC", "variableOne");
      assertEquals(2, variables.size());
      boolean cFound = false, c2Found = false, testPassed = true;

      for (AbstractYoVariable variable : variables)
      {
         if (variable.getFullNameWithNameSpace().equals("robot.registryC.variableOne"))
            cFound = true;
         if (variable.getFullNameWithNameSpace().equals("robot2.registryC.variableOne"))
            c2Found = true;
      }

      assert(cFound && c2Found);

      try
      {
         variables = yoVariableHolderImplementation.getVariables("robot", "registryC.variableOne");
         testPassed = false;
      }
      catch (Exception e)
      {
      }

      assert testPassed;

   }

   public void testHasUniqueVariable()
   {
      assert !yoVariableHolderImplementation.hasUniqueVariable("variableOne");

      assert yoVariableHolderImplementation.hasUniqueVariable("robot.registryA.variableOne");

      assert yoVariableHolderImplementation.hasUniqueVariable("registryA.variableOne");

      assert yoVariableHolderImplementation.hasUniqueVariable("robot.registryA.variableOne");

      assert !yoVariableHolderImplementation.hasUniqueVariable("istryA.variableOne");

      assert yoVariableHolderImplementation.hasUniqueVariable("robot.registryA.variableTwo");

      assert !yoVariableHolderImplementation.hasUniqueVariable("registryC.variableTwo");
   }

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
      }
      catch (Exception e)
      {
      }

      assert testPassed;

   }

   public void testYoVariableHolderImplementation()
   {
      // Already tested
   }

}
