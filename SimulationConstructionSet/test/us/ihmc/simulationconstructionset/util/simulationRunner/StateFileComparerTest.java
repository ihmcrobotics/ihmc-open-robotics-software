package us.ihmc.simulationconstructionset.util.simulationRunner;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;


public class StateFileComparerTest
{

   @Before
   public void setUp() throws Exception
   {
   }

   @After
   public void tearDown() throws Exception
   {
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAbsoluteCompareStateFiles() throws FileNotFoundException
   {
      File fileOne = new File("fileOne.state");
      File fileTwo = new File("fileTwo.state");
      
      if (fileOne.exists()) fileOne.delete();
      if (fileTwo.exists()) fileTwo.delete();
      
      writeStateFileA(fileOne);
      writeStateFileA(fileTwo);
      verifyNoDifferences(fileOne, fileTwo);
      fileOne.delete();
      fileTwo.delete();
      
      writeStateFileB(fileOne);
      writeStateFileB(fileTwo);
      verifyNoDifferences(fileOne, fileTwo);
      fileOne.delete();
      fileTwo.delete();
      
      writeStateFileA(fileOne);
      writeStateFileB(fileTwo);
      verifyDifferences(fileOne, fileTwo);
      fileOne.delete();
      fileTwo.delete();
   }
   
   private void verifyNoDifferences(File fileOne, File fileTwo)
   {
      double maxAbsoluteDiff = 1e-7;
      ArrayList<String> exceptions = new ArrayList<String>();

      ArrayList<VariableDifference> variableDifferences = StateFileComparer.absoluteCompareStateFiles(fileOne, fileTwo, maxAbsoluteDiff, exceptions);
      assertTrue(VariableDifference.allVariableDifferencesToString(variableDifferences), variableDifferences.isEmpty());
   }
   
   private void verifyDifferences(File fileOne, File fileTwo)
   {
      double maxAbsoluteDiff = 1e-7;
      ArrayList<String> exceptions = new ArrayList<String>();

      ArrayList<VariableDifference> variableDifferences = StateFileComparer.absoluteCompareStateFiles(fileOne, fileTwo, maxAbsoluteDiff, exceptions);
      assertFalse(variableDifferences.isEmpty());
   }
   
   private void writeStateFileA(File file) throws FileNotFoundException
   {
      PrintStream stream = new PrintStream(file); 

      stream.println("BipedalRobot.Time.gravityZ = -9.81;");
      stream.println("launchedBalls.Time.gravityZ = -9.8;");
      stream.close();
   }
   
   private void writeStateFileB(File file) throws FileNotFoundException
   {
      PrintStream stream = new PrintStream(file); 

      stream.println("variable1 = 0.1;");
      stream.println("variable2 = 0.2;");
      stream.close();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCompareVarLists()
   {
      YoVariableRegistry registry1 = createRegistryAndFillWithVariables();
      YoVariableRegistry registry2 = createRegistryAndFillWithVariables();
      
      ArrayList<String> exceptions = new ArrayList<String>();
      exceptions.add(new String("exceptional"));
      
      ArrayList<VariableDifference> variableDifferences = StateFileComparer.compareVarLists(registry1.createVarList(), registry2.createVarList(), 1e-7, false, exceptions);
      assertEquals(0, variableDifferences.size());
   
      ((DoubleYoVariable) registry1.getVariable("exceptionalVariable")).set(5678.0);
      
      variableDifferences = StateFileComparer.compareVarLists(registry1.createVarList(), registry2.createVarList(), 1e-7, false, exceptions);
      assertEquals(0, variableDifferences.size());
      
      ((DoubleYoVariable) registry1.getVariable("variable1")).set(3.5);
      
      variableDifferences = StateFileComparer.compareVarLists(registry1.createVarList(), registry2.createVarList(), 1e-7, false, exceptions);
      assertEquals(1, variableDifferences.size());
   }
   
   private YoVariableRegistry createRegistryAndFillWithVariables()
   {
      YoVariableRegistry root = new YoVariableRegistry("root");
      YoVariableRegistry registry0 = new YoVariableRegistry("registry0");
      YoVariableRegistry registry00 = new YoVariableRegistry("registry00");
      YoVariableRegistry registry01 = new YoVariableRegistry("registry01");
      
      root.addChild(registry0);
      registry0.addChild(registry00);
      registry0.addChild(registry01);
      
      DoubleYoVariable variable1 = new DoubleYoVariable("variable1", root);
      DoubleYoVariable variable2 = new DoubleYoVariable("variable2", registry0);
      DoubleYoVariable variable3 = new DoubleYoVariable("variable3", registry00);
      DoubleYoVariable variable4 = new DoubleYoVariable("variable4", registry01);

      BooleanYoVariable boolean1 = new BooleanYoVariable("boolean1", root);
      BooleanYoVariable boolean2 = new BooleanYoVariable("boolean2", registry01);

      DoubleYoVariable repeatNameVariable_root = new DoubleYoVariable("repeatNameVariable", root);
      DoubleYoVariable repeatNameVariable_0 = new DoubleYoVariable("repeatNameVariable", registry0);
      DoubleYoVariable repeatNameVariable_00 = new DoubleYoVariable("repeatNameVariable", registry00);
      DoubleYoVariable repeatNameVariable_01 = new DoubleYoVariable("repeatNameVariable", registry01);

      DoubleYoVariable exceptionalVariable = new DoubleYoVariable("exceptionalVariable", root);

      variable1.set(0.1);
      variable2.set(0.2);
      variable3.set(0.3);
      variable4.set(0.4);
      
      boolean1.set(true);
      boolean2.set(true);
      
      repeatNameVariable_root.set(10.0);
      repeatNameVariable_0.set(20.0);
      repeatNameVariable_00.set(30.0);
      repeatNameVariable_01.set(40.0);

      exceptionalVariable.set(1234.0);
      
      return root;
   }
}
