package us.ihmc.simulationconstructionset;

import static org.junit.Assert.*;

import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

public class YoVariableHolderImplementationNewTest
{

   public enum EnumYoVariableTestEnums
   {
      ONE, TWO;
   }

   private YoVariableHolderImplementation yoVariableHolderImplementation;
   private ArrayList<YoVariable<?>> testVariables;


   public YoVariableHolderImplementationNewTest()
   {

   }

   @Before
   public void setUp()
   {
      yoVariableHolderImplementation = new YoVariableHolderImplementation();
      testVariables = new ArrayList<YoVariable<?>>(); 
      testVariables.add(new DoubleYoVariable("doubleYoVariable", null));
      testVariables.add(new BooleanYoVariable("booleanYoVariable", null));
      testVariables.add(new IntegerYoVariable("integerYoVariable", null));
      testVariables.add(new EnumYoVariable<YoVariableHolderImplementationNewTest.EnumYoVariableTestEnums>("enumYoVariable", null, EnumYoVariableTestEnums.class));
   }

   @After
   public void tearDown()
   {
      yoVariableHolderImplementation = null;
      testVariables = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAddSingleYoVariableToHolderAndGetVariableByName()
   {
      DoubleYoVariable doubleYoVariableFromArrayList = (DoubleYoVariable) testVariables.get(0);
      yoVariableHolderImplementation.addVariableToHolder(doubleYoVariableFromArrayList);      
      assertEquals(doubleYoVariableFromArrayList, yoVariableHolderImplementation.getVariable("doubleYoVariable"));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAddMultipleYoVariablesToHolderAndGetAllVariables()
   {
      yoVariableHolderImplementation.addVariablesToHolder(testVariables);
      
      for(YoVariable<?> var : yoVariableHolderImplementation.getAllVariables())
      {
         assertTrue(testVariables.contains(var));
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetAllVariablesArray()
   {
      yoVariableHolderImplementation.addVariablesToHolder(testVariables);
      YoVariable<?>[] testVariableArray = new YoVariable[4];
      testVariableArray = yoVariableHolderImplementation.getAllVariablesArray();
      
      for(int i = 0; i < 4; i++)
      {
         assertTrue(testVariables.contains(testVariableArray [i]));         
      }
      
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVariableUsingFullNamespace()
   {
      yoVariableHolderImplementation.addVariablesToHolder(testVariables);
      assertTrue(testVariables.get(0) == yoVariableHolderImplementation.getVariableUsingFullNamespace("doubleYoVariable"));      
   }
   
   
//   @Test(timeout=300000)
//   public void testGetVariableUsingFullNamespaceError()
//   {
//      yoVariableHolderImplementation.addVariablesToHolder(testVariables);
//      yoVariableHolderImplementation.getVariableUsingFullNamespace("notPresent");
//      ByteArrayOutputStream stdErrorContents = new ByteArrayOutputStream();
//      PrintStream stdErr = new PrintStream(System.err);
//      System.setErr(new PrintStream(stdErrorContents));allihmc
//      System.setErr(stdErr);
//      assertEquals("Warning: " + "notPresent" + " not found. (YoVariableHolderImplementation.getVariable)", stdErrorContents.toString());
//      
//      
//   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVariable()
   {
      yoVariableHolderImplementation.addVariablesToHolder(testVariables);
      assertTrue(testVariables.get(0) == yoVariableHolderImplementation.getVariable("doubleYoVariable"));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVariableCaseInsensitive()
   {
      yoVariableHolderImplementation.addVariablesToHolder(testVariables);
      YoVariable<?> variable = yoVariableHolderImplementation.getVariable("DoubleYoVariable");
      assertTrue(testVariables.get(0) == variable);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVariableWithNameSpace() 
   {
      YoVariableRegistry testRegistry;
      testRegistry = new YoVariableRegistry("testRegistry"); 
      DoubleYoVariable doubleYoVariableWithNameSpace = new DoubleYoVariable("doubleYoVariableWithNameSpace", testRegistry);
      yoVariableHolderImplementation.addVariableToHolder(doubleYoVariableWithNameSpace);
      assertEquals(doubleYoVariableWithNameSpace, yoVariableHolderImplementation.getVariable("testRegistry", "doubleYoVariableWithNameSpace"));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVariableWithNameSpaceCaseInsensitiveExceptNameSpace() 
   {
      YoVariableRegistry testRegistry;
      testRegistry = new YoVariableRegistry("testRegistry"); 
      DoubleYoVariable doubleYoVariableWithNameSpace = new DoubleYoVariable("doubleYoVariableWithNameSpace", testRegistry);
      yoVariableHolderImplementation.addVariableToHolder(doubleYoVariableWithNameSpace);
      assertEquals(doubleYoVariableWithNameSpace, yoVariableHolderImplementation.getVariable("testRegistry", "DOUBLEYoVariableWithNameSpace"));
      assertNull(yoVariableHolderImplementation.getVariable("TESTRegistry", "doubleYoVariableWithNameSpace"));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testHasUniqueVariable()
   {
      yoVariableHolderImplementation.addVariablesToHolder(testVariables);      
      assertTrue(yoVariableHolderImplementation.hasUniqueVariable("doubleYoVariable"));
      assertFalse(yoVariableHolderImplementation.hasUniqueVariable("doubleYoVariableNotPresent"));
      assertTrue(yoVariableHolderImplementation.hasUniqueVariable("booleanYoVariable"));
      assertTrue(yoVariableHolderImplementation.hasUniqueVariable("integerYoVariable"));
      assertFalse(yoVariableHolderImplementation.hasUniqueVariable("integerYoVariableNotPresent"));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testHasUniqueVariableWithNameSpace()
   {
      YoVariableRegistry testRegistry1;
      YoVariableRegistry testRegistry2;
      testRegistry1 = new YoVariableRegistry("testRegistry1"); 
      testRegistry2 = new YoVariableRegistry("testRegistry2"); 
      DoubleYoVariable doubleYoVariableWithNameSpace1 = new DoubleYoVariable("doubleYoVariableWithNameSpace1", testRegistry1);
      DoubleYoVariable doubleYoVariableWithNameSpace2 = new DoubleYoVariable("doubleYoVariableWithNameSpace2", testRegistry2);
      yoVariableHolderImplementation.addVariableToHolder(doubleYoVariableWithNameSpace1);
      yoVariableHolderImplementation.addVariableToHolder(doubleYoVariableWithNameSpace2);
      assertTrue(yoVariableHolderImplementation.hasUniqueVariable("testRegistry1", "doubleYoVariableWithNameSpace1"));
      assertTrue(yoVariableHolderImplementation.hasUniqueVariable("testRegistry2", "doubleYoVariableWithNameSpace2"));
      assertFalse(yoVariableHolderImplementation.hasUniqueVariable("testRegistry1", "doubleYoVariableWithNameSpace2"));
      assertFalse(yoVariableHolderImplementation.hasUniqueVariable("testRegistry2", "doubleYoVariableWithNameSpace1"));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVariablesArrayList() //returns an ArrayList of size 1?
   {
      yoVariableHolderImplementation.addVariablesToHolder(testVariables);
//      assertEquals(testVariables, yoVariableHolderImplementation.getVariables())
   }
   
   //testGetVariables(String fullName)

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVariablesInNameSpace()
   {
      YoVariableRegistry testRegistry1;
      YoVariableRegistry testRegistry2;
      testRegistry1 = new YoVariableRegistry("testRegistry1"); 
      testRegistry2 = new YoVariableRegistry("testRegistry2"); 
      DoubleYoVariable doubleYoVariableWithNameSpace1 = new DoubleYoVariable("doubleYoVariableWithNameSpace1", testRegistry1);
      DoubleYoVariable doubleYoVariableWithNameSpace2 = new DoubleYoVariable("doubleYoVariableWithNameSpace2", testRegistry2);
      BooleanYoVariable booleanYoVariableWithNameSpace1 = new BooleanYoVariable("booleanYoVariableWithNameSpace1", testRegistry1);
      BooleanYoVariable booleanYoVariableWithNameSpace2 = new BooleanYoVariable("booleanYoVariableWithNameSpace2", testRegistry2);
      IntegerYoVariable integerYoVariableWithNameSpace1 = new IntegerYoVariable("integerYoVariableWithNameSpace1", testRegistry1);
      IntegerYoVariable integerYoVariableWithNameSpace2 = new IntegerYoVariable("integerYoVariableWithNameSpace2", testRegistry2);
      yoVariableHolderImplementation.addVariableToHolder(doubleYoVariableWithNameSpace1);
      yoVariableHolderImplementation.addVariableToHolder(doubleYoVariableWithNameSpace2);
      yoVariableHolderImplementation.addVariableToHolder(booleanYoVariableWithNameSpace1);
      yoVariableHolderImplementation.addVariableToHolder(booleanYoVariableWithNameSpace2);
      yoVariableHolderImplementation.addVariableToHolder(integerYoVariableWithNameSpace1);
      yoVariableHolderImplementation.addVariableToHolder(integerYoVariableWithNameSpace2);
      
      ArrayList<YoVariable<?>> expectedArrayListFromNameSpaceTestRegistry1 = new ArrayList<YoVariable<?>>();
      expectedArrayListFromNameSpaceTestRegistry1.add(doubleYoVariableWithNameSpace1);
      expectedArrayListFromNameSpaceTestRegistry1.add(booleanYoVariableWithNameSpace1);
      expectedArrayListFromNameSpaceTestRegistry1.add(integerYoVariableWithNameSpace1);
 
      for(int i = 0; i < expectedArrayListFromNameSpaceTestRegistry1.size(); i++)
      {
         assertTrue(expectedArrayListFromNameSpaceTestRegistry1.contains(yoVariableHolderImplementation.getVariables(testRegistry1.getNameSpace()).get(i)));
      }
    //does not return the ArrayList in specified order so contains method was used.
   }
   
   
   
   
   
   
   
   

}
