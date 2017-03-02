package us.ihmc.simulationconstructionset;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

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
import us.ihmc.simulationconstructionset.DataBuffer.RepeatDataBufferEntryException;
import us.ihmc.simulationconstructionset.gui.config.VarGroup;
import us.ihmc.simulationconstructionset.gui.config.VarGroupList;

public class DataBufferTest
{
   private final int testBufferSize = 100;
   
   private enum EnumYoVariableTestEnums
   {
      ONE, TWO;
   }

   private EnumYoVariable<EnumYoVariableTestEnums> enumYoVariable;
   private DoubleYoVariable doubleYoVariable;
   private BooleanYoVariable booleanYoVariable;
   private IntegerYoVariable integerYoVariable;
   private YoVariableRegistry registry;
   private DataBuffer dataBuffer = new DataBuffer(testBufferSize);
   
   private DoubleYoVariable a, b, c; 
   private DataBufferEntry aBuffer, bBuffer, cBuffer;

   public DataBufferTest()
   {
      
   }
   
   @Before
   public void setUp()
   {
      registry = new YoVariableRegistry("testRegistry");
      doubleYoVariable = new DoubleYoVariable("doubleYoVariable", registry);
      booleanYoVariable = new BooleanYoVariable("booleanYoVariable", registry);
      integerYoVariable = new IntegerYoVariable("integerYoVariable", registry);
      enumYoVariable = new EnumYoVariable<DataBufferTest.EnumYoVariableTestEnums>("enumYoVariable", registry, EnumYoVariableTestEnums.class);
      
      a = new DoubleYoVariable("a_arm", registry);
      b = new DoubleYoVariable("b_arm", registry);
      c = new DoubleYoVariable("c_arm", registry);      
      
      aBuffer = new DataBufferEntry(a, testBufferSize);
      bBuffer = new DataBufferEntry(b, testBufferSize);
      cBuffer = new DataBufferEntry(c, testBufferSize);
   }

   @After
   public void tearDown()
   {
      doubleYoVariable = null;
      registry = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetBufferSize()
   {
      int testBufferSize = dataBuffer.getBufferSize();
      int expectedBufferSize = testBufferSize;
      assertTrue(expectedBufferSize == testBufferSize);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetMaxBufferSize()
   {
      int expectedMaxBufferSize = 16384;
      int testMaxBufferSize = dataBuffer.getMaxBufferSize();
      assertTrue(expectedMaxBufferSize == testMaxBufferSize);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetAndSetWrapBuffer()
   {
      dataBuffer.setWrapBuffer(false);
      boolean testBoolean = dataBuffer.getWrapBuffer();
      assertFalse(testBoolean);
      dataBuffer.setWrapBuffer(true);
      testBoolean = dataBuffer.getWrapBuffer();
      assertTrue(testBoolean); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAddAndGetEntry()
   {
      DataBufferEntry doubleDataBufferEntryTest = new DataBufferEntry(doubleYoVariable, testBufferSize);
      DataBufferEntry booleanDataBufferEntryTest = new DataBufferEntry(booleanYoVariable, testBufferSize);
      DataBufferEntry integerDataBufferEntryTest = new DataBufferEntry(integerYoVariable, testBufferSize);
      DataBufferEntry enumDataBufferEntryTest = new DataBufferEntry(enumYoVariable, testBufferSize);
      dataBuffer.addEntry(doubleDataBufferEntryTest);
      dataBuffer.addEntry(booleanDataBufferEntryTest);
      dataBuffer.addEntry(integerDataBufferEntryTest);
      dataBuffer.addEntry(enumDataBufferEntryTest);
      
      DataBufferEntry testEntryReceivedViaString = dataBuffer.getEntry("doubleYoVariable");
      DataBufferEntry testEntryReceivedViaVariableName = dataBuffer.getEntry(doubleYoVariable);
      assertEquals(doubleDataBufferEntryTest, testEntryReceivedViaString);
      assertEquals(doubleDataBufferEntryTest, testEntryReceivedViaVariableName);
      
      testEntryReceivedViaString = dataBuffer.getEntry("booleanYoVariable");
      testEntryReceivedViaVariableName = dataBuffer.getEntry(booleanYoVariable);
      assertEquals(booleanDataBufferEntryTest, testEntryReceivedViaString);
      assertEquals(booleanDataBufferEntryTest, testEntryReceivedViaVariableName);
      
      testEntryReceivedViaString = dataBuffer.getEntry("integerYoVariable");
      testEntryReceivedViaVariableName = dataBuffer.getEntry(integerYoVariable);
      assertEquals(integerDataBufferEntryTest, testEntryReceivedViaString);
      assertEquals(integerDataBufferEntryTest, testEntryReceivedViaVariableName);
      
      testEntryReceivedViaString = dataBuffer.getEntry("enumYoVariable");
      testEntryReceivedViaVariableName = dataBuffer.getEntry(enumYoVariable);
      assertEquals(enumDataBufferEntryTest, testEntryReceivedViaString);
      assertEquals(enumDataBufferEntryTest, testEntryReceivedViaVariableName);

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAddNewEntry() throws RepeatDataBufferEntryException
   {
      dataBuffer.addVariable(doubleYoVariable, testBufferSize);
      dataBuffer.addVariable(booleanYoVariable, testBufferSize);
      dataBuffer.addVariable(integerYoVariable, testBufferSize);
      dataBuffer.addVariable(enumYoVariable, testBufferSize);
      
      DataBufferEntry doubleDataBufferEntryTest = new DataBufferEntry(doubleYoVariable, testBufferSize);
      DataBufferEntry booleanDataBufferEntryTest = new DataBufferEntry(booleanYoVariable, testBufferSize);
      DataBufferEntry integerDataBufferEntryTest = new DataBufferEntry(integerYoVariable, testBufferSize);
      DataBufferEntry enumDataBufferEntryTest = new DataBufferEntry(enumYoVariable, testBufferSize);
      
      assertTrue(doubleDataBufferEntryTest.getVariable() == dataBuffer.getEntry(doubleYoVariable).getVariable());
      assertTrue(booleanDataBufferEntryTest.getVariable() == dataBuffer.getEntry(booleanYoVariable).getVariable());
      assertTrue(integerDataBufferEntryTest.getVariable() == dataBuffer.getEntry(integerYoVariable).getVariable());
      assertTrue(enumDataBufferEntryTest.getVariable() == dataBuffer.getEntry(enumYoVariable).getVariable());    
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAddVariable() throws RepeatDataBufferEntryException
   {
      dataBuffer.addVariable(doubleYoVariable);
      dataBuffer.addVariable(booleanYoVariable);
      dataBuffer.addVariable(integerYoVariable);
      dataBuffer.addVariable(enumYoVariable);
      
      assertTrue(doubleYoVariable == dataBuffer.getEntry(doubleYoVariable).getVariable());
      assertTrue(booleanYoVariable == dataBuffer.getEntry(booleanYoVariable).getVariable());
      assertTrue(integerYoVariable == dataBuffer.getEntry(integerYoVariable).getVariable());
      assertTrue(enumYoVariable == dataBuffer.getEntry(enumYoVariable).getVariable());

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAddVariableWithArrayList() throws RepeatDataBufferEntryException
   {
      ArrayList<YoVariable<?>> arrayListToBeAdded = new ArrayList<YoVariable<?>>();
      arrayListToBeAdded.add(doubleYoVariable);
      arrayListToBeAdded.add(booleanYoVariable);
      arrayListToBeAdded.add(integerYoVariable);
      arrayListToBeAdded.add(enumYoVariable);
      
      dataBuffer.addVariables(arrayListToBeAdded);
      
      assertTrue(doubleYoVariable == dataBuffer.getEntry(doubleYoVariable).getVariable());
      assertTrue(booleanYoVariable == dataBuffer.getEntry(booleanYoVariable).getVariable());
      assertTrue(integerYoVariable == dataBuffer.getEntry(integerYoVariable).getVariable());
      assertTrue(enumYoVariable == dataBuffer.getEntry(enumYoVariable).getVariable());
      
   }
   
   //add dataBuffer listener?

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVariablesThatContain() throws RepeatDataBufferEntryException
   {
      DoubleYoVariable yoVariable123456789 = new DoubleYoVariable("123456789", registry);
      DoubleYoVariable yoVariable12345678 = new DoubleYoVariable("12345678", registry);
      DoubleYoVariable yoVariable1234567 = new DoubleYoVariable("1234567", registry);
      DoubleYoVariable yoVariable123456 = new DoubleYoVariable("123456", registry);
      DoubleYoVariable yoVariable12345 = new DoubleYoVariable("12345", registry);
      DoubleYoVariable yoVariable1234 = new DoubleYoVariable("1234", registry);
      DoubleYoVariable yoVariable123 = new DoubleYoVariable("123", registry);
      DoubleYoVariable yoVariable12 = new DoubleYoVariable("12", registry);
      DoubleYoVariable yoVariable1 = new DoubleYoVariable("1", registry);
      
      ArrayList<YoVariable<?>> currentlyMatched = new ArrayList<YoVariable<?>>();
      
      currentlyMatched.add(yoVariable123456789);
      currentlyMatched.add(yoVariable12345678);
      currentlyMatched.add(yoVariable1234567);
      currentlyMatched.add(yoVariable123456);
      currentlyMatched.add(yoVariable12345);
      currentlyMatched.add(yoVariable1234);
      currentlyMatched.add(yoVariable123);
      currentlyMatched.add(yoVariable12);
      currentlyMatched.add(yoVariable1);

      
      assertTrue(1 == dataBuffer.getVariablesThatContain("123456789", true, currentlyMatched).size());
      assertTrue(2 == dataBuffer.getVariablesThatContain("12345678", true, currentlyMatched).size());
      assertTrue(3 == dataBuffer.getVariablesThatContain("1234567", true, currentlyMatched).size());
      assertTrue(4 == dataBuffer.getVariablesThatContain("123456", true, currentlyMatched).size());
      assertTrue(5 == dataBuffer.getVariablesThatContain("12345", true, currentlyMatched).size());
      assertTrue(6 == dataBuffer.getVariablesThatContain("1234", true, currentlyMatched).size());
      assertTrue(7 == dataBuffer.getVariablesThatContain("123", true, currentlyMatched).size());
      assertTrue(8 == dataBuffer.getVariablesThatContain("12", true, currentlyMatched).size());
      assertTrue(9 == dataBuffer.getVariablesThatContain("1", true, currentlyMatched).size());
      assertTrue(null == dataBuffer.getVariablesThatContain("1234567890", true, currentlyMatched));
      assertTrue(null == dataBuffer.getVariablesThatContain("987654321", true, currentlyMatched));
      
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVariablesThatStartWith() throws RepeatDataBufferEntryException
   {
      
      DoubleYoVariable yoVariable1 = new DoubleYoVariable("doy", registry);
      DoubleYoVariable yoVariable2= new DoubleYoVariable("Dog", registry);
      DoubleYoVariable yoVariable3 = new DoubleYoVariable("bar", registry);
      
      dataBuffer.addVariable(doubleYoVariable);
      dataBuffer.addVariable(booleanYoVariable);
      dataBuffer.addVariable(integerYoVariable);
      dataBuffer.addVariable(enumYoVariable);
      dataBuffer.addVariable(yoVariable1);
      dataBuffer.addVariable(yoVariable2);
      dataBuffer.addVariable(yoVariable3);
      
      assertTrue(3 == dataBuffer.getVariablesThatStartWith("d", false).size());
      assertTrue(2 == dataBuffer.getVariablesThatStartWith("d").size());
      assertTrue(2 == dataBuffer.getVariablesThatStartWith("b").size());
      
      
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetEntries() throws RepeatDataBufferEntryException
   {
      ArrayList<DataBufferEntry> expectedDataEntries = new ArrayList<DataBufferEntry>();
      DataBufferEntry doubleDataBufferEntryTest = new DataBufferEntry(doubleYoVariable, testBufferSize);
      DataBufferEntry booleanDataBufferEntryTest = new DataBufferEntry(booleanYoVariable, testBufferSize);
      DataBufferEntry integerDataBufferEntryTest = new DataBufferEntry(integerYoVariable, testBufferSize);
      DataBufferEntry enumDataBufferEntryTest = new DataBufferEntry(enumYoVariable, testBufferSize);
      
      dataBuffer.addEntry(doubleDataBufferEntryTest);
      dataBuffer.addEntry(booleanDataBufferEntryTest);
      dataBuffer.addEntry(integerDataBufferEntryTest);
      dataBuffer.addEntry(enumDataBufferEntryTest);
      
      expectedDataEntries.add(doubleDataBufferEntryTest);
      expectedDataEntries.add(booleanDataBufferEntryTest);
      expectedDataEntries.add(integerDataBufferEntryTest);
      expectedDataEntries.add(enumDataBufferEntryTest);
      
      assertEquals(expectedDataEntries, dataBuffer.getEntries());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVariables() throws RepeatDataBufferEntryException
   {
      dataBuffer.addVariable(doubleYoVariable, testBufferSize);
      dataBuffer.addVariable(booleanYoVariable, testBufferSize);
      dataBuffer.addVariable(integerYoVariable, testBufferSize);
      dataBuffer.addVariable(enumYoVariable, testBufferSize);
      
      ArrayList<YoVariable<?>> expectedArrayOfVariables = new ArrayList<YoVariable<?>>();
      expectedArrayOfVariables.add(doubleYoVariable);
      expectedArrayOfVariables.add(booleanYoVariable);
      expectedArrayOfVariables.add(integerYoVariable);
      expectedArrayOfVariables.add(enumYoVariable);
      
      ArrayList<YoVariable<?>> actualArrayOfVariables = dataBuffer.getAllVariables();
      
      for(int i = 0; i < actualArrayOfVariables.size(); i++)
      {
         assertTrue(expectedArrayOfVariables.contains(actualArrayOfVariables.get(i)));
      }
 
   }
   
/*   
   //testGetVars(String [], String[])
   
   //testGetVarsFromGroup(String varGroupName, VarGroupList varGroupList)
   
   //setMaxBufferSize
   
   
//   @Test(timeout=300000)
//   public void testResetDataBuffer() throws RepeatDataBufferEntryException
//   {
//      dataBuffer.addNewEntry(doubleYoVariable, testBufferSize);
//      dataBuffer.addNewEntry(booleanYoVariable, testBufferSize);
//      dataBuffer.addNewEntry(integerYoVariable, testBufferSize);
//      dataBuffer.addNewEntry(enumYoVariable, testBufferSize);
//      
//      dataBuffer.resetDataBuffer();
//      
//      int dataBufferSize = dataBuffer.getBufferSize();
//      .println(dataBufferSize);
//      assertTrue(0 == dataBufferSize);
//
//   }
   
   
//   @Test(timeout=300000)
//   public void testClearAll() throws RepeatDataBufferEntryException
//   {
//      
//       dataBuffer.addNewEntry(doubleYoVariable, testBufferSize);
//       dataBuffer.addNewEntry(booleanYoVariable, testBufferSize);
//       dataBuffer.addNewEntry(integerYoVariable, testBufferSize);
//       dataBuffer.addNewEntry(enumYoVariable, testBufferSize);
//       
//       dataBuffer.clearAll(testBufferSize);
//       
//       int dataBufferSize = dataBuffer.getBufferSize();
//       .println(dataBufferSize);
//       assertTrue(0 == dataBufferSize);
//      
//   }
  */   

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testEmptyBufferIncreaseBufferSize()
   {
      int originalBufferSize = dataBuffer.getBufferSize();
      int newBufferSize = originalBufferSize * 2;
            
      dataBuffer.changeBufferSize(newBufferSize);
//      .println(newBufferSize + " " + dataBuffer.getBufferSize()); 
      assertEquals(newBufferSize, dataBuffer.getBufferSize());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testEmptyBufferDecreaseBufferSize()
   {
      int originalBufferSize = dataBuffer.getBufferSize();
      int newBufferSize = originalBufferSize/2;
            
      dataBuffer.changeBufferSize(newBufferSize);
//      .println(newBufferSize + " " + dataBuffer.getBufferSize()); 
      assertEquals(newBufferSize, dataBuffer.getBufferSize());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testEnlargeBufferSize()
   {
      DataBufferEntry doubleDataBufferEntryTest = new DataBufferEntry(doubleYoVariable, testBufferSize);
      dataBuffer.addEntry(doubleDataBufferEntryTest);
      
      int originalBufferSize = dataBuffer.getBufferSize();
      int newBufferSize = originalBufferSize * 2;
            
      dataBuffer.changeBufferSize(newBufferSize);
//      System.out.println(newBufferSize + " " + dataBuffer.getBufferSize()); 
      assertEquals(newBufferSize, dataBuffer.getBufferSize());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testDecreaseBufferSize()
   {
      DataBufferEntry doubleDataBufferEntryTest = new DataBufferEntry(doubleYoVariable, testBufferSize);
      dataBuffer.addEntry(doubleDataBufferEntryTest);
      
      int originalBufferSize = dataBuffer.getBufferSize();
      int newBufferSize = originalBufferSize/2;
            
      dataBuffer.changeBufferSize(newBufferSize);
//      System.out.println(newBufferSize + " " + dataBuffer.getBufferSize()); 
      assertEquals(newBufferSize, dataBuffer.getBufferSize());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testTick()
   {
      int numberOfTicksAndUpdates = 20;
      for (int i=0; i<numberOfTicksAndUpdates; i++)
      {
         dataBuffer.tickAndUpdate();
      }
      
      dataBuffer.gotoInPoint();
      
      int expectedIndex = 0;
      while(dataBuffer.getIndex() < dataBuffer.getBufferInOutLength()-1)
      {
         assertEquals(expectedIndex, dataBuffer.getIndex());
         boolean rolledOver = dataBuffer.tick(1);
         assertFalse(rolledOver);
         expectedIndex++;
      }
      
      boolean rolledOver = dataBuffer.tick(1);
      assertTrue(rolledOver);
      expectedIndex = 0;
      assertEquals(expectedIndex, dataBuffer.getIndex());
      
      rolledOver = dataBuffer.tick(1);
      assertFalse(rolledOver);
      expectedIndex = 1;
      assertEquals(expectedIndex, dataBuffer.getIndex());

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testIsIndexBetweenInAndOutPoint()
   {
      assertEquals(0, dataBuffer.getIndex());
      assertEquals(0, dataBuffer.getInPoint());
      assertEquals(0, dataBuffer.getOutPoint());
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(0));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(1));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(-1));
      
      dataBuffer.tickAndUpdate();
      assertEquals(1, dataBuffer.getIndex());
      assertEquals(0, dataBuffer.getInPoint());
      assertEquals(1, dataBuffer.getOutPoint());
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(0));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(1));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(-1));
      
      dataBuffer.tickAndUpdate();
      assertEquals(2, dataBuffer.getIndex());
      assertEquals(0, dataBuffer.getInPoint());
      assertEquals(2, dataBuffer.getOutPoint());
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(0));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(1));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(2));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(3));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(-1));
      
      int numTicks = 20;
      for (int i=0; i<numTicks; i++)
      {
         dataBuffer.tickAndUpdate();
      }
      assertEquals(dataBuffer.getOutPoint(), dataBuffer.getIndex());
      assertEquals(0, dataBuffer.getInPoint());

      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex()-1));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex()));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex()+1));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getInPoint()-1));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getInPoint()));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getInPoint()+1));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getOutPoint()-1));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getOutPoint()));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getOutPoint()+1));
      
      dataBuffer.cropData();
      dataBuffer.gotoOutPoint();
      
      dataBuffer.setWrapBuffer(true);
      
      assertEquals(dataBuffer.getOutPoint(), dataBuffer.getIndex());
      assertEquals(0, dataBuffer.getInPoint());
      
      numTicks = 7;
      
      for (int i=0; i<numTicks; i++)
      {
         dataBuffer.tickAndUpdate();
      }
      
      assertEquals(dataBuffer.getOutPoint(), dataBuffer.getIndex());
      assertEquals(numTicks - 1, dataBuffer.getOutPoint());
      assertEquals(numTicks, dataBuffer.getInPoint());
      
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex()-1));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex()));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex()+1));
      
      dataBuffer.tickAndUpdate();
      assertEquals(dataBuffer.getOutPoint(), dataBuffer.getIndex());
      assertEquals(dataBuffer.getOutPoint(), dataBuffer.getInPoint()-1);
      
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex()-1));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex()));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex()+1));

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetSafeToChangeIndex() //Luke Morris
   {
      boolean isSafe = dataBuffer.isSafeToChangeIndex();
      assertTrue(isSafe);
      dataBuffer.setSafeToChangeIndex(false);
      boolean isNowSafe = dataBuffer.isSafeToChangeIndex();
      assertFalse(isNowSafe);
      dataBuffer.setSafeToChangeIndex(true);
      boolean isFinallySafe = dataBuffer.isSafeToChangeIndex();
      assertTrue(isFinallySafe);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVariablesTwo() //Luke Morris
   {
      ArrayList<YoVariable<?>> variables = dataBuffer.getVariables();
      //    return dataBuffer.toString();
      //    return variables.toString();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVars() //Luke Morris

   {
      dataBuffer.addEntry(aBuffer);
      dataBuffer.addEntry(bBuffer);
      dataBuffer.addEntry(cBuffer);

      String[] varNames = new String[3];
      varNames[0] = "a_arm";
      varNames[1] = "b_arm";
      varNames[2] = "c_arm";

      String[] aNames = new String[1];
      aNames[0] = "a_arm";

      String[] allRegularExpressions = { ".*" };
      String[] cRegularExpressions = { "c.*" };

      ArrayList<YoVariable<?>> both = dataBuffer.getVars(varNames, allRegularExpressions);

      assertTrue(both.contains(a));
      assertTrue(both.contains(b));
      assertTrue(both.contains(c));

      ArrayList<YoVariable<?>> justNames = dataBuffer.getVars(varNames, null);

      assertTrue(justNames.contains(a));
      assertTrue(justNames.contains(b));
      assertTrue(justNames.contains(c));

      ArrayList<YoVariable<?>> justA = dataBuffer.getVars(aNames, null);

      assertTrue(justA.contains(a));
      assertFalse(justA.contains(b));
      assertFalse(justA.contains(c));

      ArrayList<YoVariable<?>> justRegExp = dataBuffer.getVars(null, allRegularExpressions);

      assertTrue(justRegExp.contains(a));
      assertTrue(justRegExp.contains(b));
      assertTrue(justRegExp.contains(c));

      ArrayList<YoVariable<?>> cRegExp = dataBuffer.getVars(null, cRegularExpressions);

      assertFalse(cRegExp.contains(a));
      assertFalse(cRegExp.contains(b));
      assertTrue(cRegExp.contains(c));

      ArrayList<YoVariable<?>> neither = dataBuffer.getVars(null, null);

      assertFalse(neither.contains(a));
      assertFalse(neither.contains(b));
      assertFalse(neither.contains(c));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVarsFromGroup()
   {
      dataBuffer.addEntry(aBuffer);
      dataBuffer.addEntry(bBuffer);
      dataBuffer.addEntry(cBuffer);

      VarGroup varGroupOne = new VarGroup("varGroupOne");
      VarGroup varGroupTwo = new VarGroup("varGroupTwo");
      VarGroup varGroupThree = new VarGroup("varGroupThree");

      varGroupOne.addVar("a_arm");
      varGroupOne.addVar("c_arm");

      String[] allRegularExpressions = { ".*" };
      String[] cRegularExpressions = { "c.*" };

      varGroupTwo.addRegularExpressions(allRegularExpressions);
      varGroupThree.addRegularExpressions(cRegularExpressions);

      VarGroupList varGroupList = new VarGroupList();
      varGroupList.addVarGroup(varGroupOne);
      varGroupList.addVarGroup(varGroupTwo);
      varGroupList.addVarGroup(varGroupThree);

      ArrayList<YoVariable<?>> allVarsFromGroup = dataBuffer.getVarsFromGroup("all", varGroupList);

      assertTrue(allVarsFromGroup.contains(a));
      assertTrue(allVarsFromGroup.contains(b));
      assertTrue(allVarsFromGroup.contains(c));

      ArrayList<YoVariable<?>> aVarsFromGroup = dataBuffer.getVarsFromGroup("varGroupOne", varGroupList);

      assertTrue(aVarsFromGroup.contains(a));
      assertFalse(aVarsFromGroup.contains(b));
      assertTrue(aVarsFromGroup.contains(c));

      ArrayList<YoVariable<?>> regExpVarsFromGroup = dataBuffer.getVarsFromGroup("varGroupTwo", varGroupList);

      assertTrue(regExpVarsFromGroup.contains(a));
      assertTrue(regExpVarsFromGroup.contains(b));
      assertTrue(regExpVarsFromGroup.contains(c));

      ArrayList<YoVariable<?>> cRegExpVarsFromGroup = dataBuffer.getVarsFromGroup("varGroupThree", varGroupList);

      assertFalse(cRegExpVarsFromGroup.contains(a));
      assertFalse(cRegExpVarsFromGroup.contains(b));
      assertTrue(cRegExpVarsFromGroup.contains(c));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetMaxBufferSize()

   {
      int minBuffer = 1;
      int maxBuffer = 10000000;
      int zeroBuffer = 0;
      int normalBuffer = 200;
      
      dataBuffer.setMaxBufferSize(minBuffer);
      assertTrue(dataBuffer.getMaxBufferSize() == minBuffer);
      
      dataBuffer.setMaxBufferSize(maxBuffer);
      assertTrue(dataBuffer.getMaxBufferSize() == maxBuffer);
      
      dataBuffer.setMaxBufferSize(zeroBuffer);
      assertTrue(dataBuffer.getMaxBufferSize() == zeroBuffer);
      
      dataBuffer.setMaxBufferSize(normalBuffer);
      assertTrue(dataBuffer.getMaxBufferSize() == normalBuffer);
      
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testClearAll()
   {
      
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
    public void testResetDataBuffer()
    {       
       dataBuffer.addEntry(aBuffer);
       dataBuffer.addEntry(bBuffer);
       dataBuffer.addEntry(cBuffer);
       
       ArrayList<YoVariable<?>> withVariables = dataBuffer.getVariables();
       
//       System.out.println(withVariables.size());
       assertTrue(withVariables.size() > 0);
       
       dataBuffer.resetDataBuffer();
       
       ArrayList<YoVariable<?>> resetVariables = dataBuffer.getVariables();
       
//       System.out.println(resetVariables.size());
//       assertTrue(resetVariables.size() == 0);
    }


}
