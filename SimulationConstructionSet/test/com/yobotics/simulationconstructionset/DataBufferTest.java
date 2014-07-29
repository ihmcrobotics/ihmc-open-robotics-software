package com.yobotics.simulationconstructionset;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import com.yobotics.simulationconstructionset.DataBuffer.RepeatDataBufferEntryException;

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
   }

   @After
   public void tearDown()
   {
      doubleYoVariable = null;
      registry = null;
   }


   
   @Test
   public void testGetBufferSize()
   {
      int testBufferSize = dataBuffer.getBufferSize();
      int expectedBufferSize = testBufferSize;
      assertTrue(expectedBufferSize == testBufferSize);
   }
   
   @Test
   public void testGetMaxBufferSize()
   {
      int expectedMaxBufferSize = 16384;
      int testMaxBufferSize = dataBuffer.getMaxBufferSize();
      assertTrue(expectedMaxBufferSize == testMaxBufferSize);
   }
   
   @Test
   public void testGetAndSetWrapBuffer()
   {
      dataBuffer.setWrapBuffer(false);
      boolean testBoolean = dataBuffer.getWrapBuffer();
      assertFalse(testBoolean);
      dataBuffer.setWrapBuffer(true);
      testBoolean = dataBuffer.getWrapBuffer();
      assertTrue(testBoolean); 
   }
   
   @Test
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
   
   
   
   @Test
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
   
   
   @Test
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
   
   @Test
   public void testAddVariableWithArrayList() throws RepeatDataBufferEntryException
   {
      ArrayList<YoVariable> arrayListToBeAdded = new ArrayList<YoVariable>();
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
   
   @Test
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
      
      ArrayList<YoVariable> currentlyMatched = new ArrayList<YoVariable>();
      
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
   
   @Test
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
   
   @Test
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
   
   @Test
   public void testGetVariables() throws RepeatDataBufferEntryException
   {
      dataBuffer.addVariable(doubleYoVariable, testBufferSize);
      dataBuffer.addVariable(booleanYoVariable, testBufferSize);
      dataBuffer.addVariable(integerYoVariable, testBufferSize);
      dataBuffer.addVariable(enumYoVariable, testBufferSize);
      
      ArrayList<YoVariable> expectedArrayOfVariables = new ArrayList<YoVariable>();
      expectedArrayOfVariables.add(doubleYoVariable);
      expectedArrayOfVariables.add(booleanYoVariable);
      expectedArrayOfVariables.add(integerYoVariable);
      expectedArrayOfVariables.add(enumYoVariable);
      
      ArrayList<YoVariable> actualArrayOfVariables = dataBuffer.getAllVariables();
      
      for(int i = 0; i < actualArrayOfVariables.size(); i++)
      {
         assertTrue(expectedArrayOfVariables.contains(actualArrayOfVariables.get(i)));
      }
 
   }
   
   
   //testGetVars(String [], String[])
   
   //testGetVarsFromGroup(String varGroupName, VarGroupList varGroupList)
   
   //setMaxBufferSize
   
   
//   @Test
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
//      System.out.println(dataBufferSize);
//      assertTrue(0 == dataBufferSize);
//
//   }
   
   
//   @Test
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
//       System.out.println(dataBufferSize);
//       assertTrue(0 == dataBufferSize);
//      
//   }
  
   
   
   @Test
   public void testEmptyBufferIncreaseBufferSize()
   {
      int originalBufferSize = dataBuffer.getBufferSize();
      int newBufferSize = originalBufferSize * 2;
            
      dataBuffer.changeBufferSize(newBufferSize);
      System.out.println(newBufferSize + " " + dataBuffer.getBufferSize()); 
      assertEquals(newBufferSize, dataBuffer.getBufferSize());
   }
   
   @Test
   public void testEmptyBufferDecreaseBufferSize()
   {
      int originalBufferSize = dataBuffer.getBufferSize();
      int newBufferSize = originalBufferSize/2;
            
      dataBuffer.changeBufferSize(newBufferSize);
      System.out.println(newBufferSize + " " + dataBuffer.getBufferSize()); 
      assertEquals(newBufferSize, dataBuffer.getBufferSize());
   }
   
   @Test
   public void testEnlargeBufferSize()
   {
      DataBufferEntry doubleDataBufferEntryTest = new DataBufferEntry(doubleYoVariable, testBufferSize);
      dataBuffer.addEntry(doubleDataBufferEntryTest);
      
      int originalBufferSize = dataBuffer.getBufferSize();
      int newBufferSize = originalBufferSize * 2;
            
      dataBuffer.changeBufferSize(newBufferSize);
      System.out.println(newBufferSize + " " + dataBuffer.getBufferSize()); 
      assertEquals(newBufferSize, dataBuffer.getBufferSize());
   }
   
   @Test
   public void testDecreaseBufferSize()
   {
      DataBufferEntry doubleDataBufferEntryTest = new DataBufferEntry(doubleYoVariable, testBufferSize);
      dataBuffer.addEntry(doubleDataBufferEntryTest);
      
      int originalBufferSize = dataBuffer.getBufferSize();
      int newBufferSize = originalBufferSize/2;
            
      dataBuffer.changeBufferSize(newBufferSize);
      System.out.println(newBufferSize + " " + dataBuffer.getBufferSize()); 
      assertEquals(newBufferSize, dataBuffer.getBufferSize());
   }
   
   @Test
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
   
   
   @Test
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
   
//   @Test
//   public void testCopyValuesThrough()
//   {
//      
//   }
   
//   @Test
//   public void testGetInAndOutLength()
//   {
//      System.out.println(dataBuffer.getBufferInOutLength());
//   }
   
   //@Test
  // public void 
   
   
}
