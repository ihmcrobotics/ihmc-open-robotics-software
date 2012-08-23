package com.yobotics.simulationconstructionset.scripts;

import static org.junit.Assert.*;

import java.io.BufferedReader;
import java.io.StringReader;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class TimeScriptTest
{
   private YoVariableRegistry rootRegistry, registryOne, registryTwo;
   private DoubleYoVariable variableOne, variableTwo, variableThree;
   
   @Before
   public void setUp() throws Exception
   {
      rootRegistry = new YoVariableRegistry("root");
      registryOne = new YoVariableRegistry("registryOne");
      registryTwo = new YoVariableRegistry("registryTwo");

      rootRegistry.addChild(registryOne);
      registryOne.addChild(registryTwo);

      variableOne = new DoubleYoVariable("variableOne", rootRegistry);
      variableTwo = new DoubleYoVariable("variableTwo", registryOne);
      variableThree = new DoubleYoVariable("variableThree", registryTwo);
     
   }

   @After
   public void tearDown() throws Exception
   {
      rootRegistry = null;
      registryOne = null;
      registryTwo = null;
      
      variableOne = null;
      variableTwo = null;
      variableThree = null;
   }

   @Test
   public void testEmptyTimeScript()
   {
      YoVariableRegistry registry = new YoVariableRegistry("Test");
      
      TimeScript timeScript = new TimeScript(registry);
      
      double time = 1.0;
      timeScript.doScript(time);
   }
   
   @Test
   public void testAddEntryAndDoScript()
   {
      TimeScript timeScript = new TimeScript(rootRegistry);
      
      double timeOne = 1.0;
      double valueOne = 1.11;
      
      double timeTwo = 1.9;
      double valueTwo = 3.55;
      
      double timeThree = 5.5;
      double valueThree = -72.4;

      double epsilon = 1e-7;
      
      double initialValue = 99.9;
      variableOne.set(initialValue);
      
      TimeScriptEntry timeScriptEntryOne = new TimeScriptEntry(timeOne);
      timeScriptEntryOne.addVarValue(variableOne, valueOne);
      timeScript.addEntry(timeScriptEntryOne);
      
      TimeScriptEntry timeScriptEntryTwo = new TimeScriptEntry(timeTwo);
      timeScriptEntryTwo.addVarValue(variableOne, valueTwo);
      timeScript.addEntry(timeScriptEntryTwo);
      
      TimeScriptEntry timeScriptEntryThree = new TimeScriptEntry(timeThree);
      timeScriptEntryThree.addVarValue(variableOne, valueThree);
      timeScript.addEntry(timeScriptEntryThree);
      
      double time = 0.0;
      timeScript.doScript(time);
      
      assertEquals(initialValue, variableOne.getDoubleValue(), epsilon);
      
      for (time=0.0; time<10.0; time=time+0.001)
      {
         timeScript.doScript(time);
         
         if (time < timeOne)
         {
            assertEquals(initialValue, variableOne.getDoubleValue(), epsilon);
         }
         else if (time < timeTwo)
         {
            assertEquals(valueOne, variableOne.getDoubleValue(), epsilon);
         }
         else if (time < timeThree)
         {
            assertEquals(valueTwo, variableOne.getDoubleValue(), epsilon);
         }
         else
         {
            assertEquals(valueThree, variableOne.getDoubleValue(), epsilon);
         }
      }
   }
   
   
   @Test
   public void testSaveAndLoad()
   {
      String[] variableNames = new String[]{"variableOne", "variableTwo", "variableThree"};
      
      double[] times = new double[]{1.0, 2.1, 5.77};
      double[][] values = new double[times.length][variableNames.length];
      
      for (int i=0; i<times.length; i++)
      {
         for (int j=0; j<values[0].length; j++)
         {
            values[i][j] = 100.0 * Math.random() - 50.0;
         }
      }
      
      TimeScript timeScript = new TimeScript(rootRegistry);
    
      String pseudoFile = "";
      
      for (int i=0; i<times.length; i++)
      {
         pseudoFile = pseudoFile + "t = " + times[i] + ":\n";
         pseudoFile = pseudoFile + "/* This is a comment!! */\n";

         for (int j=0; j<values[0].length; j++)
         {
            pseudoFile = pseudoFile + variableNames[j] + " = " + values[i][j] + ";\n";
            pseudoFile = pseudoFile + "// This is a comment!! \n";
         }
      }
      
//      System.out.println(pseudoFile);
      
      StringReader reader = new StringReader(pseudoFile);
      
      BufferedReader in = new BufferedReader(reader);
      
      timeScript.readTimeScript(rootRegistry, in);
      
      int timeIndex = -1;
      
      for (double time=0.0; time<10.0; time=time+0.001)
      {
         timeScript.doScript(time);
         
         if ((timeIndex < times.length-1) && (time >= times[timeIndex+1]))
         {
            timeIndex++;
         }
         
         if (timeIndex >= 0)
         {
            for (int variableIndex=0; variableIndex<values[0].length; variableIndex++)
            {
               DoubleYoVariable yoVariable = (DoubleYoVariable) rootRegistry.getVariable(variableNames[variableIndex]);
               
               assertEquals(values[timeIndex][variableIndex], yoVariable.getDoubleValue(), 1e-7);
            }
         }
      }
      
   }
   

}
