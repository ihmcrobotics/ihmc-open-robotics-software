package us.ihmc.simulationconstructionset.scripts;

import static org.junit.Assert.assertEquals;

import java.io.BufferedReader;
import java.io.StringReader;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;


public class TimeScriptTest
{
   private YoVariableRegistry rootRegistry, registryOne, registryTwo;
   private DoubleYoVariable doubleVariable;
   private BooleanYoVariable booleanVariable;
   private IntegerYoVariable integerVariable;
   private EnumYoVariable<TimeScriptTestEnums> enumVariable;
   
   @Before
   public void setUp() throws Exception
   {
      rootRegistry = new YoVariableRegistry("root");
      registryOne = new YoVariableRegistry("registryOne");
      registryTwo = new YoVariableRegistry("registryTwo");

      rootRegistry.addChild(registryOne);
      registryOne.addChild(registryTwo);

      doubleVariable = new DoubleYoVariable("doubleVariable", rootRegistry);
      booleanVariable = new BooleanYoVariable("booleanVariable", registryOne);
      integerVariable = new IntegerYoVariable("integerVariable", registryTwo);
      enumVariable = new EnumYoVariable<TimeScriptTestEnums>("enumVariable", registryTwo, TimeScriptTestEnums.class);
   }
   
   private enum TimeScriptTestEnums
   {
      V0, V1, V2;
   }

   @After
   public void tearDown() throws Exception
   {
      rootRegistry = null;
      registryOne = null;
      registryTwo = null;
      
      doubleVariable = null;
      booleanVariable = null;
      integerVariable = null;
      enumVariable = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testEmptyTimeScript()
   {
      YoVariableRegistry registry = new YoVariableRegistry("Test");
      
      TimeScript timeScript = new TimeScript(registry);
      
      double time = 1.0;
      timeScript.doScript(time);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
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
      doubleVariable.set(initialValue);
      
      TimeScriptEntry timeScriptEntryOne = new TimeScriptEntry(timeOne);
      timeScriptEntryOne.addVarValue(doubleVariable, valueOne);
      timeScript.addEntry(timeScriptEntryOne);
      
      TimeScriptEntry timeScriptEntryThree = new TimeScriptEntry(timeThree);
      timeScriptEntryThree.addVarValue(doubleVariable, valueThree);
      timeScript.addEntry(timeScriptEntryThree);
      
      TimeScriptEntry timeScriptEntryTwo = new TimeScriptEntry(timeTwo);
      timeScriptEntryTwo.addVarValue(doubleVariable, valueTwo);
      timeScript.addEntry(timeScriptEntryTwo);
      
      double time = 0.0;
      timeScript.doScript(time);
      
      assertEquals(initialValue, doubleVariable.getDoubleValue(), epsilon);
      
      for (time=0.0; time<10.0; time=time+0.001)
      {
         timeScript.doScript(time);
         
         if (time < timeOne)
         {
            assertEquals(initialValue, doubleVariable.getDoubleValue(), epsilon);
         }
         else if (time < timeTwo)
         {
            assertEquals(valueOne, doubleVariable.getDoubleValue(), epsilon);
         }
         else if (time < timeThree)
         {
            assertEquals(valueTwo, doubleVariable.getDoubleValue(), epsilon);
         }
         else
         {
            assertEquals(valueThree, doubleVariable.getDoubleValue(), epsilon);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testTimeScriptCommand()
   {
      TimeScript timeScript = new TimeScript(rootRegistry);

      double timeOne = 1.0;
      double timeTwo = 2.2;
      double timeThree = 3.3;

      double initialValue = 0.3;
      final double valueOne = 3.7;
      final double valueTwo = 5.6;
      final double valueThree = 9.9;
      
      doubleVariable.set(initialValue);
      
      TimeScriptEntry timeScriptEntryOne = new TimeScriptEntry(timeOne);

      TimeScriptCommand timeScriptCommandOne = new TimeScriptCommand()
      {
         @Override
         public void doCommand()
         {  
            doubleVariable.set(valueOne);
         }
      };
      
      timeScriptEntryOne.addTimeScriptCommand(timeScriptCommandOne);
      timeScript.addEntry(timeScriptEntryOne);
      
      
      TimeScriptEntry timeScriptEntryTwo = new TimeScriptEntry(timeTwo);

      TimeScriptCommand timeScriptCommandTwo = new TimeScriptCommand()
      {
         @Override
         public void doCommand()
         {  
            doubleVariable.set(valueTwo);
         }
      };
      
      timeScriptEntryTwo.addTimeScriptCommand(timeScriptCommandTwo);
      timeScript.addEntry(timeScriptEntryTwo);
      
      TimeScriptEntry timeScriptEntryThree = new TimeScriptEntry(timeThree);

      TimeScriptCommand timeScriptCommandThree = new TimeScriptCommand()
      {
         @Override
         public void doCommand()
         {  
            doubleVariable.set(valueThree);
         }
      };
      
      timeScriptEntryThree.addTimeScriptCommand(timeScriptCommandThree);
      timeScript.addEntry(timeScriptEntryThree);
      
      double epsilon = 1e-10;
      double time = 0.0;
      timeScript.doScript(time);
      
      assertEquals(initialValue, doubleVariable.getDoubleValue(), epsilon);
      
      for (time=0.0; time<10.0; time=time+0.001)
      {
         timeScript.doScript(time);
         
         if (time < timeOne)
         {
            assertEquals(initialValue, doubleVariable.getDoubleValue(), epsilon);
         }
         else if (time < timeTwo)
         {
            assertEquals(valueOne, doubleVariable.getDoubleValue(), epsilon);
         }
         else if (time < timeThree)
         {
            assertEquals(valueTwo, doubleVariable.getDoubleValue(), epsilon);
         }
         else
         {
            assertEquals(valueThree, doubleVariable.getDoubleValue(), epsilon);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSaveAndLoad()
   {
      Random random = new Random(1776L);
      
      String doubleVariableName = "doubleVariable";
      String booleanVariableName = "booleanVariable";
      String integerVariableName = "integerVariable";
      String enumVariableName = "enumVariable";
      
      double[] times = new double[]{1.0, 2.1, 5.77, 12.44, 17.90993};
      double[] doubleValues = new double[times.length];
      boolean[] booleanValues = new boolean[times.length];
      int[] integerValues = new int[times.length];
      TimeScriptTestEnums[] enumValues = new TimeScriptTestEnums[times.length];
      
      for (int i=0; i<times.length; i++)
      {
            doubleValues[i] = 100.0 * random.nextDouble() - 50.0;
            booleanValues[i] = random.nextBoolean();
            integerValues[i] = random.nextInt();
            enumValues[i] = TimeScriptTestEnums.values()[random.nextInt(TimeScriptTestEnums.values().length)];
      }
      
      TimeScript timeScript = new TimeScript(rootRegistry);
    
      String pseudoFile = "";
      
      for (int i=0; i<times.length; i++)
      {
         pseudoFile = pseudoFile + "t = " + times[i] + ":\n";
         pseudoFile = pseudoFile + "/* This is a comment!! */\n";

         pseudoFile = pseudoFile + doubleVariableName + " = " + doubleValues[i] + ";\n";
         pseudoFile = pseudoFile + "// This is a comment!! \n";
         pseudoFile = pseudoFile + booleanVariableName + " = " + booleanValues[i] + ";\n";
         pseudoFile = pseudoFile + "// This is a comment!! \n";
         pseudoFile = pseudoFile + integerVariableName + " = " + integerValues[i] + ";\n";
         pseudoFile = pseudoFile + "// This is a comment!! \n";
         pseudoFile = pseudoFile + enumVariableName + " = " + enumValues[i] + ";\n";
      }
      
//      System.out.println(pseudoFile);
      
      StringReader reader = new StringReader(pseudoFile);
      BufferedReader in = new BufferedReader(reader);
      
      timeScript.readTimeScript(rootRegistry, in);
      
      int timeIndex = -1;
      
      for (double time=0.0; time<times[times.length-1] + 1.0; time=time+0.01)
      {
         timeScript.doScript(time);
         
         if ((timeIndex < times.length-1) && (time >= times[timeIndex+1]))
         {
            timeIndex++;
         }
         
         if (timeIndex >= 0)
         {
            assertEquals(doubleValues[timeIndex], doubleVariable.getDoubleValue(), 1e-7);
            assertEquals(booleanValues[timeIndex], booleanVariable.getBooleanValue());
            assertEquals(integerValues[timeIndex], integerVariable.getIntegerValue());
            assertEquals(enumValues[timeIndex], enumVariable.getEnumValue());   
         }
      }
      
   }
   

}
