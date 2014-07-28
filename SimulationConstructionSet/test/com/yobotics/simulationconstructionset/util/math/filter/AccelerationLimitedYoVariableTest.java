package com.yobotics.simulationconstructionset.util.math.filter;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.utilities.ThreadTools;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.simulatedSensors.ProcessedSensorsReadWrite;
import com.yobotics.simulationconstructionset.util.math.functionGenerator.YoFunctionGenerator;
import com.yobotics.simulationconstructionset.util.math.functionGenerator.YoFunctionGeneratorMode;

public class AccelerationLimitedYoVariableTest
{
   public static final double EPSILON = 1e-8;
   private final Random random = new Random(3456456456456L);

   @Test
   public void testAccelerationLimitedYoVariable()
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("test"));
      double dt = 0.006;

      scs.startOnAThread();

      YoVariableRegistry registry = scs.getRootRegistry();
      DoubleYoVariable maxRate = new DoubleYoVariable("maxRate", registry);
      DoubleYoVariable maxAcceleration = new DoubleYoVariable("maxAcceleration", registry);
      DoubleYoVariable variable = new DoubleYoVariable("variable", registry);
      String name = "accelerationLimitedYoVariable";

      AccelerationLimitedYoVariable accelerationLimitedYoVariable = new AccelerationLimitedYoVariable(name, registry, maxRate, maxAcceleration, dt);

      try
      {
         DoubleYoVariable doubleYoVariable = new DoubleYoVariable(name, registry);
         fail();
      }
      catch (RuntimeException rte)
      {
         System.err.println("The previous namespace error was part of the AccelerationLimitedYoVariableTest.");
      }

      assertEquals(name, accelerationLimitedYoVariable.getName());
   }

   @Test
   public void testAccelerationLimitedYoVariable_without_Input_Variable()
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("test"));
      double dt = 0.006;

      scs.startOnAThread();

      YoVariableRegistry registry = scs.getRootRegistry();
      DoubleYoVariable maxRate = new DoubleYoVariable("maxRate", registry);
      DoubleYoVariable maxAcceleration = new DoubleYoVariable("maxAcceleration", registry);
      String name = "accelerationLimitedYoVariable";

      AccelerationLimitedYoVariable accelerationLimitedYoVariable = new AccelerationLimitedYoVariable(name, registry, maxRate, maxAcceleration, dt);

      try
      {
         DoubleYoVariable doubleYoVariable = new DoubleYoVariable(name, registry);
         fail();
      }
      catch (RuntimeException rte)
      {
         System.err.println("The previous namespace error was part of the AccelerationLimitedYoVariableTest.");
      }

      assertEquals(name, accelerationLimitedYoVariable.getName());
   }

   /*
    * To view this test properly in SCS, create three graphs and add: to the
    * first: raw & processed, 
    * to the second: rawRate, processedSmoothedRate, and max_Rate, 
    * to the third: processedSmoothAcceleration & max_Acceleration
    */
//   @Ignore
   @Test
   public void testSetMaximumAcceleration()
   {
      double maxAcceleration = 10.0;
      double maxRate = 1.0;
      double iteratorIncrement = 0.0005;
      double dt = 0.001;
      double permissibleErrorRatio = 0.1;
      Robot robot = new Robot("generic_robot");
      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      YoVariableRegistry registry = new YoVariableRegistry("generic_registry");
      scs.getRootRegistry().addChild(registry);
      DoubleYoVariable maxRateYo = new DoubleYoVariable("max_Rate", registry);
      boolean notifyListeners = true;
      maxRateYo.set(maxRate, notifyListeners);
      DoubleYoVariable maxAccelerationYo = new DoubleYoVariable("max_Acceleration", registry);
      maxAccelerationYo.set(maxAcceleration, notifyListeners);
      AccelerationLimitedYoVariable processed = new AccelerationLimitedYoVariable("processed", registry, maxRateYo, maxAccelerationYo, dt);
      DoubleYoVariable raw = new DoubleYoVariable("raw", registry);
      DoubleYoVariable alphaVariable = new DoubleYoVariable("alpha", registry);
      FilteredVelocityYoVariable rawRate = new FilteredVelocityYoVariable("rawRate", "", alphaVariable, raw, dt, registry);

      int bufferSize = (int) (maxAcceleration / iteratorIncrement) + 1;
      scs.changeBufferSize(bufferSize);

      String[] vars = {"raw", "processed", "max_Rate", "rawRate", "processedSmoothedRate", "processedSmoothedAcceleration", "max_Acceleration"};
      scs.setupVarGroup("Acceleration Var Group", vars);
      
      String[][] graphGroupVars = { { "raw", "processed"}, { "max_Rate", "rawRate", "processedSmoothedRate" }, { "processedSmoothedAcceleration", "max_Acceleration" } };
      scs.setupGraphGroup("GraphGroup for Acceleration", graphGroupVars);
      
      String[] entryBoxGroupVars = { "raw", "processed", "max_Rate", "rawRate", "processedSmoothedRate", "processedSmoothedAcceleration", "max_Acceleration" };
      scs.setupEntryBoxGroup("Acceleration Entry Box", entryBoxGroupVars);
      
      scs.setupConfiguration("Acceleration Config", "Acceleration Var Group", "GraphGroup for Acceleration", "Acceleration Entry Box");
      scs.selectConfiguration("Acceleration Config");
      
      for (double i = 0.0; i < 10.0; i += iteratorIncrement)
      {
         raw.set(i);
         rawRate.update();
         processed.update(i);
         scs.tickAndUpdate();
         if(i > 1.0)
         {
            assertTrue(isValueWithinMarginOfError(raw.getDoubleValue(), processed.getDoubleValue(), permissibleErrorRatio));
//            assertTrue(isValueWithinMarginOfError(rawRate.getDoubleValue(), processedSmoothedRate.getDoubleValue(), permissibleErrorRatio));
            
         }
      }
      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   @Test
   public void testSetMaximumRate()
   {

   }

   @Test
   public void testSetGainsByPolePlacement()
   {

   }

   @Test
   public void testUpdate()
   {

   }

   @Test
   public void testUpdate_double()
   {

   }

   @Test
   public void testInitialize()
   {

   }

   @Test
   public void testReset()
   {

   }

   @Ignore
   @Test
   public void testDump()
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("test"));
      double dt = 0.006;

      scs.startOnAThread();

      YoVariableRegistry registry = scs.getRootRegistry();
      DoubleYoVariable time = new DoubleYoVariable("time", registry);
      DoubleYoVariable maxRate = new DoubleYoVariable("maxRate", registry);
      DoubleYoVariable maxAcceleration = new DoubleYoVariable("maxAcceleration", registry);

      DoubleYoVariable variable = new DoubleYoVariable("variable", registry);

      AccelerationLimitedYoVariable smoothedYoVariable = new AccelerationLimitedYoVariable("smoothedVariable", registry, maxRate, maxAcceleration, dt);

      double breakFrequencyHertz = 20.0; //16.0;
      smoothedYoVariable.setGainsByPolePlacement(2.0 * Math.PI * breakFrequencyHertz, 1.0);
      smoothedYoVariable.setMaximumAcceleration(400.0);
      smoothedYoVariable.setMaximumRate(12.0);

      YoFunctionGenerator functionGenerator = new YoFunctionGenerator("functionGenerator", registry);
      functionGenerator.setMode(YoFunctionGeneratorMode.CHIRP_LINEAR);
      //      functionGenerator.setMode(YoFunctionGeneratorMode.DC);      
      functionGenerator.setChirpFrequencyMaxHz(30.0);
      functionGenerator.setResetTime(20.0);

      double maximumOmega0 = 200.0 / 6.0; //240.0/8.0;
      System.out.println("maximumOmega0 = " + maximumOmega0 + " = " + maximumOmega0 / (2.0 * Math.PI) + " hertz.");

      smoothedYoVariable.update(0.0);

      double amplitude = 0.1;
      variable.set(amplitude);
      functionGenerator.setAmplitude(amplitude);

      for (time.set(0.0); time.getDoubleValue() < 0.2; time.add(dt))
      {
         smoothedYoVariable.update(variable.getDoubleValue());
         scs.tickAndUpdate();
      }

      variable.set(0.0);

      for (; time.getDoubleValue() < 1.0; time.add(dt))
      {
         smoothedYoVariable.update(variable.getDoubleValue());
         scs.tickAndUpdate();
      }

      for (; time.getDoubleValue() < 20.0; time.add(dt))
      {
         variable.set(functionGenerator.getValue(time.getDoubleValue()));

         smoothedYoVariable.update(variable.getDoubleValue());
         scs.tickAndUpdate();
      }

      scs.cropBuffer();
      ThreadTools.sleepForever();
   }

   private boolean isValueWithinMarginOfError(double correctValue, double valueInQuestion, double permissibleErrorRatio)
   {
      double result = Math.abs(correctValue - valueInQuestion) / correctValue;

      if (result <= permissibleErrorRatio)
      {
         return true;
      }
      else
      {
         System.out.println("result" + result);
         return false;
      }
   }
}