package com.yobotics.simulationconstructionset.util.math.filter;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.utilities.ThreadTools;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.functionGenerator.YoFunctionGenerator;
import com.yobotics.simulationconstructionset.util.math.functionGenerator.YoFunctionGeneratorMode;

public class AccelerationLimitedYoVariableTest
{
   private static final boolean VISUALIZE = false;
   private static final boolean KEEPWINDOWSOPEN = true;
   private static final long SLEEPTIME = 5000;

   private static final double EPSILON = 1e-8;
   private static double maxAcceleration = 10.0;
   private static double maxRate = 1.0;
   private static final double dt = 0.001;
   private static final double totalTime = 10.0;
   private static double amplitude = 1.0;
   private static double frequency = 1.0;
   private static final double permissibleErrorRatio = 0.1;
   private static final boolean notifyListeners = true;
   private static final int bufferSize = (int) (maxAcceleration / (dt * 0.5)) + 1;
   private final Robot robot = new Robot("generic_robot");
   private String nameYo = "processed";
   private SimulationConstructionSet scs;
   private YoVariableRegistry registry;

   /*
    * Can't use @Before because not every test uses the GUI.
    */
   private void setupSCSStuff()
   {
      registry = new YoVariableRegistry("generic_registry");

      if (VISUALIZE)
      {
         scs = new SimulationConstructionSet(robot);
         scs.changeBufferSize(bufferSize);

         String[] vars = { "raw", nameYo, "max_Rate", "rawRate", "processedSmoothedRate", "max_Acceleration", "processedSmoothedAcceleration" };
         scs.setupVarGroup("Acceleration Var Group", vars);

         String[][] graphGroupVars = { { "raw", nameYo }, { "max_Rate", "rawRate", "processedSmoothedRate" },
               { "max_Acceleration", "processedSmoothedAcceleration" } };
         scs.setupGraphGroup("GraphGroup for Acceleration", graphGroupVars);

         String[] entryBoxGroupVars = { "raw", nameYo, "max_Rate", "rawRate", "processedSmoothedRate", "processedSmoothedAcceleration", "max_Acceleration" };
         scs.setupEntryBoxGroup("Acceleration Entry Box", entryBoxGroupVars);

         scs.setupConfiguration("Acceleration Config", "Acceleration Var Group", "GraphGroup for Acceleration", "Acceleration Entry Box");
         scs.selectConfiguration("Acceleration Config");

         scs.getRootRegistry().addChild(registry);
      }
   }

//   @Before
//   public void makeSureGUIIsNotUpWhenRunning()
//   {
//      if ( VISUALIZE) throw new RuntimeException(this.getClass() + "was checked in with the GUI enabled."); 
//   }
   
   /*
    * Can't use @After because not every test uses the GUI.
    */
   private void shutdownSCSStuff(SimulationConstructionSet scs)
   {
      if (VISUALIZE)
      {
         scs.startOnAThread();

         if (KEEPWINDOWSOPEN)
         {
            ThreadTools.sleepForever();
         }
         else
         {
            ThreadTools.sleep(SLEEPTIME);
         }
         scs.closeAndDispose();
      }
   }

   @Test
   public void testSetMaximumRateAndAcceleration_ConstantVelocity()
   {
      setupSCSStuff();

      DoubleYoVariable maxRateYo = new DoubleYoVariable("max_Rate", registry);
      maxRateYo.set(maxRate, notifyListeners);
      DoubleYoVariable maxAccelerationYo = new DoubleYoVariable("max_Acceleration", registry);
      maxAccelerationYo.set(maxAcceleration, notifyListeners);
      AccelerationLimitedYoVariable processed = new AccelerationLimitedYoVariable("processed", registry, maxRateYo, maxAccelerationYo, dt);
      processed.setMaximumAcceleration(maxAcceleration);
      processed.setMaximumRate(maxRate);

      try
      {
         @SuppressWarnings("unused")
         DoubleYoVariable doubleYoVariable = new DoubleYoVariable(nameYo, registry);
         fail();
      }
      catch (RuntimeException rte)
      {
         System.err.println("The previous namespace error was part of the AccelerationLimitedYoVariableTest.");
      }

      assertEquals(nameYo, processed.getName());

      DoubleYoVariable raw = new DoubleYoVariable("raw", registry);
      DoubleYoVariable alphaVariable = new DoubleYoVariable("alpha", registry);
      FilteredVelocityYoVariable rawRate = new FilteredVelocityYoVariable("rawRate", "", alphaVariable, raw, dt, registry);

      for (double time = 0.0; time < totalTime; time += dt * 0.5)
      {
         raw.set(time);
         rawRate.update();
         processed.update(time);
         if (VISUALIZE)
            scs.tickAndUpdate();

         assertTrue(maxRate >= processed.getSmoothedRate().getDoubleValue());
         assertTrue(maxAcceleration >= processed.getSmoothedAcceleration().getDoubleValue());

         if (time > 0.3 * totalTime)
         {
            //            assertTrue(isValueWithinMarginOfError(raw.getDoubleValue(), processed.getDoubleValue(), permissibleErrorRatio));
            //            assertTrue(isValueWithinMarginOfError(rawRate.getDoubleValue(), processed.getSmoothedRate().getDoubleValue(), permissibleErrorRatio));
            //            assertTrue(isValueWithinMarginOfError(0.0, processed.getSmoothedAcceleration().getDoubleValue(), permissibleErrorRatio));
         }
      }
      shutdownSCSStuff(scs);
   }

   @Test
   public void testSetMaximumRateAndAcceleration_Sine() //tests update(double argument)
   {
      setupSCSStuff();

      DoubleYoVariable maxRateYo = new DoubleYoVariable("max_Rate", registry);
      maxRateYo.set(maxRate, notifyListeners);
      DoubleYoVariable maxAccelerationYo = new DoubleYoVariable("max_Acceleration", registry);
      maxAccelerationYo.set(maxAcceleration, notifyListeners);
      AccelerationLimitedYoVariable processed = new AccelerationLimitedYoVariable(nameYo, registry, maxRateYo, maxAccelerationYo, dt);
      DoubleYoVariable raw = new DoubleYoVariable("raw", registry);
      DoubleYoVariable alphaVariable = new DoubleYoVariable("alpha", registry);
      FilteredVelocityYoVariable rawRate = new FilteredVelocityYoVariable("rawRate", "", alphaVariable, raw, dt, registry);

      DoubleYoVariable timeYo = new DoubleYoVariable("time", registry);
      YoFunctionGenerator sineFunction = new YoFunctionGenerator("sineFunction", timeYo, registry);
      sineFunction.setMode(YoFunctionGeneratorMode.SINE);
      sineFunction.setFrequency(frequency);
      sineFunction.setAmplitude(amplitude);

      double value = 0.0;

      for (double time = 0.0; time < totalTime; time += dt * 0.5)
      {
         timeYo.set(time);
         value = sineFunction.getValue();

         raw.set(value);
         rawRate.update();
         processed.update(value);
         if (VISUALIZE)
            scs.tickAndUpdate();

         assertTrue(maxRate >= processed.getSmoothedRate().getDoubleValue());
         assertTrue(maxAcceleration >= processed.getSmoothedAcceleration().getDoubleValue());

         assertTrue(Math.abs(processed.getDoubleValue()) <= amplitude);
         assertTrue(Math.abs(processed.getSmoothedRate().getDoubleValue()) <= maxRate);
         assertTrue(Math.abs(processed.getSmoothedAcceleration().getDoubleValue()) <= maxAcceleration);
      }
      shutdownSCSStuff(scs);
   }

   @Test
   public void testSetMaximumRateAndAcceleration_RiseTimeSquareWave()
   {
      setupSCSStuff();

      DoubleYoVariable maxRateYo = new DoubleYoVariable("max_Rate", registry);
      maxRate = 10.0; 
      maxRateYo.set(maxRate, notifyListeners);
      DoubleYoVariable maxAccelerationYo = new DoubleYoVariable("max_Acceleration", registry);
      maxAcceleration = 1.0;
      maxAccelerationYo.set(maxAcceleration, notifyListeners);
      AccelerationLimitedYoVariable processed = new AccelerationLimitedYoVariable(nameYo, registry, maxRateYo, maxAccelerationYo, dt);
      DoubleYoVariable raw = new DoubleYoVariable("raw", registry);
      DoubleYoVariable alphaVariable = new DoubleYoVariable("alpha", registry);
      FilteredVelocityYoVariable rawRate = new FilteredVelocityYoVariable("rawRate", "", alphaVariable, raw, dt, registry);

      DoubleYoVariable timeYo = new DoubleYoVariable("time", registry);
      amplitude = 1.0;

      double value = 0.0;

      for (double time = 0.0; time < totalTime; time += dt * 0.5)
      {
         timeYo.set(time);
         value = (time < 0.5 * totalTime) ? 0.0 : amplitude;

         raw.set(value);
         rawRate.update();
         processed.update(value);
         
         if (VISUALIZE)
            scs.tickAndUpdate();

         assertTrue(maxRate >= processed.getSmoothedRate().getDoubleValue());
         assertTrue(maxAcceleration >= processed.getSmoothedAcceleration().getDoubleValue());

//         assertTrue(Math.abs(processed.getDoubleValue()) <= amplitude);
//         assertTrue(Math.abs(processed.getSmoothedRate().getDoubleValue()) <= maxRate);
//         assertTrue(Math.abs(processed.getSmoothedAcceleration().getDoubleValue()) <= maxAcceleration);
      }
      shutdownSCSStuff(scs);
   }

   @Test
   public void testSetMaximumRateAndAcceleration_SquareWaves() //tests update(double argument)
   {
      setupSCSStuff();

      DoubleYoVariable maxRateYo = new DoubleYoVariable("max_Rate", registry);
      maxRateYo.set(maxRate, notifyListeners);
      DoubleYoVariable maxAccelerationYo = new DoubleYoVariable("max_Acceleration", registry);
      maxAccelerationYo.set(maxAcceleration, notifyListeners);
      AccelerationLimitedYoVariable processed = new AccelerationLimitedYoVariable(nameYo, registry, maxRateYo, maxAccelerationYo, dt);
      DoubleYoVariable raw = new DoubleYoVariable("raw", registry);
      DoubleYoVariable alphaVariable = new DoubleYoVariable("alpha", registry);
      FilteredVelocityYoVariable rawRate = new FilteredVelocityYoVariable("rawRate", "", alphaVariable, raw, dt, registry);

      DoubleYoVariable timeYo = new DoubleYoVariable("time", registry);
      YoFunctionGenerator squareFunction = new YoFunctionGenerator("sineFunction", timeYo, registry);
      squareFunction.setMode(YoFunctionGeneratorMode.SQUARE);
      squareFunction.setFrequency(frequency);
      squareFunction.setAmplitude(amplitude);

      double value = 0.0;

      for (double time = 0.0; time < totalTime; time += dt * 0.5)
      {
         timeYo.set(time);
         value = squareFunction.getValue();

         raw.set(value);
         rawRate.update();
         processed.update(value);
         if (VISUALIZE)
            scs.tickAndUpdate();

//         assertTrue(maxRate >= processed.getSmoothedRate().getDoubleValue());
//         assertTrue(maxAcceleration >= processed.getSmoothedAcceleration().getDoubleValue());
//
//         assertTrue(Math.abs(processed.getDoubleValue()) <= amplitude);
//         assertTrue(Math.abs(processed.getSmoothedRate().getDoubleValue()) <= maxRate);
//         assertTrue(Math.abs(processed.getSmoothedAcceleration().getDoubleValue()) <= maxAcceleration);
      }
      shutdownSCSStuff(scs);
   }

   @Test
   public void testSetMaximumRateAndAcceleration_Chirp() //tests update() with no arguments
   {
      setupSCSStuff();

      DoubleYoVariable maxRateYo = new DoubleYoVariable("max_Rate", registry);
      maxRateYo.set(maxRate, notifyListeners);
      DoubleYoVariable maxAccelerationYo = new DoubleYoVariable("max_Acceleration", registry);
      maxAccelerationYo.set(maxAcceleration, notifyListeners);
      DoubleYoVariable inputVariable = new DoubleYoVariable("inputVariable", registry);
      AccelerationLimitedYoVariable processed = new AccelerationLimitedYoVariable(nameYo, registry, maxRateYo, maxAccelerationYo, inputVariable, dt);
      DoubleYoVariable raw = new DoubleYoVariable("raw", registry);
      DoubleYoVariable alphaVariable = new DoubleYoVariable("alpha", registry);
      FilteredVelocityYoVariable rawRate = new FilteredVelocityYoVariable("rawRate", "", alphaVariable, raw, dt, registry);

      DoubleYoVariable timeYo = new DoubleYoVariable("time", registry);
      YoFunctionGenerator chirpFunction = new YoFunctionGenerator("sineFunction", timeYo, registry);
      chirpFunction.setMode(YoFunctionGeneratorMode.CHIRP_LINEAR);
      chirpFunction.setAmplitude(amplitude);

      double value = 0.0;

      for (double time = 0.0; time < totalTime; time += dt * 0.5)
      {
         timeYo.set(time);
         value = chirpFunction.getValue();

         raw.set(value);
         rawRate.update();
         inputVariable.set(value);
         processed.update();
         if (VISUALIZE)
            scs.tickAndUpdate();

         assertTrue(maxRate >= processed.getSmoothedRate().getDoubleValue());
         assertTrue(maxAcceleration >= processed.getSmoothedAcceleration().getDoubleValue());

         assertTrue(Math.abs(processed.getDoubleValue()) <= amplitude
               || isValueWithinMarginOfError(amplitude, processed.getDoubleValue(), permissibleErrorRatio));
         assertTrue(Math.abs(processed.getSmoothedRate().getDoubleValue()) <= maxRate);
         assertTrue(Math.abs(processed.getSmoothedAcceleration().getDoubleValue()) <= maxAcceleration);
      }
      shutdownSCSStuff(scs);
   }

   @Ignore
   @Test
   public void testSetGainsByPolePlacement()
   {
   }

   @Test
   public void testInitialize()
   {
      YoVariableRegistry registry = new YoVariableRegistry("generic_registry");

      DoubleYoVariable maxRateYo = new DoubleYoVariable("max_Rate", registry);
      maxRateYo.set(maxRate, notifyListeners);
      DoubleYoVariable maxAccelerationYo = new DoubleYoVariable("max_Acceleration", registry);
      maxAccelerationYo.set(maxAcceleration, notifyListeners);
      AccelerationLimitedYoVariable processed = new AccelerationLimitedYoVariable(nameYo, registry, maxRateYo, maxAccelerationYo, dt);
      DoubleYoVariable raw = new DoubleYoVariable("raw", registry);
      DoubleYoVariable alphaVariable = new DoubleYoVariable("alpha", registry);
      FilteredVelocityYoVariable rawRate = new FilteredVelocityYoVariable("rawRate", "", alphaVariable, raw, dt, registry);

      DoubleYoVariable timeYo = new DoubleYoVariable("time", registry);
      YoFunctionGenerator squareFunction = new YoFunctionGenerator("sineFunction", timeYo, registry);
      squareFunction.setMode(YoFunctionGeneratorMode.SQUARE);
      squareFunction.setFrequency(frequency);
      squareFunction.setAmplitude(amplitude);

      double value = 0.0;

      for (double time = 0.0; time < totalTime; time += dt * 0.5)
      {
         timeYo.set(time);
         value = squareFunction.getValue();

         raw.set(value);
         rawRate.update();
         processed.update(value);

         assertTrue(maxRate >= processed.getSmoothedRate().getDoubleValue());
         assertTrue(maxAcceleration >= processed.getSmoothedAcceleration().getDoubleValue());

         assertTrue(Math.abs(processed.getDoubleValue()) <= amplitude);
         assertTrue(Math.abs(processed.getSmoothedRate().getDoubleValue()) <= maxRate);
         assertTrue(Math.abs(processed.getSmoothedAcceleration().getDoubleValue()) <= maxAcceleration);
      }

      processed.initialize(3.0);

      //After initialize
      assertEquals(3.0, processed.getDoubleValue(), EPSILON);
      assertTrue(processed.hasBeenInitialized());
      assertEquals(0.0, processed.getSmoothedRate().getDoubleValue(), EPSILON);
      assertEquals(0.0, processed.getSmoothedAcceleration().getDoubleValue(), EPSILON);
   }

   @Test
   public void testReset()
   {
      YoVariableRegistry registry = new YoVariableRegistry("generic_registry");

      DoubleYoVariable maxRateYo = new DoubleYoVariable("max_Rate", registry);
      maxRateYo.set(maxRate, notifyListeners);
      DoubleYoVariable maxAccelerationYo = new DoubleYoVariable("max_Acceleration", registry);
      maxAccelerationYo.set(maxAcceleration, notifyListeners);
      AccelerationLimitedYoVariable processed = new AccelerationLimitedYoVariable(nameYo, registry, maxRateYo, maxAccelerationYo, dt);
      DoubleYoVariable raw = new DoubleYoVariable("raw", registry);
      DoubleYoVariable alphaVariable = new DoubleYoVariable("alpha", registry);
      FilteredVelocityYoVariable rawRate = new FilteredVelocityYoVariable("rawRate", "", alphaVariable, raw, dt, registry);

      DoubleYoVariable timeYo = new DoubleYoVariable("time", registry);
      YoFunctionGenerator squareFunction = new YoFunctionGenerator("sineFunction", timeYo, registry);
      squareFunction.setMode(YoFunctionGeneratorMode.SQUARE);
      squareFunction.setFrequency(frequency);
      squareFunction.setAmplitude(amplitude);

      double value = 0.0;

      for (double time = 0.0; time < totalTime; time += dt * 0.5)
      {
         timeYo.set(time);
         value = squareFunction.getValue();

         raw.set(value);
         rawRate.update();
         processed.update(value);

         assertTrue(maxRate >= processed.getSmoothedRate().getDoubleValue());
         assertTrue(maxAcceleration >= processed.getSmoothedAcceleration().getDoubleValue());

         assertTrue(Math.abs(processed.getDoubleValue()) <= amplitude);
         assertTrue(Math.abs(processed.getSmoothedRate().getDoubleValue()) <= maxRate);
         assertTrue(Math.abs(processed.getSmoothedAcceleration().getDoubleValue()) <= maxAcceleration);
      }

      processed.reset();

      //After reset
      assertFalse(processed.hasBeenInitialized());
      assertEquals(0.0, processed.getSmoothedRate().getDoubleValue(), EPSILON);
      assertEquals(0.0, processed.getSmoothedAcceleration().getDoubleValue(), EPSILON);
   }

   @Ignore
   @Test
   public void testDump()
   {
      double dt = 0.006;
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("test"));

      scs.startOnAThread();

      YoVariableRegistry registry = scs.getRootRegistry();
      DoubleYoVariable time = new DoubleYoVariable("time", registry);
      DoubleYoVariable maxRateYo = new DoubleYoVariable("maxRate", registry);
      DoubleYoVariable maxAccelerationYo = new DoubleYoVariable("maxAcceleration", registry);

      DoubleYoVariable variable = new DoubleYoVariable("variable", registry);

      AccelerationLimitedYoVariable smoothedYoVariable = new AccelerationLimitedYoVariable("smoothedVariable", registry, maxRateYo, maxAccelerationYo, dt);

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

   @Test
   public void testErrorTooHigh()
   {
      assertFalse(isValueWithinMarginOfError(1.0, 0.0, 0.1));
      assertTrue(isValueWithinMarginOfError(1.0, 0.9, 0.11));
   }

   private boolean isValueWithinMarginOfError(double correctValue, double valueInQuestion, double permissibleErrorRatio)
   {
      double denominator = (correctValue == 0.0) ? EPSILON : correctValue;
      double result = Math.abs(correctValue - valueInQuestion) / denominator;

      if (result <= permissibleErrorRatio)
      {
         return true;
      }
      else
      {
         System.out.println("result = " + result);
         return false;
      }
   }
}