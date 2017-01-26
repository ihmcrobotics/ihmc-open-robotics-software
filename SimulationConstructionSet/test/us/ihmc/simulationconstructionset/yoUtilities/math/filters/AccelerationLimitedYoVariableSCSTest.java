package us.ihmc.simulationconstructionset.yoUtilities.math.filters;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.AccelerationLimitedYoVariable;
import us.ihmc.robotics.math.filters.FilteredVelocityYoVariable;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.thread.ThreadTools;

public class AccelerationLimitedYoVariableSCSTest
{
   private static final boolean VISUALIZE = false;
   private static final boolean KEEPWINDOWSOPEN = false;
   private static final long SLEEPTIME = 10000;

   private static final double EPSILON = 1e-8;

   private static double maxRate = 10.0;
   private static double maxAcceleration = 10.0;
   private static final double dt = 0.001;
   private static final double totalTime = 10.0;

   DoubleYoVariable raw;
   DoubleYoVariable alphaVariable;
   FilteredVelocityYoVariable rawRate;
   FilteredVelocityYoVariable rawAcceleration;

   private static double amplitude = 1.0;
   private static double frequency = 1.0;

   private static final double permissibleErrorRatio = 0.1;

   private static final boolean notifyListeners = true;
   private static final int bufferSize = (int) (totalTime / dt) + 5; //make sure it fits in the graph
   private final Robot robot = new Robot("generic_robot");
   private String nameYo = "processed";
   private SimulationConstructionSet scs;
   private YoVariableRegistry registry;
   private Random random = new Random(4546556L);

   /*
    * Can't use @Before because not every test uses the GUI.
    */
   private void setupSCSStuff()
   {
      registry = new YoVariableRegistry("generic_registry");

      raw = new DoubleYoVariable("raw", registry);
      alphaVariable = new DoubleYoVariable("alpha", registry);
      alphaVariable.set(0.0); //set to zero to prevent smoothing of velocity
      rawRate = new FilteredVelocityYoVariable("rawRate", "", alphaVariable, raw, dt, registry);
      rawAcceleration = new FilteredVelocityYoVariable("rawAcceleration", "", alphaVariable, rawRate, dt, registry);

      if (VISUALIZE)
      {
         scs = new SimulationConstructionSet(robot);
         scs.changeBufferSize(bufferSize);

         String[] vars = { "raw", nameYo, "max_Rate", "rawRate", "processedSmoothedRate", "rawAcceleration", "max_Acceleration",
               "processedSmoothedAcceleration" };
         scs.setupVarGroup("Acceleration Var Group", vars);

         String[][] graphGroupVars = { { "raw", nameYo }, { "max_Rate", "rawRate", "processedSmoothedRate" },
               { "max_Acceleration", "rawAcceleration", "processedSmoothedAcceleration" } };
         scs.setupGraphGroup("GraphGroup for Acceleration", graphGroupVars);

         String[] entryBoxGroupVars = { "raw", nameYo, "max_Rate", "rawRate", "processedSmoothedRate", "rawAcceleration", "processedSmoothedAcceleration",
               "max_Acceleration" };
         scs.setupEntryBoxGroup("Acceleration Entry Box", entryBoxGroupVars);

         scs.setupConfiguration("Acceleration Config", "Acceleration Var Group", "GraphGroup for Acceleration", "Acceleration Entry Box");
         scs.selectConfiguration("Acceleration Config");

         scs.getRootRegistry().addChild(registry);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
         public void makeSureGUIIsNotUpWhenRunning()
         {
            if ( VISUALIZE) throw new RuntimeException(this.getClass() + "was checked in with the GUI enabled. Better fix that."); 
         }

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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void test_ZeroVelocity()
   {
      setupSCSStuff();

      maxRate = Double.POSITIVE_INFINITY;
      maxAcceleration = Double.POSITIVE_INFINITY;
      DoubleYoVariable maxRateYo = new DoubleYoVariable("max_Rate", registry);
      maxRateYo.set(maxRate, notifyListeners);
      DoubleYoVariable maxAccelerationYo = new DoubleYoVariable("max_Acceleration", registry);
      maxAccelerationYo.set(maxAcceleration, notifyListeners);
      AccelerationLimitedYoVariable processed = new AccelerationLimitedYoVariable("processed", registry, maxRateYo, maxAccelerationYo, dt);

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

      double constant = random.nextDouble();

      for (double time = 0.0; time < totalTime; time += dt)
      {
         raw.set(constant);
         rawRate.update(raw.getDoubleValue());
         rawAcceleration.update(rawRate.getDoubleValue());
         processed.update(raw.getDoubleValue());

         if (VISUALIZE)
            scs.tickAndUpdate();

         assertTrue(maxRate >= processed.getSmoothedRate().getDoubleValue());
         assertTrue(maxAcceleration >= processed.getSmoothedAcceleration().getDoubleValue());

         assertEquals(0.0, processed.getSmoothedRate().getDoubleValue(), EPSILON);
         assertEquals(0.0, processed.getSmoothedAcceleration().getDoubleValue(), EPSILON);

      }
      shutdownSCSStuff(scs);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void test_ConstantVelocity()
   {
      setupSCSStuff();

      maxRate = 5.0;
      maxAcceleration = 5.0;
      DoubleYoVariable maxRateYo = new DoubleYoVariable("max_Rate", registry);
      maxRateYo.set(maxRate, notifyListeners);
      DoubleYoVariable maxAccelerationYo = new DoubleYoVariable("max_Acceleration", registry);
      maxAccelerationYo.set(maxAcceleration, notifyListeners);
      AccelerationLimitedYoVariable processed = new AccelerationLimitedYoVariable("processed", registry, maxRateYo, maxAccelerationYo, dt);

      double value = random.nextDouble();

      for (double time = 0.0; time < totalTime; time += dt)
      {
         raw.set(value * time);
         rawRate.update(raw.getDoubleValue());
         rawAcceleration.update(rawRate.getDoubleValue());
         processed.update(raw.getDoubleValue());

         if (VISUALIZE)
            scs.tickAndUpdate();

         assertTrue(maxRate >= processed.getSmoothedRate().getDoubleValue());
         assertTrue(maxAcceleration >= processed.getSmoothedAcceleration().getDoubleValue());

         if (time > 0.3 * totalTime)
         {
            assertEquals(value, processed.getSmoothedRate().getDoubleValue(), EPSILON);
            assertEquals(0.0, processed.getSmoothedAcceleration().getDoubleValue(), EPSILON);
         }
      }
      shutdownSCSStuff(scs);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void test_ConstantAcceleration_PlusInitialize()
   {
      setupSCSStuff();

      double arbitraryMultiplier = 10.0;
      double constant = arbitraryMultiplier * random.nextDouble();

      maxRate = 2 * constant * totalTime;
      maxAcceleration = 2 * constant;
      DoubleYoVariable maxRateYo = new DoubleYoVariable("max_Rate", registry);
      maxRateYo.set(maxRate, notifyListeners);
      DoubleYoVariable maxAccelerationYo = new DoubleYoVariable("max_Acceleration", registry);
      maxAccelerationYo.set(maxAcceleration, notifyListeners);
      AccelerationLimitedYoVariable processed = new AccelerationLimitedYoVariable("processed", registry, maxRateYo, maxAccelerationYo, dt);

      assertEquals(nameYo, processed.getName());

      for (double time = 0.0; time < totalTime; time += dt)
      {
         raw.set(constant * time * time);
         rawRate.update(raw.getDoubleValue());
         rawAcceleration.update(rawRate.getDoubleValue());
         processed.update(raw.getDoubleValue());

         if (VISUALIZE)
            scs.tickAndUpdate();

         assertTrue(maxRate >= processed.getSmoothedRate().getDoubleValue());
         assertTrue(maxAcceleration >= processed.getSmoothedAcceleration().getDoubleValue());

         if (time > 1.0 * totalTime)
         {
            assertEquals(2 * constant * time, processed.getSmoothedRate().getDoubleValue(), EPSILON);
            assertEquals(2 * constant, processed.getSmoothedAcceleration().getDoubleValue(), EPSILON);
         }

         processed.initialize(3.0);

         //After initialize
         assertEquals(3.0, processed.getDoubleValue(), EPSILON);
         assertTrue(processed.hasBeenInitialized());
         assertEquals(0.0, processed.getSmoothedRate().getDoubleValue(), EPSILON);
         assertEquals(0.0, processed.getSmoothedAcceleration().getDoubleValue(), EPSILON);
      }
      shutdownSCSStuff(scs);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void test_Sine_Plus_Reset_Plus_Update()
   {
      setupSCSStuff();

      maxRate = 1.0;
      maxAcceleration = 1.0;
      DoubleYoVariable maxRateYo = new DoubleYoVariable("max_Rate", registry);
      maxRateYo.set(maxRate, notifyListeners);
      DoubleYoVariable maxAccelerationYo = new DoubleYoVariable("max_Acceleration", registry);
      maxAccelerationYo.set(maxAcceleration, notifyListeners);
      AccelerationLimitedYoVariable processed = new AccelerationLimitedYoVariable(nameYo, registry, maxRateYo, maxAccelerationYo, dt);

      DoubleYoVariable timeYo = new DoubleYoVariable("time", registry);
      YoFunctionGenerator sineFunction = new YoFunctionGenerator("sineFunction", timeYo, registry);
      sineFunction.setMode(YoFunctionGeneratorMode.SINE);
      frequency = 1.0;
      amplitude = 1.0;
      sineFunction.setFrequency(frequency);
      sineFunction.setAmplitude(amplitude);

      double value = 0.0;

      for (double time = 0.0; time < totalTime; time += dt)
      {
         timeYo.set(time);
         value = sineFunction.getValue();

         raw.set(value);
         rawRate.update(raw.getDoubleValue());
         rawAcceleration.update(rawRate.getDoubleValue());
         processed.update(raw.getDoubleValue());
         if (VISUALIZE)
            scs.tickAndUpdate();

         assertTrue(maxRate >= processed.getSmoothedRate().getDoubleValue());
         assertTrue(maxAcceleration >= processed.getSmoothedAcceleration().getDoubleValue());

         assertTrue(Math.abs(processed.getDoubleValue()) <= amplitude);
         assertTrue(Math.abs(processed.getSmoothedRate().getDoubleValue()) <= maxRate);
         assertTrue(Math.abs(processed.getSmoothedAcceleration().getDoubleValue()) <= maxAcceleration);

         processed.reset();

         //After reset
         assertFalse(processed.hasBeenInitialized());
         assertEquals(0.0, processed.getSmoothedRate().getDoubleValue(), EPSILON);
         assertEquals(0.0, processed.getSmoothedAcceleration().getDoubleValue(), EPSILON);
      }
      shutdownSCSStuff(scs);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void test_RiseTimeSquareWave()
   {
      setupSCSStuff();

      maxRate = 10000.0;
      maxAcceleration = 1000.0;
      DoubleYoVariable maxRateYo = new DoubleYoVariable("max_Rate", registry);
      maxRateYo.set(maxRate, notifyListeners);
      DoubleYoVariable maxAccelerationYo = new DoubleYoVariable("max_Acceleration", registry);
      maxAccelerationYo.set(maxAcceleration, notifyListeners);
      AccelerationLimitedYoVariable processed = new AccelerationLimitedYoVariable(nameYo, registry, maxRateYo, maxAccelerationYo, dt);

      DoubleYoVariable timeYo = new DoubleYoVariable("time", registry);
      amplitude = 1.0;

      double value = 0.0;

      for (double time = 0.0; time < totalTime; time += dt)
      {
         timeYo.set(time);

         if (time < 0.25 * totalTime)
         {
            value = 0.0;
         }
         else if (time < 0.5 * totalTime)
         {
            value = 1.0;
         }
         else
         {
            value = 0.0;
         }

         raw.set(value);
         rawRate.update(raw.getDoubleValue());
         rawAcceleration.update(rawRate.getDoubleValue());
         processed.update(raw.getDoubleValue());

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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void test_SquareWaves()
   {
      setupSCSStuff();

      maxRate = 10000.0;
      maxAcceleration = 1000.0;
      DoubleYoVariable maxRateYo = new DoubleYoVariable("max_Rate", registry);
      maxRateYo.set(maxRate, notifyListeners);
      DoubleYoVariable maxAccelerationYo = new DoubleYoVariable("max_Acceleration", registry);
      maxAccelerationYo.set(maxAcceleration, notifyListeners);
      AccelerationLimitedYoVariable processed = new AccelerationLimitedYoVariable(nameYo, registry, maxRateYo, maxAccelerationYo, dt);

      DoubleYoVariable timeYo = new DoubleYoVariable("time", registry);
      YoFunctionGenerator squareFunction = new YoFunctionGenerator("sineFunction", timeYo, registry);
      squareFunction.setMode(YoFunctionGeneratorMode.SQUARE);
      squareFunction.setFrequency(frequency);
      squareFunction.setAmplitude(amplitude);

      double value = 0.0;

      for (double time = 0.0; time < totalTime; time += dt)
      {
         timeYo.set(time);
         value = squareFunction.getValue();

         raw.set(value);
         rawRate.update(raw.getDoubleValue());
         rawAcceleration.update(rawRate.getDoubleValue());
         processed.update(raw.getDoubleValue());
         if (VISUALIZE)
            scs.tickAndUpdate();

         assertTrue(maxRate >= processed.getSmoothedRate().getDoubleValue());
         assertTrue(maxAcceleration >= processed.getSmoothedAcceleration().getDoubleValue());

         assertTrue(Math.abs(processed.getSmoothedRate().getDoubleValue()) <= maxRate);
         assertTrue(Math.abs(processed.getSmoothedAcceleration().getDoubleValue()) <= maxAcceleration);
      }
      shutdownSCSStuff(scs);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void test_Chirp_Plus_UpdateWithoutNoArguments()
   {
      setupSCSStuff();

      maxRate = 10000.0;
      maxAcceleration = 1000.0;
      DoubleYoVariable maxRateYo = new DoubleYoVariable("max_Rate", registry);
      maxRateYo.set(maxRate, notifyListeners);
      DoubleYoVariable maxAccelerationYo = new DoubleYoVariable("max_Acceleration", registry);
      maxAccelerationYo.set(maxAcceleration, notifyListeners);
      DoubleYoVariable inputVariable = new DoubleYoVariable("inputVariable", registry);
      AccelerationLimitedYoVariable processed = new AccelerationLimitedYoVariable(nameYo, registry, maxRateYo, maxAccelerationYo, inputVariable, dt);

      DoubleYoVariable timeYo = new DoubleYoVariable("time", registry);
      YoFunctionGenerator chirpFunction = new YoFunctionGenerator("sineFunction", timeYo, registry);
      chirpFunction.setMode(YoFunctionGeneratorMode.CHIRP_LINEAR);
      chirpFunction.setAmplitude(amplitude);

      double value = 0.0;

      for (double time = 0.0; time < totalTime; time += dt)
      {
         timeYo.set(time);
         value = chirpFunction.getValue();

         raw.set(value);
         rawRate.update(raw.getDoubleValue());
         rawAcceleration.update(rawRate.getDoubleValue());

         inputVariable.set(raw.getDoubleValue());
         double beforeValue = processed.getDoubleValue();
         processed.update();
         double afterValue = processed.getDoubleValue();

         if (beforeValue - afterValue == 0 && time > 0.0)
            fail();

         if (processed.getSmoothedRate().getDoubleValue() > maxRate)
            fail();

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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetAndGetGainsByPolePlacement()
   {
      setupSCSStuff();
      
      DoubleYoVariable maxRateYo = new DoubleYoVariable("max_Rate", registry);
      maxRateYo.set(maxRate, notifyListeners);
      DoubleYoVariable maxAccelerationYo = new DoubleYoVariable("max_Acceleration", registry);
      maxAccelerationYo.set(maxAcceleration, notifyListeners);
      AccelerationLimitedYoVariable processed = new AccelerationLimitedYoVariable(nameYo, registry, maxRateYo, maxAccelerationYo, dt);
      
      double positionGain = random.nextDouble();
      double velocityGain = random.nextDouble();
      processed.setGainsByPolePlacement(positionGain, velocityGain);
      
      double positionGainResult = positionGain * positionGain;
      double velocityGainResult = 2 * positionGain * velocityGain;
      
      assertEquals(positionGainResult, processed.getPositionGain().getDoubleValue(), EPSILON);
      assertEquals(velocityGainResult, processed.getVelocityGain().getDoubleValue(), EPSILON);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testUpdate()
   {
      setupSCSStuff();
      
      maxRate = 10.0;
      maxAcceleration = 7.0;
      double dT = 1;
      DoubleYoVariable maxRateYo = new DoubleYoVariable("max_Rate", registry);
      maxRateYo.set(maxRate, notifyListeners);
      DoubleYoVariable maxAccelerationYo = new DoubleYoVariable("max_Acceleration", registry);
      maxAccelerationYo.set(maxAcceleration, notifyListeners);
      AccelerationLimitedYoVariable processed = new AccelerationLimitedYoVariable(nameYo, registry, maxRateYo, maxAccelerationYo, dT);
      processed.setMaximumRate(maxRate);
      processed.setMaximumAcceleration(maxAcceleration);
      
      for(double displacement = 1.0; displacement < 10000.0; displacement += 1000.0)
      {
         processed.update(displacement);
         assertTrue(processed.getSmoothedRate().getDoubleValue() <= maxRate);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetAndSetMaximumRateaAndAcceleration()
   {
      setupSCSStuff();
      
      maxRate = 1.0;
      maxAcceleration = 1.0;
      DoubleYoVariable maxRateYo = new DoubleYoVariable("max_Rate", registry);
      maxRateYo.set(maxRate, notifyListeners);
      DoubleYoVariable maxAccelerationYo = new DoubleYoVariable("max_Acceleration", registry);
      maxAccelerationYo.set(maxAcceleration, notifyListeners);
      AccelerationLimitedYoVariable processed = new AccelerationLimitedYoVariable(nameYo, registry, maxRateYo, maxAccelerationYo, dt);
      
      processed.setMaximumRate(maxRate);
      processed.setMaximumAcceleration(maxAcceleration);
      assertEquals(maxRate, processed.getMaximumRate(), EPSILON);
      assertEquals(maxAcceleration, processed.getMaximumAcceleration(), EPSILON);
      
      maxRate = 8.5;
      maxAcceleration = 4.5;
      processed.setMaximumRate(maxRate);
      processed.setMaximumAcceleration(maxAcceleration);
      assertEquals(maxRate, processed.getMaximumRate(), EPSILON);
      assertEquals(maxAcceleration, processed.getMaximumAcceleration(), EPSILON);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout=300000)
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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
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