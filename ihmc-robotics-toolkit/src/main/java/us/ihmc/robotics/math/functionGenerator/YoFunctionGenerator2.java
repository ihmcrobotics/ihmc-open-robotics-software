package us.ihmc.robotics.math.functionGenerator;

import java.util.Arrays;

import org.apache.commons.lang3.mutable.MutableDouble;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class YoFunctionGenerator2
{
   private final YoDouble transitionDuration;
   private final InputFilter offset;
   private final InputFilter amplitude;
   private final InputFilter phase;
   private final InputFilter frequency;
   private final InputFilter chirpLowFrequency;
   private final InputFilter chirpHighFrequency;
   private final InputFilter chirpDuration;
   private final InputFilter[] inputs;

   private final YoDouble angle;

   private final YoDouble value;
   private final YoDouble valueDot;
   private final YoDouble valueDDot;

   private final InputFilter modeTransition;
   private final YoEnum<YoFunctionGeneratorMode> mode;
   private final YoEnum<YoFunctionGeneratorMode> modePrevious;
   private boolean hasModeChanged = false;

   private final SineWaveFunctionGenerator sineFunction = new SineWaveFunctionGenerator();
   private final TriangleWaveFunctionGenerator triangleFunction = new TriangleWaveFunctionGenerator();
   private final SawtoothWaveFunctionGenerator sawtoothFunction = new SawtoothWaveFunctionGenerator();
   private final SquareWaveFunctionGenerator squareFunction = new SquareWaveFunctionGenerator();
   private final WhiteNoiseFunctionGenerator whiteNoiseFunction = new WhiteNoiseFunctionGenerator();
   private final ChirpLinearFunctionGenerator chirpLinearFunction = new ChirpLinearFunctionGenerator();
   private final double dt;

   public YoFunctionGenerator2(String namePrefix, double dt, YoRegistry registry)
   {
      this.dt = dt;

      angle = new YoDouble(namePrefix + "Angle", registry);

      value = new YoDouble(namePrefix + "Value", registry);
      valueDot = new YoDouble(namePrefix + "ValueDot", registry);
      valueDDot = new YoDouble(namePrefix + "ValueDDot", registry);

      transitionDuration = new YoDouble(namePrefix + "TransitionDurationFilt", registry);
      transitionDuration.set(0.5);
      offset = new InputFilter(namePrefix + "Offset", transitionDuration, registry);
      amplitude = new InputFilter(namePrefix + "Amp", transitionDuration, registry);
      phase = new InputFilter(namePrefix + "Phase", transitionDuration, registry);
      frequency = new InputFilter(namePrefix + "Freq", transitionDuration, registry);
      chirpLowFrequency = new InputFilter(namePrefix + "ChirpLowFreq", transitionDuration, registry);
      chirpHighFrequency = new InputFilter(namePrefix + "ChirpHighFreq", transitionDuration, registry);
      chirpDuration = new InputFilter(namePrefix + "ChirpDuration", transitionDuration, registry);
      inputs = new InputFilter[] {offset, amplitude, phase, frequency, chirpLowFrequency, chirpHighFrequency, chirpDuration};

      modeTransition = new InputFilter(namePrefix + "ModeTransition", transitionDuration, registry);
      mode = new YoEnum<>(namePrefix + "Mode", registry, YoFunctionGeneratorMode.class);
      modePrevious = new YoEnum<>(namePrefix + "ModePrevious", registry, YoFunctionGeneratorMode.class);

      mode.set(YoFunctionGeneratorMode.OFF);
      modePrevious.set(YoFunctionGeneratorMode.OFF);
      mode.addListener(v -> hasModeChanged = true);

      for (BaseFunctionGenerator function : Arrays.asList(sineFunction, triangleFunction, sawtoothFunction, squareFunction, whiteNoiseFunction))
      {
         function.setOffset(offset);
         function.setAmplitude(amplitude);
         function.setFrequency(frequency);
         function.setPhase(phase);
      }

      chirpLinearFunction.setOffset(offset);
      chirpLinearFunction.setAmplitude(amplitude);
      chirpLinearFunction.setPhase(phase);
      chirpLinearFunction.setLowFrequency(chirpLowFrequency);
      chirpLinearFunction.setHighFrequency(chirpHighFrequency);
      chirpLinearFunction.setChirpDuration(chirpDuration);

      offset.set(0.0);
      amplitude.set(0.0);
      phase.set(0.0);
      frequency.set(1.0);
      chirpDuration.set(20.0);
      chirpLowFrequency.set(0.0);
      chirpHighFrequency.set(2.0);
   }

   public void reset()
   {
      angle.set(0.0);
   }

   private final MutableDouble valueA = new MutableDouble();
   private final MutableDouble valueDotA = new MutableDouble();
   private final MutableDouble valueDDotA = new MutableDouble();
   private final MutableDouble valueB = new MutableDouble();
   private final MutableDouble valueDotB = new MutableDouble();
   private final MutableDouble valueDDotB = new MutableDouble();

   public void update()
   {
      for (InputFilter input : inputs)
      {
         input.update(dt);
      }

      if (!isModeValid(mode.getValue()))
      {
         mode.set(YoFunctionGeneratorMode.OFF);
         return;
      }

      if (hasModeChanged)
      {
         if (mode.getValue() == modePrevious.getValue())
         {
            hasModeChanged = false;
         }
         else
         {
            modeTransition.inputFiltered.set(0.0);
            modeTransition.set(1.0);
            modeTransition.hasUserInputChanged = true;
            initializeMode(mode.getValue());
            hasModeChanged = false;
         }
      }

      if (modeTransition.isTransitioning())
      { // In transition between 2 modes
         angle.set(compute(modePrevious.getValue(), valueA, valueDotA, valueDDotA));
         compute(mode.getValue(), valueB, valueDotB, valueDDotB);
         value.set(EuclidCoreTools.interpolate(valueA.getValue(), valueB.getValue(), modeTransition.getValue()));
         valueDot.set(EuclidCoreTools.interpolate(valueDotA.getValue(), valueDotB.getValue(), modeTransition.getValue()));
         valueDDot.set(EuclidCoreTools.interpolate(valueDDotA.getValue(), valueDDotB.getValue(), modeTransition.getValue()));
         modeTransition.update(dt);

         if (!modeTransition.isTransitioning())
            modePrevious.set(mode.getValue());
      }
      else
      {
         angle.set(compute(mode.getValue(), valueA, valueDotA, valueDDotA));
         value.set(valueA.doubleValue());
         valueDot.set(valueDotA.doubleValue());
         valueDDot.set(valueDDotA.doubleValue());

         if (mode.getValue() == YoFunctionGeneratorMode.CHIRP_LINEAR)
         {
            frequency.set(chirpLinearFunction.getFrequency());
            frequency.inputFiltered.set(chirpLinearFunction.getFrequency());
         }
      }
   }

   private boolean isModeValid(YoFunctionGeneratorMode mode)
   {
      switch (mode)
      {
         case OFF:
         case DC:
         case WHITE_NOISE:
         case SQUARE:
         case SINE:
         case SAWTOOTH:
         case TRIANGLE:
         case CHIRP_LINEAR:
            return true;
         default:
            return false;
      }
   }

   private void initializeMode(YoFunctionGeneratorMode mode)
   {
      switch (mode)
      {
         case SQUARE:
            squareFunction.resetAngle();
            return;
         case SINE:
            sineFunction.resetAngle();
            return;
         case SAWTOOTH:
            sawtoothFunction.resetAngle();
            return;
         case TRIANGLE:
            triangleFunction.resetAngle();
            return;
         case CHIRP_LINEAR:
            chirpLinearFunction.resetChirp();
            return;
         default:
            return;
      }
   }

   private double compute(YoFunctionGeneratorMode mode, MutableDouble value, MutableDouble valueDot, MutableDouble valueDDot)
   {
      switch (mode)
      {
         case OFF:
            return computeOff(value, valueDot, valueDDot);
         case DC:
            return computeDC(value, valueDot, valueDDot);
         case WHITE_NOISE:
            return computeFunction(whiteNoiseFunction, value, valueDot, valueDDot);
         case SQUARE:
            return computeFunction(squareFunction, value, valueDot, valueDDot);
         case SINE:
            return computeFunction(sineFunction, value, valueDot, valueDDot);
         case SAWTOOTH:
            return computeFunction(sawtoothFunction, value, valueDot, valueDDot);
         case TRIANGLE:
            return computeFunction(triangleFunction, value, valueDot, valueDDot);
         case CHIRP_LINEAR:
            return computeChirpLinear(value, valueDot, valueDDot);
         default:
            return Double.NaN;
      }
   }

   private double computeOff(MutableDouble value, MutableDouble valueDot, MutableDouble valueDDot)
   {
      value.setValue(offset.getValue());
      valueDot.setValue(0.0);
      valueDDot.setValue(0.0);
      return 0.0;
   }

   private double computeDC(MutableDouble value, MutableDouble valueDot, MutableDouble valueDDot)
   {
      value.setValue(offset.getValue() + amplitude.getValue());
      valueDot.setValue(0.0);
      valueDDot.setValue(0.0);
      return 0.0;
   }

   private double computeFunction(BaseFunctionGenerator function, MutableDouble value, MutableDouble valueDot, MutableDouble valueDDot)
   {
      function.integrateAngle(dt);
      value.setValue(function.getValue());
      valueDot.setValue(function.getValueDot());
      valueDDot.setValue(function.getValueDDot());
      return function.getAngle();
   }

   private double computeChirpLinear(MutableDouble value, MutableDouble valueDot, MutableDouble valueDDot)
   {
      chirpLinearFunction.integrateAngle(dt);
      value.setValue(chirpLinearFunction.getValue());
      valueDot.setValue(chirpLinearFunction.getValueDot());
      valueDDot.setValue(chirpLinearFunction.getValueDDot());
      return chirpLinearFunction.getAngle();
   }

   public double getValue()
   {
      return value.getValue();
   }

   public double getValueDot()
   {
      return valueDot.getValue();
   }

   public double getValueDDot()
   {
      return valueDDot.getValue();
   }

   public void setOffset(double offset)
   {
      this.offset.set(offset);
   }

   public void setAmplitude(double amplitude)
   {
      this.amplitude.set(amplitude);
   }

   public void setPhase(double phase)
   {
      this.phase.set(phase);
   }

   public void setFrequency(double frequency)
   {
      this.frequency.set(frequency);
   }

   public void setChirpLowFrequency(double chirpLowFrequency)
   {
      this.chirpLowFrequency.set(chirpLowFrequency);
   }

   public void setChirpHighFrequency(double chirpHighFrequency)
   {
      this.chirpHighFrequency.set(chirpHighFrequency);
   }

   public void setChirpDuration(double chirpDuration)
   {
      this.chirpDuration.set(chirpDuration);
   }

   public void setMode(YoFunctionGeneratorMode mode)
   {
      this.mode.set(mode);
   }

   public double getOffset()
   {
      return offset.getValue();
   }

   public double getAmplitude()
   {
      return amplitude.getValue();
   }

   public double getPhase()
   {
      return phase.getValue();
   }

   public double getFrequency()
   {
      return frequency.getValue();
   }

   public double getChirpLowFrequency()
   {
      return chirpLowFrequency.getValue();
   }

   public double getChirpHighFrequency()
   {
      return chirpHighFrequency.getValue();
   }

   public double getChirpDuration()
   {
      return chirpDuration.getValue();
   }

   public YoDouble getAngle()
   {
      return angle;
   }

   public YoEnum<YoFunctionGeneratorMode> getMode()
   {
      return mode;
   }

   public YoEnum<YoFunctionGeneratorMode> getModePrevious()
   {
      return modePrevious;
   }

   private static class InputFilter implements DoubleProvider
   {
      private final YoDouble input;
      private final YoDouble inputFiltered;
      private final YoDouble filterRamp;
      private final DoubleProvider transitionDuration;
      private boolean hasUserInputChanged = true;

      public InputFilter(String inputName, DoubleProvider transitionDuration, YoRegistry registry)
      {
         this.transitionDuration = transitionDuration;

         input = new YoDouble(inputName, registry);
         input.addListener(v -> hasUserInputChanged = true);
         inputFiltered = new YoDouble(inputName + "Filt", registry);
         filterRamp = new YoDouble(inputName + "FiltRamp", registry);
      }

      public void update(double dt)
      {
         double desiredValue = input.getValue();

         if (hasUserInputChanged)
         {
            hasUserInputChanged = false;
            double currentValue = inputFiltered.getValue();
            filterRamp.set((desiredValue - currentValue) / transitionDuration.getValue());
         }

         if (filterRamp.getValue() != 0.0)
         {
            double nextValue = inputFiltered.getValue() + filterRamp.getValue() * dt;

            if ((desiredValue - nextValue) * filterRamp.getValue() <= 0.0)
            {
               nextValue = desiredValue;
               filterRamp.set(0.0);
            }

            inputFiltered.set(nextValue);
         }
      }

      public boolean isTransitioning()
      {
         return input.getValue() != inputFiltered.getValue();
      }

      public void set(double value)
      {
         input.set(value);
      }

      @Override
      public double getValue()
      {
         return inputFiltered.getValue();
      }
   }
}
