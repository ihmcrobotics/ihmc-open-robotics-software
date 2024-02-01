package us.ihmc.robotics.math.functionGenerator;

import java.util.Arrays;

import org.apache.commons.lang3.mutable.MutableDouble;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.yoVariables.exceptions.IllegalOperationException;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class YoFunctionGeneratorNew
{
   public enum ChirpBaseFunctionMode
   {
      SINE(new SineWaveFunctionGenerator()),
      SQUARE(new SquareWaveFunctionGenerator()),
      SAWTOOTH(new SawtoothWaveFunctionGenerator()),
      TRIANGLE(new TriangleWaveFunctionGenerator());

      private final BaseFunctionGenerator function;

      private ChirpBaseFunctionMode(BaseFunctionGenerator function)
      {
         this.function = function;
      }
   };

   /** Duration used to smooth changes in values of the inputs or to smooth switch between modes. */
   private final YoDouble transitionDuration;
   /** Signals will be centered around the offset. */
   private final InputFilter offset;
   /** Defines signals amplitude such as the resulting value oscillate in [-amplitude; +amplitude]. */
   private final InputFilter amplitude;
   /** Phase shift in radians of the signal. */
   private final InputFilter phase;
   /** Frequency of the signal. */
   private final InputFilter frequency;
   private final YoDouble chirpCurrentFrequency;
   /** Only for chirp signals, defines the lowest frequency spanned by the chirp. */
   private final InputFilter chirpLowFrequency;
   /** Only for chirp signals, defines the highest frequency spanned by the chirp. */
   private final InputFilter chirpHighFrequency;
   /**
    * Only for chirp signals, defines the duration to span from low to high frequency and vice-versa.
    */
   private final InputFilter chirpDuration;
   /** Time before this function generator will automatically turn off. */
   private final YoDouble resetTime;

   private final InputFilter[] inputs;

   private final YoDouble angle;

   private final YoDouble value;
   private final YoDouble valueDot;
   private final YoDouble valueDDot;

   private final InputFilter modeTransition;
   private final YoEnum<YoFunctionGeneratorMode> mode;
   private final YoEnum<YoFunctionGeneratorMode> modePrevious;
   private final YoEnum<ChirpBaseFunctionMode> chirpBaseFunctionMode;
   private boolean hasModeChanged = false;

   private final SineWaveFunctionGenerator sineFunction = new SineWaveFunctionGenerator();
   private final TriangleWaveFunctionGenerator triangleFunction = new TriangleWaveFunctionGenerator();
   private final SawtoothWaveFunctionGenerator sawtoothFunction = new SawtoothWaveFunctionGenerator();
   private final SquareWaveFunctionGenerator squareFunction = new SquareWaveFunctionGenerator();
   private final WhiteNoiseFunctionGenerator whiteNoiseFunction = new WhiteNoiseFunctionGenerator();
   private final ChirpLinearFunctionGenerator chirpLinearFunction = new ChirpLinearFunctionGenerator();
   private final TimeToDTConverter timeToDTConverter;
   private final DoubleProvider dt;

   public YoFunctionGeneratorNew(String namePrefix, YoRegistry registry)
   {
      this(namePrefix, null, new TimeToDTConverter(), registry);
   }

   public YoFunctionGeneratorNew(String namePrefix, DoubleProvider time, YoRegistry registry)
   {
      this(namePrefix, null, time, registry);
   }

   public YoFunctionGeneratorNew(String namePrefix, double dt, YoRegistry registry)
   {
      this(namePrefix, () -> dt, null, registry);
   }

   private YoFunctionGeneratorNew(String namePrefix, DoubleProvider dt, DoubleProvider time, YoRegistry registry)
   {
      this.dt = dt;
      timeToDTConverter = new TimeToDTConverter(time);

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
      chirpCurrentFrequency = new YoDouble(namePrefix + "ChirpCurrentFreq", registry);
      chirpLowFrequency = new InputFilter(namePrefix + "ChirpLowFreq", transitionDuration, registry);
      chirpHighFrequency = new InputFilter(namePrefix + "ChirpHighFreq", transitionDuration, registry);
      chirpDuration = new InputFilter(namePrefix + "ChirpDuration", transitionDuration, registry);
      inputs = new InputFilter[] {offset, amplitude, phase, frequency, chirpLowFrequency, chirpHighFrequency, chirpDuration};

      resetTime = new YoDouble(namePrefix + "ResetTime", registry);
      resetTime.set(Double.POSITIVE_INFINITY);

      modeTransition = new InputFilter(namePrefix + "ModeTransition", transitionDuration, registry);
      mode = new YoEnum<>(namePrefix + "Mode", registry, YoFunctionGeneratorMode.class);
      modePrevious = new YoEnum<>(namePrefix + "ModePrevious", registry, YoFunctionGeneratorMode.class);
      chirpBaseFunctionMode = new YoEnum<>(namePrefix + "ChirpBaseFunctionMode", registry, ChirpBaseFunctionMode.class);
      chirpBaseFunctionMode.set(ChirpBaseFunctionMode.SINE);

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

   /**
    * Updates the internal signal, should be called once per control tick.
    * 
    * @throws IllegalStateException if this was not constructed with either a time provider or a dt.
    */
   public void update()
   {
      if (dt != null)
      {
         updateWithDT(dt.getValue());
      }
      else if (timeToDTConverter != null)
      {
         timeToDTConverter.update();
         updateWithDT(timeToDTConverter.getValue());
      }
      else
      {
         throw new IllegalStateException("This function generator was not properly configured with time info.");
      }
   }

   /**
    * Updates the internal signal, should be called once per control tick.
    * 
    * @param time absolute time value.
    * @throws IllegalOperationException if this was setup with a dt constant.
    */
   public void updateWithTime(double time)
   {
      if (dt != null)
         throw new IllegalOperationException("This function generator was configured with DT");
      timeToDTConverter.update(time);
      updateWithDT(timeToDTConverter.getValue());
   }

   /**
    * Updates the internal signal, should be called once per control tick.
    * 
    * @param dt value of a time increment.
    */
   public void updateWithDT(double dt)
   {
      for (InputFilter input : inputs)
      {
         input.update(dt);
      }

      if (mode.getValue() != YoFunctionGeneratorMode.OFF && Double.isFinite(resetTime.getValue()))
      {
         resetTime.sub(dt);
         if (resetTime.getValue() <= 0.0)
         {
            resetTime.set(0.0);
            mode.set(YoFunctionGeneratorMode.OFF);
         }
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
         angle.set(compute(modePrevious.getValue(), dt, valueA, valueDotA, valueDDotA));
         compute(mode.getValue(), dt, valueB, valueDotB, valueDDotB);
         value.set(EuclidCoreTools.interpolate(valueA.getValue(), valueB.getValue(), modeTransition.getValue()));
         valueDot.set(EuclidCoreTools.interpolate(valueDotA.getValue(), valueDotB.getValue(), modeTransition.getValue()));
         valueDDot.set(EuclidCoreTools.interpolate(valueDDotA.getValue(), valueDDotB.getValue(), modeTransition.getValue()));
         modeTransition.update(dt);

         if (!modeTransition.isTransitioning())
            modePrevious.set(mode.getValue());
      }
      else
      {
         angle.set(compute(mode.getValue(), dt, valueA, valueDotA, valueDDotA));
         value.set(valueA.doubleValue());
         valueDot.set(valueDotA.doubleValue());
         valueDDot.set(valueDDotA.doubleValue());

         if (mode.getValue() == YoFunctionGeneratorMode.CHIRP_LINEAR)
         {
            // For viz purposes
            chirpCurrentFrequency.set(chirpLinearFunction.getFrequency());
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
      BaseFunctionGenerator function = switch (mode)
      {
         case SQUARE -> squareFunction;
         case SINE -> sineFunction;
         case SAWTOOTH -> sawtoothFunction;
         case TRIANGLE -> triangleFunction;
         default -> null;
      };

      if (function != null)
      { // Reset all the providers, the chirp changes the frequency.
         function.setOffset(offset);
         function.setAmplitude(amplitude);
         function.setFrequency(frequency);
         function.setPhase(phase);
         function.resetAngle();
      }

      if (mode == YoFunctionGeneratorMode.CHIRP_LINEAR)
      {
         BaseFunctionGenerator baseFunction = chirpBaseFunctionMode.getValue().function;
         baseFunction.setOffset(offset);
         baseFunction.setAmplitude(amplitude);
         baseFunction.setFrequency(frequency);
         baseFunction.setPhase(phase);
         baseFunction.resetAngle();
         chirpLinearFunction.setBaseFunction(baseFunction);
         chirpLinearFunction.resetChirp();
      }
   }

   private double compute(YoFunctionGeneratorMode mode, double dt, MutableDouble value, MutableDouble valueDot, MutableDouble valueDDot)
   {
      switch (mode)
      {
         case OFF:
            return computeOff(value, valueDot, valueDDot);
         case DC:
            return computeDC(value, valueDot, valueDDot);
         case WHITE_NOISE:
            return computeFunction(whiteNoiseFunction, dt, value, valueDot, valueDDot);
         case SQUARE:
            return computeFunction(squareFunction, dt, value, valueDot, valueDDot);
         case SINE:
            return computeFunction(sineFunction, dt, value, valueDot, valueDDot);
         case SAWTOOTH:
            return computeFunction(sawtoothFunction, dt, value, valueDot, valueDDot);
         case TRIANGLE:
            return computeFunction(triangleFunction, dt, value, valueDot, valueDDot);
         case CHIRP_LINEAR:
            return computeChirpLinear(dt, value, valueDot, valueDDot);
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

   private double computeFunction(BaseFunctionGenerator function, double dt, MutableDouble value, MutableDouble valueDot, MutableDouble valueDDot)
   {
      function.integrateAngle(dt);
      value.setValue(function.getValue());
      valueDot.setValue(function.getValueDot());
      valueDDot.setValue(function.getValueDDot());
      return function.getAngle();
   }

   private double computeChirpLinear(double dt, MutableDouble value, MutableDouble valueDot, MutableDouble valueDDot)
   {
      chirpLinearFunction.integrateAngle(dt);
      value.setValue(chirpLinearFunction.getValue());
      valueDot.setValue(chirpLinearFunction.getValueDot());
      valueDDot.setValue(chirpLinearFunction.getValueDDot());
      return chirpLinearFunction.getAngle();
   }

   /**
    * Gets the current value of the signal.
    * <p>
    * IMPORTANT: call one of the update method to update the internal function over time.
    * </p>
    * 
    * @return the current value of the signal.
    * @see #update()
    * @see #updateWithDT(double)
    * @see #updateWithTime(double)
    */
   public double getValue()
   {
      return value.getValue();
   }

   /**
    * Gets the current value of the first derivative of the signal if available, returns 0 otherwise.
    * <p>
    * IMPORTANT: call one of the update method to update the internal function over time.
    * </p>
    * 
    * @return the current value of the first derivative of the signal if available, returns 0
    *         otherwise.
    * @see #update()
    * @see #updateWithDT(double)
    * @see #updateWithTime(double)
    */
   public double getValueDot()
   {
      return valueDot.getValue();
   }

   /**
    * Gets the current value of the second derivative of the signal if available, returns 0 otherwise.
    * <p>
    * IMPORTANT: call one of the update method to update the internal function over time.
    * </p>
    * 
    * @return the current value of the second derivative of the signal if available, returns 0
    *         otherwise.
    * @see #update()
    * @see #updateWithDT(double)
    * @see #updateWithTime(double)
    */
   public double getValueDDot()
   {
      return valueDDot.getValue();
   }

   /** Duration used to smooth changes in values of the inputs or to smooth switch between modes. */
   public void setTransitionDuration(double transitionDuration)
   {
      this.transitionDuration.set(transitionDuration);
   }

   /** Signals will be centered around the offset. */
   public void setOffset(double offset)
   {
      this.offset.set(offset);
   }

   /** Defines signals amplitude such as the resulting value oscillate in [-amplitude; +amplitude]. */
   public void setAmplitude(double amplitude)
   {
      this.amplitude.set(amplitude);
   }

   /** Phase shift in radians of the signal. */
   public void setPhase(double phase)
   {
      this.phase.set(phase);
   }

   /** Frequency of the signal. */
   public void setFrequency(double frequency)
   {
      this.frequency.set(frequency);
   }

   /** Only for chirp signals, defines the lowest frequency spanned by the chirp. */
   public void setChirpLowFrequency(double chirpLowFrequency)
   {
      this.chirpLowFrequency.set(chirpLowFrequency);
   }

   /** Only for chirp signals, defines the highest frequency spanned by the chirp. */
   public void setChirpHighFrequency(double chirpHighFrequency)
   {
      this.chirpHighFrequency.set(chirpHighFrequency);
   }

   /**
    * Only for chirp signals, defines the duration to span from low to high frequency and vice-versa.
    */
   public void setChirpDuration(double chirpDuration)
   {
      this.chirpDuration.set(chirpDuration);
   }

   /** Time before this function generator will automatically turn off. */
   public void setResetTime(double resetTime)
   {
      this.resetTime.set(resetTime);
   }

   /**
    * Sets the type of signal to use.
    */
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

   public double getResetTime()
   {
      return resetTime.getValue();
   }

   public YoDouble getAngle()
   {
      return angle;
   }

   public YoFunctionGeneratorMode getMode()
   {
      return mode.getValue();
   }

   public YoFunctionGeneratorMode getModePrevious()
   {
      return modePrevious.getValue();
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

            if (transitionDuration.getValue() < dt)
            {
               inputFiltered.set(desiredValue);
               filterRamp.set(0.0);
            }
            else
            {
               filterRamp.set((desiredValue - currentValue) / transitionDuration.getValue());
            }
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

   private static class TimeToDTConverter implements DoubleProvider
   {
      private double dt = 0.0;
      private double timePrevious = Double.NaN;

      private final DoubleProvider time;

      public TimeToDTConverter()
      {
         this(null);
      }

      public TimeToDTConverter(DoubleProvider time)
      {
         this.time = time;
      }

      public void update()
      {
         update(time.getValue());
      }

      public void update(double time)
      {
         if (Double.isNaN(timePrevious))
         {
            timePrevious = time;
         }
         else
         {
            dt = time - timePrevious;
            timePrevious = time;
         }
      }

      @Override
      public double getValue()
      {
         return dt;
      }
   }
}
