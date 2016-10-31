package us.ihmc.robotics.math.functionGenerator;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.math.YoSignalDerivative;
import us.ihmc.robotics.math.YoSignalDerivative.DifferentiationMode;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.FilteredVelocityYoVariable;


/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class YoFunctionGenerator
{
   private static final double TWO_PI = 2.0 * Math.PI;
   private final double MIN_EXPONENTIAL_SWEEP_TIME = 1.0;
   private final double MIN_LINEAR_SWEEP_TIME = 0.001;

   private YoVariableRegistry registry;

   private final Random random = new Random(1776L);

   private final DoubleYoVariable value, offset, amplitude, frequency, phase, resetTime, pauseTime, chirpRate, chirpFrequency, chirpFrequencyMax;
   private final DoubleYoVariable valueDot;
   private final FilteredVelocityYoVariable valueDotFromFilter;

   private final DoubleYoVariable timeModeChanged;
   private final DoubleYoVariable timeInCurrentMode;
   private final DoubleYoVariable kRateForExponentialChirp;
   private final BooleanYoVariable chirpUpAndDown;
   private final BooleanYoVariable stopAfterResetTime;

   private final EnumYoVariable<YoFunctionGeneratorMode> mode;
   private final EnumYoVariable<YoFunctionGeneratorMode> modePrevious;
   private final YoVariable[] createdVariables;
   private final YoSignalDerivative signalDerivative;

   private final DoubleYoVariable time;
   private double sweepFrequencyLowHz = 0.001;
   private double previousResetTime, previousChirpFrequency;
   private boolean previousChirpUpAndDown;

   private boolean smoothParameters = false;

   private boolean modeChanged;
   private final AlphaFilteredYoVariable offsetFiltered, amplitudeFiltered;
   private final DoubleYoVariable frequencyFiltered, phaseFiltered;
   private final DoubleYoVariable alphaFilter;

   public YoFunctionGenerator(String name, YoVariableRegistry registry)
   {
      this(name, null, registry, false, -1.0);
   }

   public YoFunctionGenerator(String name, YoVariableRegistry registry, boolean smoothParameters)
   {
      this(name, null, registry);
   }

   public YoFunctionGenerator(String name, DoubleYoVariable time, YoVariableRegistry parentRegistry)
   {
      this(name, time, parentRegistry, false, -1.0);
   }

   public YoFunctionGenerator(String name, DoubleYoVariable time, YoVariableRegistry parentRegistry, boolean smoothParameters, double dT)
   {
      this.smoothParameters = smoothParameters;

      registry = new YoVariableRegistry(name + "YoFunGen");

      this.time = time;

      value = new DoubleYoVariable(name + "Value", registry);

      valueDot = new DoubleYoVariable(name + "ValueDot", registry);
      if (dT != -1.0)
         valueDotFromFilter = new FilteredVelocityYoVariable(name + "ValueDotFromFilter", "", 0.0, value, dT, registry);
      else
         valueDotFromFilter = null;

      offset = new DoubleYoVariable(name + "Offset", registry);
      amplitude = new DoubleYoVariable(name + "Amp", registry);
      frequency = new DoubleYoVariable(name + "Freq", registry);
      phase = new DoubleYoVariable(name + "Phase", registry);

      alphaFilter = new DoubleYoVariable("alphaFilter", registry);
      if (!smoothParameters)
         alphaFilter.set(0.0);
      else
         alphaFilter.set(0.999);

      offsetFiltered = new AlphaFilteredYoVariable("offsetFiltered", registry, alphaFilter, offset);
      amplitudeFiltered = new AlphaFilteredYoVariable("amplitudeFiltered", registry, alphaFilter, amplitude);

      frequencyFiltered = new DoubleYoVariable("frequencyFiltered", registry);
      phaseFiltered = new DoubleYoVariable("phaseFiltered", registry);

      pauseTime = new DoubleYoVariable(name + "PauseTime", registry);
      resetTime = new DoubleYoVariable(name + "ResetTime", registry);
      chirpRate = new DoubleYoVariable(name + "ChirpRate", registry);
      chirpUpAndDown = new BooleanYoVariable(name + "ChirpUpAndDown", registry);
      stopAfterResetTime = new BooleanYoVariable(name + "StopAfterResetTime", registry);
      stopAfterResetTime.set(false);

      chirpFrequency = new DoubleYoVariable(name + "ChirpFrequency", registry);
      chirpFrequencyMax = new DoubleYoVariable(name + "ChirpFrequencyMax", registry);

      timeModeChanged = new DoubleYoVariable(name + "TimeModeChanged", registry);
      timeInCurrentMode = new DoubleYoVariable(name + "TimeInCurrentMode", registry);
      kRateForExponentialChirp = new DoubleYoVariable(name + "KRateForExponentialChirp", registry);

      mode = EnumYoVariable.create(name + "Mode", YoFunctionGeneratorMode.class, registry);
      modePrevious = EnumYoVariable.create(name + "ModePrevious", YoFunctionGeneratorMode.class, registry);

      // There is used to be no initialization. This makes it explicit
      mode.set(YoFunctionGeneratorMode.OFF);
      modePrevious.set(mode.getEnumValue());

      resetTime.set(10.0);
      frequency.set(1.0);
      chirpRate.set(1.0);
      chirpFrequencyMax.set(1.0);

      createdVariables = new YoVariable[] { value, offset, amplitude, frequency, phase, resetTime, chirpRate, mode };

      signalDerivative = new YoSignalDerivative(name, registry);
      signalDerivative.setDifferentiationMode(DifferentiationMode.USING_DT);
      signalDerivative.resetToZero();

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   public String[] getCreatedVariableNames()
   {
      String[] ret = new String[createdVariables.length];

      for (int i = 0; i < createdVariables.length; i++)
      {
         ret[i] = createdVariables[i].getName();
      }

      return ret;
   }

   public void setOffset(double offset)
   {
      this.offset.set(offset);
   }

   public void setOffsetFiltered(double offset)
   {
      this.offsetFiltered.reset();
      this.offsetFiltered.set(offset);
      this.offset.set(offset);
   }

   public double getOffset()
   {
      return offset.getDoubleValue();
   }

   public void setAmplitude(double amplitude)
   {
      this.amplitude.set(amplitude);
   }

   public double getAmplitude()
   {
      return amplitude.getDoubleValue();
   }

   public void setFrequencyWithContinuousOutput(double frequency)
   {
      setPhase(getPhase() + TWO_PI * (getFrequency() - frequency) * timeInCurrentMode.getDoubleValue());
      setFrequency(frequency);
   }

   public void setFrequency(double frequency)
   {
      this.frequency.set(frequency);
   }

   public double getFrequency()
   {
      return frequency.getDoubleValue();
   }

   public void setChirpFrequencyMaxHz(double frequencyHz)
   {
      this.chirpFrequencyMax.set(frequencyHz);
   }

   public double getChirpFrequencyMax()
   {
      return chirpFrequencyMax.getDoubleValue();
   }

   public void setPhase(double phase)
   {
      this.phase.set(phase);
   }

   public double getPhase()
   {
      return phase.getDoubleValue();
   }

   public void setMode(YoFunctionGeneratorMode mode)
   {
      this.mode.set(mode);
   }

   public YoFunctionGeneratorMode getMode()
   {
      return mode.getEnumValue();
   }

   public void setResetTime(double resetTime)
   {
      this.resetTime.set(resetTime);
   }

   public double getResetTime()
   {
      return resetTime.getDoubleValue();
   }

   public void setPauseTime(double pauseTime)
   {
      this.pauseTime.set(pauseTime);
   }

   public double getPauseTime()
   {
      return pauseTime.getDoubleValue();
   }

   public void setChirpRate(double frequencyRate)
   {
      this.chirpRate.set(frequencyRate);
   }

   public double getChirpRate()
   {
      return chirpRate.getDoubleValue();
   }

   public double getKRateForExponentialChirp()
   {
      return kRateForExponentialChirp.getDoubleValue();
   }

   public void setChirpUpAndDown(boolean value)
   {
      chirpUpAndDown.set(value);
   }

   public double getValue()
   {
      if (time == null)
      {
         throw new RuntimeException(
               "Function Generator wasn't created with a time YoVariable. Need to create with a time variable or call getValue(double time) instead");
      }

      return getValue(time.getDoubleValue());
   }

   public double getValueDot()
   {
      return valueDot.getDoubleValue();
   }

   public boolean getStopAfterResetTime()
   {
      return stopAfterResetTime.getBooleanValue();
   }

   public void setStopAfterResetTime(boolean stopAfterResetTime)
   {
      this.stopAfterResetTime.set(stopAfterResetTime);
   }

   public void resetTimeModeChanged()
   {
      timeModeChanged.set(0.0);
   }

   public double getValue(double time)
   {
      updateFilteredValues();

      if (mode.getEnumValue().equals(modePrevious.getEnumValue()))
         modeChanged = false;
      else
         performModeChangeAction(time);

      timeInCurrentMode.set(time - timeModeChanged.getDoubleValue());

      double effectiveResetTime = resetTime.getDoubleValue();
      if (chirpUpAndDown.getBooleanValue()
            && (mode.getEnumValue().equals(YoFunctionGeneratorMode.CHIRP_EXPONENTIAL) || mode.getEnumValue().equals(YoFunctionGeneratorMode.CHIRP_LINEAR)))
      {
         effectiveResetTime = effectiveResetTime * 2.0;
      }

      effectiveResetTime = effectiveResetTime + pauseTime.getDoubleValue();

      if (timeInCurrentMode.getDoubleValue() > effectiveResetTime && stopAfterResetTime.getBooleanValue())
      {
         mode.set(YoFunctionGeneratorMode.OFF);
      }

      switch (mode.getEnumValue())
      {
      case OFF:
      {
         // In OFF state, it will send out the offset.
         value.set(offsetFiltered.getDoubleValue());

         break;
      }

      case SINE:
      {
         value.set(offsetFiltered.getDoubleValue() + amplitudeFiltered.getDoubleValue()
               * Math.sin(2.0 * Math.PI * frequencyFiltered.getDoubleValue() * timeInCurrentMode.getDoubleValue() + phaseFiltered.getDoubleValue()));

         valueDot.set(amplitudeFiltered.getDoubleValue() * Math.PI * 2.0 * frequencyFiltered.getDoubleValue()
               * Math.cos(2.0 * Math.PI * frequencyFiltered.getDoubleValue() * timeInCurrentMode.getDoubleValue()));
         if (valueDotFromFilter != null)
            valueDotFromFilter.update();

         break;
      }

      case CHIRP_LINEAR:
      {
         verifyChirpParametersAreInRange();

         // The linear chirp will be governed by two input: reset time, peak freq
         if (previousResetTime != resetTime.getDoubleValue())
            performModeChangeAction(time);

         if (previousChirpFrequency != chirpFrequencyMax.getDoubleValue())
            performModeChangeAction(time);

         if (previousChirpUpAndDown != chirpUpAndDown.getBooleanValue())
            performModeChangeAction(time);

         if (modeChanged)
         {
            previousResetTime = resetTime.getDoubleValue();
            previousChirpFrequency = chirpFrequencyMax.getDoubleValue();
            previousChirpUpAndDown = chirpUpAndDown.getBooleanValue();
            chirpRate.set(chirpFrequencyMax.getDoubleValue() / resetTime.getDoubleValue());
            
         }

         if (resetTime.getDoubleValue() < MIN_LINEAR_SWEEP_TIME)
            resetTime.set(MIN_LINEAR_SWEEP_TIME);

         double chirpTime;
         if (chirpUpAndDown.getBooleanValue())
            chirpTime = timeInCurrentMode.getDoubleValue() % (pauseTime.getDoubleValue() + 2.0 * resetTime.getDoubleValue());
         else
            chirpTime = timeInCurrentMode.getDoubleValue() % (pauseTime.getDoubleValue() + resetTime.getDoubleValue());

         double phaseForWayDown = 0.0;
         if (chirpTime < pauseTime.getDoubleValue())
         {
            chirpFrequency.set(0.0);
            chirpTime = 0.0;
         }
         else
         {
            chirpTime = chirpTime - pauseTime.getDoubleValue();

            if (chirpUpAndDown.getBooleanValue())
            {
               // figure out if on way up or down
               if (chirpTime > resetTime.getDoubleValue())
               {
                  // on way down
                  chirpTime = 2.0 * resetTime.getDoubleValue() - chirpTime;
                  phaseForWayDown = Math.PI;
               }
            }

            chirpFrequency.set(chirpRate.getDoubleValue() * chirpTime);
         }

         value.set(offsetFiltered.getDoubleValue() + amplitudeFiltered.getDoubleValue()
               * Math.sin(Math.PI * chirpFrequency.getDoubleValue() * chirpTime + phaseForWayDown));
         valueDot.set(amplitudeFiltered.getDoubleValue() * Math.PI * 2.0 * chirpTime * chirpRate.getDoubleValue()
               * Math.cos(Math.PI * chirpFrequency.getDoubleValue() * chirpTime));

         if (valueDotFromFilter != null)
            valueDotFromFilter.update();

         break;
      }

      case CHIRP_EXPONENTIAL:
      {
         verifyChirpParametersAreInRange();

         double effectiveFrequency;

         if (previousResetTime != resetTime.getDoubleValue())
            performModeChangeAction(time);

         if (previousChirpFrequency != chirpFrequencyMax.getDoubleValue())
            performModeChangeAction(time);

         if (previousChirpUpAndDown != chirpUpAndDown.getBooleanValue())
            performModeChangeAction(time);

         if (modeChanged)
         {
            kRateForExponentialChirp.set(YoFunctionGenerator.getkRate(sweepFrequencyLowHz, chirpFrequencyMax.getDoubleValue(), resetTime.getDoubleValue()));

            previousResetTime = resetTime.getDoubleValue();
            previousChirpFrequency = chirpFrequencyMax.getDoubleValue();
            previousChirpUpAndDown = chirpUpAndDown.getBooleanValue();
         }

         if (resetTime.getDoubleValue() < MIN_EXPONENTIAL_SWEEP_TIME)
            resetTime.set(MIN_EXPONENTIAL_SWEEP_TIME);

         double chirpTime;
         if (chirpUpAndDown.getBooleanValue())
            chirpTime = timeInCurrentMode.getDoubleValue() % (pauseTime.getDoubleValue() + 2.0 * resetTime.getDoubleValue());
         else
            chirpTime = timeInCurrentMode.getDoubleValue() % (pauseTime.getDoubleValue() + resetTime.getDoubleValue());

         if (chirpTime < pauseTime.getDoubleValue())
         {
            effectiveFrequency = 0.0;
         }
         else
         {
            chirpTime = chirpTime - pauseTime.getDoubleValue();

            if (chirpUpAndDown.getBooleanValue())
            {
               // figure out if on way up or down
               if (chirpTime > resetTime.getDoubleValue())
               {
                  // on way down
                  chirpTime = 2.0 * resetTime.getDoubleValue() - chirpTime;
               }
            }

            if (chirpTime == 0.0)
               effectiveFrequency = 0.0;
            else
            {
               effectiveFrequency = sweepFrequencyLowHz * (Math.pow(kRateForExponentialChirp.getDoubleValue(), chirpTime) - 1.0)
                     / (Math.log(kRateForExponentialChirp.getDoubleValue()) * chirpTime);
            }

            if (effectiveFrequency > chirpFrequencyMax.getDoubleValue())
            {
               // In theory, this should not happen, or only for a few mS, because we are properly calculating kRateForExponentialChirp
               effectiveFrequency = 0.0;
            }

         }

         value.set(offsetFiltered.getDoubleValue() + amplitudeFiltered.getDoubleValue() * Math.sin(2.0 * Math.PI * effectiveFrequency * chirpTime));
         double velocity = signalDerivative.getDerivative(value.getDoubleValue(), timeInCurrentMode.getDoubleValue());
         if(velocity > Double.MAX_VALUE || velocity < -Double.MAX_VALUE)
            velocity = 0.0;
         valueDot.set(velocity);

         break;
      }

      case SQUARE:
      {
         double square;
         if (timeInCurrentMode.getDoubleValue() < pauseTime.getDoubleValue())
            square = 0.0;
         else
         {
            double timeAfterPause = timeInCurrentMode.getDoubleValue() - pauseTime.getDoubleValue();
            square = Math.sin(TWO_PI * frequencyFiltered.getDoubleValue() * timeAfterPause + phaseFiltered.getDoubleValue());
            if (square > 0.0)
               square = 1.0;
            else
               square = -1.0;
         }

         value.set(offsetFiltered.getDoubleValue() + amplitudeFiltered.getDoubleValue() * square);

         break;
      }

      case SAWTOOTH:
      {
         double angle = (TWO_PI * frequencyFiltered.getDoubleValue() * timeInCurrentMode.getDoubleValue() + phaseFiltered.getDoubleValue()) % TWO_PI;
         if (angle < 0.0)
            angle = angle + TWO_PI;

         if (angle < Math.PI / 2.0)
         {
            double percentUp = angle / (Math.PI / 2.0);
            value.set(offsetFiltered.getDoubleValue() + amplitudeFiltered.getDoubleValue() * percentUp);
         }
         else if (angle < 3.0 * Math.PI / 2.0)
         {
            double percentDown = (angle - Math.PI / 2.0) / (Math.PI / 2.0);
            value.set(offsetFiltered.getDoubleValue() + amplitudeFiltered.getDoubleValue() - amplitudeFiltered.getDoubleValue() * percentDown);
         }
         else if (angle < TWO_PI)
         {
            double percentUp = (angle - 3.0 * Math.PI / 2.0) / (Math.PI / 2.0);
            value.set(offsetFiltered.getDoubleValue() - amplitudeFiltered.getDoubleValue() + amplitudeFiltered.getDoubleValue() * percentUp);
         }
         else
         {
            value.set(offsetFiltered.getDoubleValue());
         }

         break;
      }

      case TRIANGLE:
      {
         double p = 1 / (2 * frequencyFiltered.getDoubleValue());
         value.set((2 * amplitudeFiltered.getDoubleValue() / p) * (p - Math.abs(timeInCurrentMode.getDoubleValue() % (2 * p) - p))
               - amplitudeFiltered.getDoubleValue() + offsetFiltered.getDoubleValue());
         valueDot.set(-1 * Math.signum((timeInCurrentMode.getDoubleValue() % (2 * p) - p))
               * Math.abs(2 * amplitudeFiltered.getDoubleValue() / ((1 / frequencyFiltered.getDoubleValue()) / 2)));
         
         // update velocity also inside the numerical derivative computer for comparison purpose
         signalDerivative.getDerivative(value.getDoubleValue(), timeInCurrentMode.getDoubleValue());

         break;
      }

      case WHITE_NOISE:
      {
         double randomDouble = 2.0 * random.nextDouble() - 1.0;
         value.set(offsetFiltered.getDoubleValue() + amplitudeFiltered.getDoubleValue() * randomDouble);

         break;
      }

      case DC:
      {
         value.set(offsetFiltered.getDoubleValue() + amplitudeFiltered.getDoubleValue());

         break;
      }

      default:
      {
         throw new RuntimeException("Shouldn't get here!");
      }
      }

      modePrevious.set(mode.getEnumValue());

      return value.getDoubleValue();
   }

   private void verifyChirpParametersAreInRange()
   {
      if (chirpFrequencyMax.getDoubleValue() <= sweepFrequencyLowHz)
      {
         chirpFrequencyMax.set(2.0 * sweepFrequencyLowHz + 1.0);
      }

      if (resetTime.getDoubleValue() <= 0.01)
      {
         resetTime.set(0.01);
      }
   }

   private void updateFilteredValues()
   {
      offsetFiltered.update();
      amplitudeFiltered.update();

      if (!smoothParameters)
      {
         frequencyFiltered.set(frequency.getDoubleValue());
         phaseFiltered.set(phase.getDoubleValue());
      }
      else
      {
         //only update when argument to sine function is zero.
         if (Math.abs(Math.sin(TWO_PI * frequencyFiltered.getDoubleValue() * timeInCurrentMode.getDoubleValue() + phaseFiltered.getDoubleValue())) < 0.01)
         {
            frequencyFiltered.set(frequency.getDoubleValue());
            phaseFiltered.set(phase.getDoubleValue());
         }
      }
   }

   private void performModeChangeAction(double time)
   {
      modeChanged = true;
      timeModeChanged.set(time);
      timeInCurrentMode.set(time - timeModeChanged.getDoubleValue());
      signalDerivative.initialize(DifferentiationMode.USING_DT, value.getDoubleValue(), timeInCurrentMode.getDoubleValue(), valueDot.getDoubleValue());
   }

   private static double getkRate(double sweepFreqLow, double sweepFreqHigh, double sweepTime)
   {
      boolean DEBUG = false;

      if (sweepFreqHigh <= sweepFreqLow)
      {
         System.err.println("sweepFreqHigh must be greater than sweepFreqLow. Setting sweepFreqHigh to 2*sweepFreqLow + 1.0");
         sweepFreqHigh = 2.0 * sweepFreqLow + 1.0;
      }

      if (sweepTime <= 0.0)
      {
         System.err.println("sweep time must be greater than 0.0. Using 1.0");
         sweepTime = 1.0;
      }

      double lowerBound = 0.0001;
      double upperBound = 1e12;

      boolean keepTrying = true;
      int numberOfIterations = 0;

      while (keepTrying)
      {
         double midpoint = (upperBound + lowerBound) / 2.0;

         double midPointValue = computeExpressionForChirpRate(sweepFreqLow, sweepFreqHigh, sweepTime, midpoint);

         if (DEBUG)
         {
            double lowPointValue = computeExpressionForChirpRate(sweepFreqLow, sweepFreqHigh, sweepTime, lowerBound);
            double upperPointValue = computeExpressionForChirpRate(sweepFreqLow, sweepFreqHigh, sweepTime, upperBound);

            System.out.println("");
            System.out.println(numberOfIterations + ", " + lowPointValue + ", " + midPointValue + ", " + upperPointValue);
            System.out.println(numberOfIterations + ", " + lowerBound + ", " + midpoint + ", " + upperBound);
         }

         if (midPointValue < 0.0)
            lowerBound = midpoint;
         else if (midPointValue > 0.0)
            upperBound = midpoint;
         else
         {
            keepTrying = false;
            upperBound = midpoint;
         }

         numberOfIterations++;

         if (numberOfIterations > 100)
            keepTrying = false;
      }

      return upperBound;
   }

   private static double computeExpressionForChirpRate(double sweepFreqLow, double sweepFreqHigh, double sweepTime, double kRate)
   {
      return ((sweepFreqLow * (Math.pow(kRate, sweepTime) - 1.0)) / (sweepFreqHigh * Math.log(kRate) * sweepTime) - 1.0);
   }

   public static void generateTestData(YoFunctionGenerator yoFunctionGenerator)
   {
      System.out.println("starting generateTestData()");

      yoFunctionGenerator.setMode(YoFunctionGeneratorMode.CHIRP_EXPONENTIAL);

      double sweepFreqHigh = 50.0;
      double sweepTime = 20.0;

      yoFunctionGenerator.setChirpFrequencyMaxHz(sweepFreqHigh);
      yoFunctionGenerator.setResetTime(sweepTime);
      yoFunctionGenerator.setAmplitude(1.0);

      double deltaTime = 0.01;

      ArrayList<Double> timeArray = new ArrayList<Double>();
      ArrayList<Double> valueArray = new ArrayList<Double>();

      for (double time = 0.0; time < 1.03 * sweepTime; time = time + deltaTime)
      {
         timeArray.add(time);
         valueArray.add(yoFunctionGenerator.getValue(time));
      }

      for (int i = 0; i < timeArray.size(); i++)
      {
         System.out.println(timeArray.get(i) + ", " + valueArray.get(i));
      }

      System.out.println("KRateForExponentialChirp=" + yoFunctionGenerator.getKRateForExponentialChirp());
   }

   public static void main(String[] args)
   {
      YoFunctionGenerator yoFunctionGenerator = new YoFunctionGenerator("test", null);

      YoFunctionGenerator.generateTestData(yoFunctionGenerator);
   }
}