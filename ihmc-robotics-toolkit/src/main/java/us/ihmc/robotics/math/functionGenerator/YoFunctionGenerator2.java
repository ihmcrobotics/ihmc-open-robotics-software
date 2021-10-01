package us.ihmc.robotics.math.functionGenerator;

import java.util.Arrays;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class YoFunctionGenerator2
{
   private final YoDouble offset;
   private final YoDouble amplitude;
   private final YoDouble phase;
   private final YoDouble frequency;
   private final YoDouble chirpLowFrequency;
   private final YoDouble chirpHighFrequency;
   private final YoDouble chirpDuration;

   private final YoDouble angle;

   private final YoDouble value;
   private final YoDouble valueDot;
   private final YoDouble valueDDot;

   private final YoEnum<YoFunctionGeneratorMode> mode;
   private final YoEnum<YoFunctionGeneratorMode> modePrevious;

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

      offset = new YoDouble(namePrefix + "Offset", registry);
      amplitude = new YoDouble(namePrefix + "Amp", registry);
      phase = new YoDouble(namePrefix + "Phase", registry);
      frequency = new YoDouble(namePrefix + "Freq", registry);
      chirpLowFrequency = new YoDouble(namePrefix + "ChirpLowFreq", registry);
      chirpHighFrequency = new YoDouble(namePrefix + "ChirpHighFreq", registry);
      chirpDuration = new YoDouble(namePrefix + "ChirpDuration", registry);

      mode = new YoEnum<>(namePrefix + "Mode", registry, YoFunctionGeneratorMode.class);
      modePrevious = new YoEnum<>(namePrefix + "ModePrevious", registry, YoFunctionGeneratorMode.class);

      mode.set(YoFunctionGeneratorMode.OFF);
      modePrevious.set(YoFunctionGeneratorMode.OFF);

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
      chirpDuration.set(5.0);
      chirpLowFrequency.set(1.0);
      chirpHighFrequency.set(2.0);
   }

   public void reset()
   {
      angle.set(0.0);
   }

   public void update()
   {
      boolean modeChanged = mode.getValue() != modePrevious.getValue();

      switch (mode.getValue())
      {
         case OFF:
         {
            angle.set(0.0);
            value.set(offset.getValue());
            valueDot.set(0.0);
            valueDDot.set(0.0);
            break;
         }
         case DC:
         {
            angle.set(0.0);
            value.set(offset.getValue() + amplitude.getValue());
            valueDot.set(0.0);
            valueDDot.set(0.0);
            break;
         }
         case WHITE_NOISE:
         {
            angle.set(0.0);
            whiteNoiseFunction.integrateAngle(dt);
            value.set(whiteNoiseFunction.getValue());
            valueDot.set(whiteNoiseFunction.getValueDot());
            valueDDot.set(whiteNoiseFunction.getValueDDot());
            break;
         }
         case SQUARE:
         {
            if (modeChanged)
               squareFunction.setAngle(angle.getValue());
            else
               squareFunction.integrateAngle(dt);
            angle.set(squareFunction.getAngle());
            value.set(squareFunction.getValue());
            valueDot.set(squareFunction.getValueDot());
            valueDDot.set(squareFunction.getValueDDot());
            break;
         }
         case SINE:
         {
            if (modeChanged)
               sineFunction.setAngle(angle.getValue());
            else
               sineFunction.integrateAngle(dt);
            angle.set(sineFunction.getAngle());
            value.set(sineFunction.getValue());
            valueDot.set(sineFunction.getValueDot());
            valueDDot.set(sineFunction.getValueDDot());
            break;
         }
         case SAWTOOTH:
         {
            if (modeChanged)
               sawtoothFunction.setAngle(angle.getValue());
            else
               sawtoothFunction.integrateAngle(dt);
            angle.set(sawtoothFunction.getAngle());
            value.set(sawtoothFunction.getValue());
            valueDot.set(sawtoothFunction.getValueDot());
            valueDDot.set(sawtoothFunction.getValueDDot());
            break;
         }
         case TRIANGLE:
         {
            if (modeChanged)
               triangleFunction.setAngle(angle.getValue());
            else
               triangleFunction.integrateAngle(dt);
            angle.set(triangleFunction.getAngle());
            value.set(triangleFunction.getValue());
            valueDot.set(triangleFunction.getValueDot());
            valueDDot.set(triangleFunction.getValueDDot());
            break;
         }
         case CHIRP_LINEAR:
         {
            if (modeChanged)
               chirpLinearFunction.setAngle(angle.getValue());
            else
               chirpLinearFunction.integrateAngle(dt);
            angle.set(chirpLinearFunction.getAngle());
            value.set(chirpLinearFunction.getValue());
            valueDot.set(chirpLinearFunction.getValueDot());
            valueDDot.set(chirpLinearFunction.getValueDDot());
            break;
         }
         default:
            mode.set(YoFunctionGeneratorMode.OFF);
            break;
      }

      modePrevious.set(mode.getValue());
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

   public YoDouble getOffset()
   {
      return offset;
   }

   public YoDouble getAmplitude()
   {
      return amplitude;
   }

   public YoDouble getPhase()
   {
      return phase;
   }

   public YoDouble getFrequency()
   {
      return frequency;
   }

   public YoDouble getChirpLowFrequency()
   {
      return chirpLowFrequency;
   }

   public YoDouble getChirpHighFrequency()
   {
      return chirpHighFrequency;
   }

   public YoDouble getChirpDuration()
   {
      return chirpDuration;
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
}
