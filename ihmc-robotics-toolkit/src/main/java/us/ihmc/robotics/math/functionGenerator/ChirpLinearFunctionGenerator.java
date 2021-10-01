package us.ihmc.robotics.math.functionGenerator;

import static us.ihmc.robotics.math.functionGenerator.BaseFunctionGenerator.zeroDoubleProvider;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class ChirpLinearFunctionGenerator
{
   private DoubleProvider lowFrequency = zeroDoubleProvider;
   private DoubleProvider highFrequency = zeroDoubleProvider;
   private DoubleProvider chirpDuration = zeroDoubleProvider;

   private final TriangleWaveFunctionGenerator frequencyFunction = new TriangleWaveFunctionGenerator();
   private final SineWaveFunctionGenerator sinewave = new SineWaveFunctionGenerator();

   public ChirpLinearFunctionGenerator()
   {
      frequencyFunction.setOffset(() -> 0.5);
      frequencyFunction.setAmplitude(() -> 0.5);
      frequencyFunction.setFrequency(() -> 0.5 / chirpDuration.getValue());
      sinewave.setFrequency(() -> EuclidCoreTools.interpolate(lowFrequency.getValue(), highFrequency.getValue(), frequencyFunction.getValue()));
   }

   private void markDirty()
   {
      frequencyFunction.markDirty();
      sinewave.markDirty();
   }

   public void resetChirp()
   {
      frequencyFunction.resetAngle();
      sinewave.resetAngle();
   }

   public void integrateAngle(double dt)
   {
      frequencyFunction.integrateAngle(dt);
      sinewave.integrateAngle(dt);
   }

   public void setAngle(double angle)
   {
      sinewave.setAngle(angle);
   }

   public void setOffset(double offset)
   {
      setOffset(() -> offset);
   }

   public void setOffset(DoubleProvider offset)
   {
      sinewave.setOffset(offset);
   }

   public void setAmplitude(double amplitude)
   {
      setAmplitude(() -> amplitude);
   }

   public void setAmplitude(DoubleProvider amplitude)
   {
      sinewave.setAmplitude(amplitude);
   }

   public void setLowFrequency(double lowFrequency)
   {
      setLowFrequency(() -> lowFrequency);
   }

   public void setLowFrequency(DoubleProvider lowFrequency)
   {
      this.lowFrequency = lowFrequency;
      markDirty();
   }

   public void setHighFrequency(double highFrequency)
   {
      setHighFrequency(() -> highFrequency);
   }

   public void setHighFrequency(DoubleProvider highFrequency)
   {
      this.highFrequency = highFrequency;
      markDirty();
   }

   public void setChirpDuration(double chirpDuration)
   {
      setChirpDuration(() -> chirpDuration);
   }

   public void setChirpDuration(DoubleProvider chirpDuration)
   {
      this.chirpDuration = chirpDuration;
      markDirty();
   }

   public void setPhase(double phase)
   {
      setPhase(() -> phase);
   }

   public void setPhase(DoubleProvider phase)
   {
      sinewave.setPhase(phase);
   }

   public double getOffset()
   {
      return sinewave.getOffset();
   }

   public double getAmplitude()
   {
      return sinewave.getAmplitude();
   }

   public double getFrequency()
   {
      return sinewave.getFrequency();
   }

   public double getLowFrequency()
   {
      return lowFrequency.getValue();
   }

   public double getHighFrequency()
   {
      return highFrequency.getValue();
   }

   public double getChirpDuration()
   {
      return chirpDuration.getValue();
   }

   public double getPhase()
   {
      return sinewave.getPhase();
   }

   public double getAngle()
   {
      return sinewave.getAngle();
   }

   public double getAngleDot()
   {
      return sinewave.getAngleDot();
   }

   public double getValue()
   {
      return sinewave.getValue();
   }

   public double getValueDot()
   {
      return sinewave.getValueDot();
   }

   public double getValueDDot()
   {
      return sinewave.getValueDDot();
   }
}
