package us.ihmc.robotics.math.functionGenerator;

import static us.ihmc.robotics.math.functionGenerator.BaseFunctionGenerator.zeroDoubleProvider;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class ChirpLinearFunctionGenerator
{
   private DoubleProvider lowFrequency = zeroDoubleProvider;
   private DoubleProvider highFrequency = zeroDoubleProvider;
   private DoubleProvider chirpDuration = zeroDoubleProvider;
   private final DoubleProvider frequencyProvider;

   private final TriangleWaveFunctionGenerator frequencyFunction = new TriangleWaveFunctionGenerator();
   private BaseFunctionGenerator baseFunction = new SineWaveFunctionGenerator();

   public ChirpLinearFunctionGenerator()
   {
      frequencyFunction.setOffset(() -> 0.5);
      frequencyFunction.setAmplitude(() -> 0.5);
      frequencyFunction.setFrequency(() -> 0.5 / chirpDuration.getValue());
      frequencyProvider = () -> EuclidCoreTools.interpolate(lowFrequency.getValue(), highFrequency.getValue(), frequencyFunction.getValue());
      setBaseFunction(new SineWaveFunctionGenerator());
   }

   public void setBaseFunction(BaseFunctionGenerator baseFunction)
   {
      this.baseFunction = baseFunction;
   }

   public DoubleProvider getFrequencyProvider()
   {
      return frequencyProvider;
   }

   private void markDirty()
   {
      frequencyFunction.markDirty();
      baseFunction.markDirty();
   }

   public void resetChirp()
   {
      frequencyFunction.resetAngle();
      baseFunction.setFrequency(frequencyProvider);
      baseFunction.resetAngle();
   }

   public void integrateAngle(double dt)
   {
      frequencyFunction.integrateAngle(dt);
      baseFunction.setFrequency(frequencyProvider);
      baseFunction.integrateAngle(dt);
   }

   public void setAngle(double angle)
   {
      baseFunction.setAngle(angle);
   }

   public void setOffset(double offset)
   {
      setOffset(() -> offset);
   }

   public void setOffset(DoubleProvider offset)
   {
      baseFunction.setOffset(offset);
   }

   public void setAmplitude(double amplitude)
   {
      setAmplitude(() -> amplitude);
   }

   public void setAmplitude(DoubleProvider amplitude)
   {
      baseFunction.setAmplitude(amplitude);
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
      baseFunction.setPhase(phase);
   }

   public double getOffset()
   {
      return baseFunction.getOffset();
   }

   public double getAmplitude()
   {
      return baseFunction.getAmplitude();
   }

   public double getFrequency()
   {
      return baseFunction.getFrequency();
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
      return baseFunction.getPhase();
   }

   public double getAngle()
   {
      return baseFunction.getAngle();
   }

   public double getAngleDot()
   {
      return baseFunction.getAngleDot();
   }

   public double getValue()
   {
      return baseFunction.getValue();
   }

   public double getValueDot()
   {
      return baseFunction.getValueDot();
   }

   public double getValueDDot()
   {
      return baseFunction.getValueDDot();
   }
}
