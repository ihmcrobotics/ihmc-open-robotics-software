package us.ihmc.robotics.math.functionGenerator;

import us.ihmc.yoVariables.providers.DoubleProvider;

public class SinewaveFunctionGenerator
{
   public static final DoubleProvider zeroDoubleProvider = () -> 0.0;
   private DoubleProvider offset = zeroDoubleProvider;
   private DoubleProvider amplitude = zeroDoubleProvider;
   private DoubleProvider frequency = zeroDoubleProvider;
   private DoubleProvider phase = zeroDoubleProvider;

   private double angle = 0.0;

   private boolean outputDirty = true;
   private double value = 0.0;
   private double valueDot = 0.0;
   private double valueDDot = 0.0;

   public SinewaveFunctionGenerator()
   {
   }

   public void resetAngle()
   {
      angle = 0.0;
   }

   public void integrateAngle(double dt)
   {
      setAngle(getAngle() + getAngleDot() * dt);
   }

   private void markDirty()
   {
      outputDirty = true;
   }

   private void updateOutput()
   {
      if (!outputDirty)
         return;
      outputDirty = false;
      value = offset.getValue() + amplitude.getValue() * Math.sin(angle + phase.getValue());
      double angleDot = getAngleDot();
      valueDot = angleDot * amplitude.getValue() * Math.cos(angle + phase.getValue());
      valueDDot = -angleDot * angleDot * amplitude.getValue() * Math.sin(angle + phase.getValue());
   }

   public void setAngle(double angle)
   {
      if (angle == this.angle)
         return;
      this.angle = angle;
      markDirty();
   }

   public void setOffset(double offset)
   {
      setOffset(() -> offset);
   }

   public void setOffset(DoubleProvider offset)
   {
      this.offset = offset;
      markDirty();
   }

   public void setAmplitude(double amplitude)
   {
      setAmplitude(() -> amplitude);
   }

   public void setAmplitude(DoubleProvider amplitude)
   {
      this.amplitude = amplitude;
      markDirty();
   }

   public void setFrequency(double frequency)
   {
      setFrequency(() -> frequency);
   }

   public void setFrequency(DoubleProvider frequency)
   {
      this.frequency = frequency;
      markDirty();
   }

   public void setPhase(double phase)
   {
      setPhase(() -> phase);
   }

   public void setPhase(DoubleProvider phase)
   {
      this.phase = phase;
      markDirty();
   }

   public double getAngle()
   {
      return angle;
   }

   public double getAngleDot()
   {
      return 2.0 * Math.PI * frequency.getValue();
   }

   public double getValue()
   {
      updateOutput();
      return value;
   }

   public double getValueDot()
   {
      updateOutput();
      return valueDot;
   }

   public double getValueDDot()
   {
      updateOutput();
      return valueDDot;
   }
}
