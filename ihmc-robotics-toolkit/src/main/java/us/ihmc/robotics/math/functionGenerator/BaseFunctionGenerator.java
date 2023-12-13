package us.ihmc.robotics.math.functionGenerator;

import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.yoVariables.providers.DoubleProvider;

public abstract class BaseFunctionGenerator
{
   public static final DoubleProvider zeroDoubleProvider = () -> 0.0;
   public static final double twoPi = 2.0 * Math.PI;

   private DoubleProvider offset = zeroDoubleProvider;
   private DoubleProvider amplitude = zeroDoubleProvider;
   private DoubleProvider frequency = zeroDoubleProvider;
   private DoubleProvider phase = zeroDoubleProvider;

   private double angle = 0.0;

   private boolean outputDirty = true;
   private double value = 0.0;
   private double valueDot = 0.0;
   private double valueDDot = 0.0;

   public BaseFunctionGenerator()
   {
   }

   public void resetAngle()
   {
      angle = 0.0;
   }

   public void integrateAngle(double dt)
   {
      setAngle(getAngle() + getAngleDot() * dt);
      markDirty();
   }

   void markDirty()
   {
      outputDirty = true;
   }

   private void updateOutput()
   {
      if (!outputDirty)
         return;
      outputDirty = false;
      value = computeValue();
      valueDot = computeValueDot();
      valueDDot = computeValueDDot();
   }

   protected abstract double computeValue();

   protected abstract double computeValueDot();

   protected abstract double computeValueDDot();

   public void setAngle(double angle)
   {
      angle = AngleTools.shiftAngleToStartOfRange(angle, 0.0);

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

   public double getOffset()
   {
      return offset.getValue();
   }

   public double getAmplitude()
   {
      return amplitude.getValue();
   }

   public double getFrequency()
   {
      return frequency.getValue();
   }

   public double getPhase()
   {
      return phase.getValue();
   }

   public double getAngle()
   {
      return angle;
   }

   public double getAngleDot()
   {
      return twoPi * frequency.getValue();
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
