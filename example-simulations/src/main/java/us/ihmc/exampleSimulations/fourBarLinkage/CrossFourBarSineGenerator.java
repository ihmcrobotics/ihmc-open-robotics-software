package us.ihmc.exampleSimulations.fourBarLinkage;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

class CrossFourBarSineGenerator
{
   private final YoDouble position;
   private final YoDouble velocity;
   private final YoDouble acceleration;

   private final YoDouble amplitude;
   private final YoDouble frequency;
   private final YoDouble phase;
   private final YoDouble offset;

   private final DoubleProvider time;

   public CrossFourBarSineGenerator(String name, DoubleProvider timeProvider, YoRegistry registry)
   {
      this.time = timeProvider;
      position = new YoDouble(name + "Position", registry);
      velocity = new YoDouble(name + "Velocity", registry);
      acceleration = new YoDouble(name + "Acceleration", registry);

      amplitude = new YoDouble(name + "Amplitude", registry);
      frequency = new YoDouble(name + "Frequency", registry);
      phase = new YoDouble(name + "Phase", registry);
      offset = new YoDouble(name + "Offset", registry);
   }

   public void setAmplitude(double amplitude)
   {
      this.amplitude.set(amplitude);
   }

   public void setFrequency(double frequency)
   {
      this.frequency.set(frequency);
   }

   public void setPhase(double phase)
   {
      this.phase.set(phase);
   }

   public void setOffset(double offset)
   {
      this.offset.set(offset);
   }

   public void update()
   {
      double omega = 2.0 * Math.PI * frequency.getValue();
      position.set(offset.getValue() + amplitude.getValue() * Math.sin(omega * time.getValue() + phase.getValue()));
      velocity.set(omega * amplitude.getValue() * Math.cos(omega * time.getValue() + phase.getValue()));
      acceleration.set(-omega * omega * amplitude.getValue() * Math.sin(omega * time.getValue() + phase.getValue()));
   }

   public double getPosition()
   {
      return position.getValue();
   }

   public double getVelocity()
   {
      return velocity.getValue();
   }

   public double getAcceleration()
   {
      return acceleration.getValue();
   }

   public double getAmplitude()
   {
      return amplitude.getDoubleValue();
   }

   public double getFrequency()
   {
      return frequency.getValue();
   }

   public double getPhase()
   {
      return phase.getValue();
   }

   public double getOffset()
   {
      return offset.getValue();
   }
}
