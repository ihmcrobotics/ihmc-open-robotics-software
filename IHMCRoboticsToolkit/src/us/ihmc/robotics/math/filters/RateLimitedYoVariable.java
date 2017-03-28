package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;


public class RateLimitedYoVariable extends DoubleYoVariable
{
   private final DoubleYoVariable maxRateVariable;

   private final DoubleYoVariable position;
   private final BooleanYoVariable limited;

   private final double dt;

   private final BooleanYoVariable hasBeenCalled;

   public RateLimitedYoVariable(String name, YoVariableRegistry registry, double maxRate, double dt)
   {
      super(name, registry);
      this.hasBeenCalled = new BooleanYoVariable(name + "HasBeenCalled", registry);
      this.limited = new BooleanYoVariable(name  + "Limited", registry);

      this.maxRateVariable = new DoubleYoVariable(name + "MaxRate", registry);
      this.maxRateVariable.set(maxRate);

      this.position = null;

      this.dt = dt;

      reset();
   }

   public RateLimitedYoVariable(String name, YoVariableRegistry registry, DoubleYoVariable maxRateVariable, double dt)
   {
      super(name, registry);
      this.hasBeenCalled = new BooleanYoVariable(name + "HasBeenCalled", registry);
      this.limited = new BooleanYoVariable(name  + "Limited", registry);

      this.maxRateVariable = maxRateVariable;
      this.position = null;

      this.dt = dt;
      reset();
   }

   public RateLimitedYoVariable(String name, YoVariableRegistry registry, double maxRate, DoubleYoVariable positionVariable, double dt)
   {
      super(name, registry);
      this.hasBeenCalled = new BooleanYoVariable(name + "HasBeenCalled", registry);
      this.limited = new BooleanYoVariable(name  + "Limited", registry);

      position = positionVariable;

      this.maxRateVariable = new DoubleYoVariable(name + "MaxRate", registry);
      this.maxRateVariable.set(maxRate);

      this.dt = dt;

      reset();
   }

   public RateLimitedYoVariable(String name, YoVariableRegistry registry, DoubleYoVariable maxRateVariable, DoubleYoVariable positionVariable, double dt)
   {
      super(name, registry);
      this.hasBeenCalled = new BooleanYoVariable(name + "HasBeenCalled", registry);
      this.limited = new BooleanYoVariable(name  + "Limited", registry);

      position = positionVariable;
      this.maxRateVariable = maxRateVariable;

      this.dt = dt;

      reset();
   }

   public void setMaxRate(double maxRate)
   {
      this.maxRateVariable.set(maxRate);
   }

   public void reset()
   {
      hasBeenCalled.set(false);
   }

   public void update()
   {
      if (position == null)
      {
         throw new NullPointerException("YoAlphaFilteredVariable must be constructed with a non null "
               + "position variable to call update(), otherwise use update(double)");
      }

      update(position.getDoubleValue());
   }

   public void update(double currentPosition)
   {
      if (!hasBeenCalled.getBooleanValue())
      {
         hasBeenCalled.set(true);
         set(currentPosition);
      }

      if (maxRateVariable.getDoubleValue() < 0)
         throw new RuntimeException("The maxRate parameter in the RateLimitedYoVariable cannot be negative.");

      double difference = currentPosition - getDoubleValue();
      if (Math.abs(difference) > maxRateVariable.getDoubleValue() * dt)
      {
         difference = Math.signum(difference) * maxRateVariable.getDoubleValue() * dt;
         this.limited.set(true);
      }
      else
         this.limited.set(false);

      set(getDoubleValue() + difference);
   }
}