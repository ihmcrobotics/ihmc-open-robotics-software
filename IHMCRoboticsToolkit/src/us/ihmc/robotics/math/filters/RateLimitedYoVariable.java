package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;


public class RateLimitedYoVariable extends DoubleYoVariable
{
   private double maxRate;
   private final DoubleYoVariable maxRateVariable;

   private final DoubleYoVariable position;

   private final double dt;

   private final BooleanYoVariable hasBeenCalled;

   public RateLimitedYoVariable(String name, YoVariableRegistry registry, double maxRate, double dt)
   {
      super(name, registry);
      this.hasBeenCalled = new BooleanYoVariable(name + "HasBeenCalled", registry);

      this.maxRate = maxRate;

      this.maxRateVariable = null;
      this.position = null;

      this.dt = dt;

      reset();
   }

   public RateLimitedYoVariable(String name, YoVariableRegistry registry, DoubleYoVariable maxRateVariable, double dt)
   {
      super(name, registry);
      this.hasBeenCalled = new BooleanYoVariable(name + "HasBeenCalled", registry);

      this.maxRate = 0.0;

      this.maxRateVariable = maxRateVariable;
      this.position = null;

      this.dt = dt;
      reset();
   }

   public RateLimitedYoVariable(String name, YoVariableRegistry registry, double maxRate, DoubleYoVariable positionVariable, double dt)
   {
      super(name, registry);
      this.hasBeenCalled = new BooleanYoVariable(name + "HasBeenCalled", registry);

      this.maxRate = maxRate;
      position = positionVariable;

      this.maxRateVariable = null;
      this.dt = dt;

      reset();
   }

   public RateLimitedYoVariable(String name, YoVariableRegistry registry, DoubleYoVariable maxRateVariable, DoubleYoVariable positionVariable, double dt)
   {
      super(name, registry);
      this.hasBeenCalled = new BooleanYoVariable(name + "HasBeenCalled", registry);

      position = positionVariable;
      this.maxRateVariable = maxRateVariable;

      this.maxRate = 0.0;
      this.dt = dt;

      reset();
   }

   public void setMaxRate(double maxRate)
   {
      if (maxRateVariable != null)
         maxRateVariable.set(maxRate);

      this.maxRate = maxRate;
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

      final double maxRateToUse;

      if (maxRateVariable != null)
         maxRateToUse = maxRateVariable.getDoubleValue();
      else
         maxRateToUse = maxRate;

      if (maxRateToUse < 0)
         throw new RuntimeException("The maxRate parameter in the RateLimitedYoVariable constructor cannot be negative.");

      double difference = currentPosition - getDoubleValue();
      if (Math.abs(difference) > maxRateToUse * dt)
         difference = Math.signum(difference) * maxRateToUse * dt;

      set(getDoubleValue() + difference);
   }
}