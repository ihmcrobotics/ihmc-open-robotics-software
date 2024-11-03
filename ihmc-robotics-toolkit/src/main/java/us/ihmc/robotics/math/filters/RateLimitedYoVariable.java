package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class RateLimitedYoVariable extends YoDouble
{
   private final DoubleProvider maxRateVariable;

   private final YoDouble position;
   private final YoBoolean limited;

   private final double dt;

   private final YoBoolean hasBeenCalled;

   public RateLimitedYoVariable(String name, YoRegistry registry, double maxRate, double dt)
   {
      super(name, registry);
      this.hasBeenCalled = new YoBoolean(name + "HasBeenCalled", registry);
      this.limited = new YoBoolean(name + "Limited", registry);

      YoDouble maxRateVariable = new YoDouble(name + "MaxRate", registry);
      maxRateVariable.set(maxRate);
      this.maxRateVariable = maxRateVariable;
      
      this.position = null;

      this.dt = dt;

      reset();
   }

   public RateLimitedYoVariable(String name, YoRegistry registry, DoubleProvider maxRateVariable, double dt)
   {
      super(name, registry);
      this.hasBeenCalled = new YoBoolean(name + "HasBeenCalled", registry);
      this.limited = new YoBoolean(name + "Limited", registry);

      this.maxRateVariable = maxRateVariable;
      this.position = null;

      this.dt = dt;
      reset();
   }

   public RateLimitedYoVariable(String name, YoRegistry registry, double maxRate, YoDouble positionVariable, double dt)
   {
      super(name, registry);
      this.hasBeenCalled = new YoBoolean(name + "HasBeenCalled", registry);
      this.limited = new YoBoolean(name + "Limited", registry);

      position = positionVariable;

      YoDouble maxRateVariable = new YoDouble(name + "MaxRate", registry);
      maxRateVariable.set(maxRate);
      this.maxRateVariable = maxRateVariable;
      
      this.dt = dt;

      reset();
   }

   public RateLimitedYoVariable(String name, YoRegistry registry, DoubleProvider maxRateVariable, YoDouble positionVariable, double dt)
   {
      super(name, registry);
      this.hasBeenCalled = new YoBoolean(name + "HasBeenCalled", registry);
      this.limited = new YoBoolean(name + "Limited", registry);

      position = positionVariable;
      this.maxRateVariable = maxRateVariable;

      this.dt = dt;

      reset();
   }

   public void reset()
   {
      hasBeenCalled.set(false);
   }

   public void update()
   {
      if (position == null)
      {
         throw new NullPointerException(getClass().getSimpleName() + " must be constructed with a non null "
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

      if (maxRateVariable.getValue() < 0)
         throw new RuntimeException("The maxRate parameter in the RateLimitedYoVariable cannot be negative.");

      double difference = currentPosition - getDoubleValue();
      if (Math.abs(difference) > maxRateVariable.getValue() * dt)
      {
         difference = Math.signum(difference) * maxRateVariable.getValue() * dt;
         this.limited.set(true);
      }
      else
         this.limited.set(false);

      set(getDoubleValue() + difference);
   }
}