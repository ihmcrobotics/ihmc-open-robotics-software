package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class RateLimitedYoVariable extends YoDouble
{
   private final DoubleProvider maxRateVariable;

   private final DoubleProvider unlimitedPosition;
   private final YoBoolean limited;

   private final double dt;

   private final YoBoolean hasBeenCalled;

   public RateLimitedYoVariable(String name, YoRegistry registry, double maxRate, double dt)
   {
      this(name, registry, maxRate, null, dt);
   }

   public RateLimitedYoVariable(String name, YoRegistry registry, DoubleProvider maxRateVariable, double dt)
   {
      this(name, registry, maxRateVariable, null, dt);
   }

   public RateLimitedYoVariable(String name, YoRegistry registry, double maxRate, DoubleProvider positionVariable, double dt)
   {
      this(name, registry, VariableTools.createMaxRateYoDouble(name, "", maxRate, registry), positionVariable, dt);
   }

   public RateLimitedYoVariable(String name, YoRegistry registry, DoubleProvider maxRateVariable, DoubleProvider unlimitedPosition, double dt)
   {
      super(name, registry);

      this.hasBeenCalled = VariableTools.createHasBeenCalledYoBoolean(name, "", registry);
      this.limited = VariableTools.createLimitedCalledYoBoolean(name, "", registry);

      this.unlimitedPosition = unlimitedPosition;
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
      if (unlimitedPosition == null)
      {
         throw new NullPointerException(getClass().getSimpleName() + " must be constructed with a non null "
               + "position variable to call update(), otherwise use update(double)");
      }

      update(unlimitedPosition.getValue());
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