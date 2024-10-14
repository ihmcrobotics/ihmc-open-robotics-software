package us.ihmc.robotics.math.filters;

import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

/**
 * This class is designed to perform an estimate of a velocity signal that may contain backlash. It does essentially the same as
 * {@link BacklashCompensatingVelocityYoVariable}, except it takes a velocity signal as input.
 *
 * It works by zeroing out the estimated velocity whenever the finite-differenced velocity changes sign. It then ramps this value back up to the value returned
 * by finite estimating over the course of the slop duration. It assumes that the slop time is some fixed, constant period when the estimate is unreliable.
 */
public class BacklashProcessingYoVariable extends YoDouble implements ProcessingYoVariable
{
   private final YoDouble velocity;

   private final YoBoolean hasBeenCalled;

   private final YoEnum<BacklashState> backlashState;
   private final DoubleProvider slopTime;

   private final YoDouble timeSinceSloppy;

   private final double dt;

   public BacklashProcessingYoVariable(String name, String description, double dt, DoubleProvider slopTime, YoRegistry registry)
   {
      this(name, description, null, dt, slopTime, registry);
   }

   public BacklashProcessingYoVariable(String name, String description, YoDouble velocityVariable, double dt, DoubleProvider slopTime, YoRegistry registry)
   {
      super(name, description, registry);

      this.hasBeenCalled = VariableTools.createHasBeenCalledYoBoolean(name, "", registry);

      backlashState = new YoEnum<>(name + "BacklashState", registry, BacklashState.class, true);
      backlashState.set(null);
      timeSinceSloppy = new YoDouble(name + "TimeSinceSloppy", registry);

      velocity = velocityVariable;

      this.slopTime = slopTime;

      this.dt = dt;

      reset();
   }

   @Override
   public void reset()
   {
      hasBeenCalled.set(false);
      backlashState.set(null);
   }

   @Override
   public void update()
   {
      if (velocity == null)
      {
         throw new NullPointerException(
               "BacklashProcessingYoVariable must be constructed with a non null velocity variable to call update(), otherwise use update(double)");
      }

      update(velocity.getDoubleValue());
   }

   public void update(double currentVelocity)
   {
      if (backlashState.getEnumValue() == null)
      {
         backlashState.set(BacklashState.FORWARD_OK);
      }

      if (!hasBeenCalled.getBooleanValue())
      {
         hasBeenCalled.set(true);
         set(currentVelocity);
      }

      timeSinceSloppy.add(dt);

      switch (backlashState.getEnumValue())
      {
         case BACKWARD_OK:
         {
            if (currentVelocity > 0.0)
            {
               timeSinceSloppy.set(0.0);
               backlashState.set(BacklashState.FORWARD_SLOP);
            }

            break;
         }

         case FORWARD_OK:
         {
            if (currentVelocity < 0.0)
            {
               timeSinceSloppy.set(0.0);
               backlashState.set(BacklashState.BACKWARD_SLOP);
            }

            break;
         }

         case BACKWARD_SLOP:
         {
            if (currentVelocity > 0.0)
            {
               timeSinceSloppy.set(0.0);
               backlashState.set(BacklashState.FORWARD_SLOP);
            }
            else if (timeSinceSloppy.getDoubleValue() > slopTime.getValue())
            {
               backlashState.set(BacklashState.BACKWARD_OK);
            }

            break;
         }

         case FORWARD_SLOP:
         {
            if (currentVelocity < 0.0)
            {
               timeSinceSloppy.set(0.0);
               backlashState.set(BacklashState.BACKWARD_SLOP);
            }
            else if (timeSinceSloppy.getDoubleValue() > slopTime.getValue())
            {
               backlashState.set(BacklashState.FORWARD_OK);
            }

            break;
         }
      }

      double percent = timeSinceSloppy.getDoubleValue() / slopTime.getValue();
      percent = MathTools.clamp(percent, 0.0, 1.0);
      if (Double.isNaN(percent) || slopTime.getValue() < dt)
         percent = 1.0;

      this.set(percent * currentVelocity);
   }
}
