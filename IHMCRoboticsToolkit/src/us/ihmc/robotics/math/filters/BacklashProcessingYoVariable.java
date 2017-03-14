package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;

/**
 * This does essentially the same as RevisedBacklashCompensatingVelocityYoVariable, except it takes a velocity signal as input.
 *
 */
public class BacklashProcessingYoVariable extends DoubleYoVariable implements ProcessingYoVariable
{
   private final DoubleYoVariable velocity;

   private final BooleanYoVariable hasBeenCalled;

   private final EnumYoVariable<BacklashState> backlashState;
   private final DoubleYoVariable slopTime;

   private final DoubleYoVariable timeSinceSloppy;

   private final double dt;

   public BacklashProcessingYoVariable(String name, String description, double dt, DoubleYoVariable slopTime, YoVariableRegistry registry)
   {
      this(name, description, null, dt, slopTime, registry);
   }

   public BacklashProcessingYoVariable(String name, String description, DoubleYoVariable velocityVariable, double dt, DoubleYoVariable slopTime,
         YoVariableRegistry registry)
   {
      super(name, description, registry);

      this.hasBeenCalled = new BooleanYoVariable(name + "HasBeenCalled", registry);

      backlashState = new EnumYoVariable<BacklashState>(name + "BacklashState", registry, BacklashState.class, true);
      backlashState.set(null);
      timeSinceSloppy = new DoubleYoVariable(name + "TimeSinceSloppy", registry);

      velocity = velocityVariable;

      this.slopTime = slopTime;

      this.dt = dt;

      reset();
   }

   public void reset()
   {
      hasBeenCalled.set(false);
      backlashState.set(null);
   }

   public void update()
   {
      if (velocity == null)
      {
         throw new NullPointerException(
               "BacklashProcessingYoVariable must be constructed with a non null " + "velocity variable to call update(), otherwise use update(double)");
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
         else if (timeSinceSloppy.getDoubleValue() > slopTime.getDoubleValue())
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
         else if (timeSinceSloppy.getDoubleValue() > slopTime.getDoubleValue())
         {
            backlashState.set(BacklashState.FORWARD_OK);
         }

         break;
      }
      }

      double percent = timeSinceSloppy.getDoubleValue() / slopTime.getDoubleValue();
      percent = MathTools.clamp(percent, 0.0, 1.0);
      if (Double.isNaN(percent) || slopTime.getDoubleValue() < dt)
         percent = 1.0;

      this.set(percent * currentVelocity);
   }

   public void setSlopTime(double slopTime)
   {
      this.slopTime.set(slopTime);
   }

   private enum BacklashState
   {
      BACKWARD_OK, FORWARD_OK, BACKWARD_SLOP, FORWARD_SLOP;
   }
}
