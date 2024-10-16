package us.ihmc.yoVariables.filters;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

/**
 * This variable is designed to compute the velocity of a position signal that may contain backlash. It computes the velocity of the position using a
 * {@link FilteredFiniteDifferenceYoVariable}. When that velocity changes sign, the input is computed to have changed directions, and it enters a "slop" period.
 * During the slop period, the output of this variable returns the previous velocity value, which is slowly scaled towards the new measured value over the course of
 * the slop duration.
 */
public class BacklashCompensatingVelocityYoVariable extends YoDouble implements ProcessingYoVariable
{
   private final double dt;

   private final FilteredFiniteDifferenceYoVariable finiteDifferenceVelocity;

   private final DoubleProvider alphaVariable;
   private final DoubleProvider position;

   private final YoDouble lastPosition;
   private final YoBoolean hasBeenCalled;

   private final YoEnum<BacklashState> backlashState;
   private final DoubleProvider slopTime;

   private final YoDouble timeSinceSloppy;

   public BacklashCompensatingVelocityYoVariable(String name,
                                                 String description,
                                                 DoubleProvider alphaVariable,
                                                 DoubleProvider positionVariable,
                                                 double dt,
                                                 DoubleProvider slopTime,
                                                 YoRegistry registry)
   {

      super(name, description, registry);
      finiteDifferenceVelocity = new FilteredFiniteDifferenceYoVariable(name + "finiteDifferenceVelocity", "", alphaVariable, dt, registry);

      this.hasBeenCalled = VariableTools.createHasBeenCalledYoBoolean(name, "", registry);

      backlashState = new YoEnum<>(name + "BacklashState", registry, BacklashState.class, true);
      backlashState.set(null);
      timeSinceSloppy = new YoDouble(name + "TimeSinceSloppy", registry);

      position = positionVariable;

      this.alphaVariable = alphaVariable;
      this.slopTime = slopTime;

      this.dt = dt;

      lastPosition = new YoDouble(name + "_lastPosition", registry);

      reset();
   }

   public BacklashCompensatingVelocityYoVariable(String name,
                                                 String description,
                                                 YoDouble alphaVariable,
                                                 double dt,
                                                 YoDouble slopTime,
                                                 YoRegistry registry)
   {
      this(name, description, alphaVariable, null, dt, slopTime, registry);
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
      if (position == null)
      {
         throw new NullPointerException(
               "BacklashCompensatingVelocityYoVariable must be constructed with a non null position variable to call update(), otherwise use update(double)");
      }

      update(position.getValue());
   }

   public void update(double currentPosition)
   {
      if (!hasBeenCalled.getBooleanValue())
      {
         hasBeenCalled.set(true);
         lastPosition.set(currentPosition);
         set(0.0);
      }

      finiteDifferenceVelocity.update(currentPosition);
      double velocityFromFiniteDifferences = finiteDifferenceVelocity.getDoubleValue();

      // increment time.
      timeSinceSloppy.add(dt);

      // No estimate of the backlash state is available, so initialize it based on the current estimated velocity.
      if (backlashState.getEnumValue() == null)
      {
         if (velocityFromFiniteDifferences < 0.0)
            backlashState.set(BacklashState.BACKWARD_OK);
         else if (velocityFromFiniteDifferences > 0.0)
            backlashState.set(BacklashState.FORWARD_OK);
      }
      else
      {
         switch (backlashState.getEnumValue())
         {
            case BACKWARD_OK:
            {
               if (velocityFromFiniteDifferences > 0.0)
               {
                  timeSinceSloppy.set(0.0);
                  backlashState.set(BacklashState.FORWARD_SLOP);
               }

               break;
            }

            case FORWARD_OK:
            {
               if (velocityFromFiniteDifferences < 0.0)
               {
                  timeSinceSloppy.set(0.0);
                  backlashState.set(BacklashState.BACKWARD_SLOP);
               }

               break;
            }

            case BACKWARD_SLOP:
            {
               if (velocityFromFiniteDifferences > 0.0)
               {
                  timeSinceSloppy.set(0.0);
                  backlashState.set(BacklashState.FORWARD_SLOP);
               }
               else if (timeSinceSloppy.getDoubleValue() > slopTime.getValue())
               {
                  backlashState.set(BacklashState.BACKWARD_OK);
                  timeSinceSloppy.set(0.0);
               }

               break;
            }

            case FORWARD_SLOP:
            {
               if (velocityFromFiniteDifferences < 0.0)
               {
                  timeSinceSloppy.set(0.0);
                  backlashState.set(BacklashState.BACKWARD_SLOP);
               }
               else if (timeSinceSloppy.getDoubleValue() > slopTime.getValue())
               {
                  backlashState.set(BacklashState.FORWARD_OK);
                  timeSinceSloppy.set(0.0);
               }

               break;
            }
         }
      }

      double difference = currentPosition - lastPosition.getDoubleValue();

      // During the slop period, we want to increase the difference from 0 to 1, so that there is no velocity at the beginning of slop, and the velocity is
      // increased to the current finite differenced value at the end of the slop period.
      if (backlashState.getEnumValue() != null && backlashState.getEnumValue().isInBacklash())
      {
         double alpha = timeSinceSloppy.getDoubleValue() / slopTime.getValue();
         alpha = MathTools.clamp(alpha, 0.0, 1.0);
         if (Double.isNaN(alpha))
            alpha = 1.0;

         difference = alpha * difference;
      }

      // The difference is then finite differenced and alpha filtered
      updateUsingDifference(difference);
      lastPosition.set(currentPosition);
   }

   private void updateUsingDifference(double difference)
   {
      double previousFilteredDerivative = getDoubleValue();
      double currentRawDerivative = difference / dt;

      this.set(EuclidCoreTools.interpolate(currentRawDerivative, previousFilteredDerivative, alphaVariable.getValue()));
   }
}
