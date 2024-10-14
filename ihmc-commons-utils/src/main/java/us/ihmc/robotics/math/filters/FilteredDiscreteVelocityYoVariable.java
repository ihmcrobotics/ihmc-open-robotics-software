package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

/**
 * @author jrebula
 *         <p>
 *         A {@link FilteredDiscreteVelocityYoVariable} is a filtered velocity of a position. That is, it takes the finite difference of the position signal to
 *         compute a velocity, it then filters the velocity using an alpha filter
 *         Either a YoVariable holding the position is passed in to the
 *         constructor and update() is called every tick, or update(double) is
 *         called every tick. The YoFilteredVelocityVariable updates it's val
 *         with the current velocity after a filter of
 *         </p>
 *
 * <pre>
 *            vel_{n} = alpha * vel{n-1} + (1 - alpha) * (pos_{n} - pos_{n-1})
 * </pre>
 *
 */
public class FilteredDiscreteVelocityYoVariable extends YoDouble
{

   private final YoDouble time;

   private final DoubleProvider alphaVariable;
   private final YoDouble position;

   private final YoDouble lastUpdateTime;
   private final YoEnum<Direction> lastUpdateDirection;
   private final YoDouble unfilteredVelocity;

   private final YoDouble lastPosition;
   private boolean hasBeenCalled;

   public FilteredDiscreteVelocityYoVariable(String name, String description, double alpha, YoDouble time, YoRegistry registry)
   {
      this(name, description, alpha, null, time, registry);
   }

   public FilteredDiscreteVelocityYoVariable(String name, String description, double alpha, YoDouble positionVariable, YoDouble time,
         YoRegistry registry)
   {
      this(name, description, VariableTools.createAlphaYoDouble(name, "", alpha, registry), positionVariable, time, registry);
   }

   public FilteredDiscreteVelocityYoVariable(String name, String description, DoubleProvider alphaVariable, YoDouble positionVariable,
                                             YoDouble time, YoRegistry registry)
   {
      super(name, description, registry);
      position = positionVariable;
      this.alphaVariable = alphaVariable;

      this.time = time;

      lastPosition = new YoDouble(name + "_lastPosition", registry);
      lastUpdateTime = new YoDouble(name + "_lastUpdateTime", registry);
      lastUpdateDirection = new YoEnum<>(name + "_lastUpdateDirection", registry, Direction.class);
      unfilteredVelocity = new YoDouble(name + "_unfilteredVelocity", registry);

      reset();
   }

   public void reset()
   {
      hasBeenCalled = false;
   }

   public void update()
   {
      if (position == null)
      {
         throw new NullPointerException("YoFilteredVelocityVariable must be constructed with a non null "
               + "position variable to call update(), otherwise use update(double)");
      }

      update(position.getDoubleValue());
   }

   public void update(double currentPosition)
   {
      if (!hasBeenCalled)
      {
         hasBeenCalled = true;

         //       lastPosition = currentPosition;
         lastPosition.set(currentPosition);
         lastUpdateTime.set(time.getDoubleValue());
         lastUpdateDirection.set(Direction.NONE);
      }

      // Figure out if the count changed and if so, if the direction changed or not and then update the direction:
      boolean countChanged = false;
      if (currentPosition != lastPosition.getDoubleValue())
         countChanged = true;

      // If the count changed, figure out if the direction changed:

      boolean directionChanged = false;
      if (countChanged)
      {
         if (currentPosition > lastPosition.getDoubleValue())
         {
            if (lastUpdateDirection.getEnumValue() != Direction.FORWARD)
               directionChanged = true;
            lastUpdateDirection.set(Direction.FORWARD);
         }
         else if (currentPosition < lastPosition.getDoubleValue())
         {
            if (lastUpdateDirection.getEnumValue() != Direction.BACKWARD)
               directionChanged = true;
            lastUpdateDirection.set(Direction.BACKWARD);
         }
      }

      // If the direction changed, then the velocity is set to zero:
      if (directionChanged)
      {
         unfilteredVelocity.set(0.0);
      }

      // If the direction hasn't changed, but the count changed then compute the velocity based on the time since last update:
      else if (countChanged)
      {
         double diffTime = time.getDoubleValue() - lastUpdateTime.getDoubleValue();
         if (diffTime < 1e-7)
            unfilteredVelocity.set(0.0);
         else
         {
            unfilteredVelocity.set((currentPosition - lastPosition.getDoubleValue()) / diffTime);
         }
      }

      else
      {
         // If the count hasn't changed, then not quite sure what the velocity is.
         // We could just use the current velocity, but really should try to figure out if things have been slowing down or not.
         // For now, multiply by some largish fraction, just to make sure it does trail off to zero if the velocity stops quickly.
         unfilteredVelocity.set(0.99 * unfilteredVelocity.getDoubleValue());
      }

      // Low pass alpha filter it...
      set(EuclidCoreTools.interpolate(unfilteredVelocity.getDoubleValue(), getDoubleValue(), alphaVariable.getValue()));

      // Remember the position and the currentTime if the countChanged:

      if (countChanged)
      {
         lastPosition.set(currentPosition);
         lastUpdateTime.set(time.getDoubleValue());
      }
   }

   public double getUnfilteredVelocity()
   {
      return unfilteredVelocity.getDoubleValue();
   }

   private enum Direction
   {
      NONE, FORWARD, BACKWARD;
   }

}
