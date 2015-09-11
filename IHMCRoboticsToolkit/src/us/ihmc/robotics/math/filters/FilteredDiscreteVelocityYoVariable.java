package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;

/**
 * @author jrebula
 *         <p>
 *         A YoFilteredVelocityVariable is a filtered velocity of a position.
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
public class FilteredDiscreteVelocityYoVariable extends DoubleYoVariable
{
   private final double alpha;

   private final DoubleYoVariable time;

   private final DoubleYoVariable alphaVariable;
   private final DoubleYoVariable position;

   private final DoubleYoVariable lastUpdateTime;
   private final EnumYoVariable<Direction> lastUpdateDirection;
   private final DoubleYoVariable unfilteredVelocity;

   private final DoubleYoVariable lastPosition;
   private boolean hasBeenCalled;

   public FilteredDiscreteVelocityYoVariable(String name, String description, double alpha, DoubleYoVariable time, YoVariableRegistry registry)
   {
      super(name, description, registry);

      this.alpha = alpha;
      this.alphaVariable = null;
      this.position = null;

      this.time = time;

      lastPosition = new DoubleYoVariable(name + "_lastPosition", registry);
      lastUpdateTime = new DoubleYoVariable(name + "_lastUpdateTime", registry);
      lastUpdateDirection = EnumYoVariable.create(name + "_lastUpdateDirection", Direction.class, registry);
      unfilteredVelocity = new DoubleYoVariable(name + "_unfilteredVelocity", registry);

      reset();
   }

   public FilteredDiscreteVelocityYoVariable(String name, String description, double alpha, DoubleYoVariable positionVariable, DoubleYoVariable time,
         YoVariableRegistry registry)
   {
      super(name, description, registry);

      this.alpha = alpha;
      this.position = positionVariable;

      this.alphaVariable = null;

      this.time = time;

      lastPosition = new DoubleYoVariable(name + "_lastPosition", registry);
      lastUpdateTime = new DoubleYoVariable(name + "_lastUpdateTime", registry);
      lastUpdateDirection = EnumYoVariable.create(name + "_lastUpdateDirection", Direction.class, registry);
      unfilteredVelocity = new DoubleYoVariable(name + "_unfilteredVelocity", registry);

      reset();
   }

   public FilteredDiscreteVelocityYoVariable(String name, String description, DoubleYoVariable alphaVariable, DoubleYoVariable positionVariable,
         DoubleYoVariable time, YoVariableRegistry registry)
   {
      super(name, description, registry);
      position = positionVariable;
      this.alphaVariable = alphaVariable;
      this.alpha = 0.0;

      this.time = time;

      lastPosition = new DoubleYoVariable(name + "_lastPosition", registry);
      lastUpdateTime = new DoubleYoVariable(name + "_lastUpdateTime", registry);
      lastUpdateDirection = EnumYoVariable.create(name + "_lastUpdateDirection", Direction.class, registry);
      unfilteredVelocity = new DoubleYoVariable(name + "_unfilteredVelocity", registry);

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

      double alphaToUse = alpha;

      if (alphaVariable != null)
      {
         alphaToUse = alphaVariable.getDoubleValue();
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
      set(alphaToUse * getDoubleValue() + (1.0 - alphaToUse) * unfilteredVelocity.getDoubleValue());

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
