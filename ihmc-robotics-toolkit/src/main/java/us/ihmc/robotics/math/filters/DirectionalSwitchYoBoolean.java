package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class DirectionalSwitchYoBoolean extends YoBoolean
{
   private final YoBoolean lastValue;
   private final Direction direction;

   public DirectionalSwitchYoBoolean(String name, YoRegistry registry)
   {
      this(name, Direction.FALSE_TO_TRUE, registry);
   }

   public DirectionalSwitchYoBoolean(String name, Direction direction, YoRegistry registry)
   {
      super(name, registry);
      this.lastValue = new YoBoolean(name + "_LastValue", registry);
      this.direction = direction;
   }

   public void update(boolean currentValue)
   {
      switch (direction)
      {
         // If the last value was false and the current value is true, set the filtered value to true
         case FALSE_TO_TRUE -> set(!lastValue.getValue() && currentValue);
         // If the last value was true and the current value is false, set the filtered value to false
         case TRUE_TO_FALSE -> set(lastValue.getValue() && !currentValue);
         case BOTH -> set(lastValue.getValue() != currentValue);
      }
      lastValue.set(currentValue);
   }

   public enum Direction
   {
      FALSE_TO_TRUE, TRUE_TO_FALSE, BOTH
   }
}
