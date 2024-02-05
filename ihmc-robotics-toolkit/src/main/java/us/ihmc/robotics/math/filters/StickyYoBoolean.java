package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class StickyYoBoolean extends YoBoolean
{
   private int counter;
   private int ticksToKeepTrue;

   public StickyYoBoolean(String name, int ticksToKeepTrue, YoRegistry registry)
   {
      super(name, registry);
      counter = 0;
      this.ticksToKeepTrue = ticksToKeepTrue;
   }

   public void activate()
   {
      set(true);
      counter = 0;
   }

   public void update()
   {
      if (getValue() && counter < ticksToKeepTrue)
      {
         set(true);
         counter++;
      }
      else
         set(false);
   }
}
