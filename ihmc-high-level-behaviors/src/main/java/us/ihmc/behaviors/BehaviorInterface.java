package us.ihmc.behaviors;

import us.ihmc.yoVariables.registry.YoRegistry;

public interface BehaviorInterface
{
//   private final BehaviorHelper helper;
//
//   public BehaviorInterface(BehaviorHelper helper)
//   {
//      this.helper = helper;
//   }

   public abstract void setEnabled(boolean enabled);

   public default YoRegistry getYoRegistry()
   {
      return null;
   }

   public default void destroy()
   {
      // TODO: Destroy behavior helper
      // helper.destroy();
   }
}
