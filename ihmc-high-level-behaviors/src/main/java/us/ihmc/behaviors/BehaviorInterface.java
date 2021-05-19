package us.ihmc.behaviors;

import us.ihmc.yoVariables.registry.YoRegistry;

public interface BehaviorInterface
{
   void setEnabled(boolean enabled);

   default YoRegistry getYoRegistry()
   {
      return null;
   }

   default void destroy()
   {

   }
}
