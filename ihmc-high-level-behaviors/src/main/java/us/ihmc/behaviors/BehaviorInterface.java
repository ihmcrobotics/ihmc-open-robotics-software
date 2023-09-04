package us.ihmc.behaviors;

import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeControlFlowNodeBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public interface BehaviorInterface extends BehaviorTreeControlFlowNodeBasics
{
   public default YoRegistry getYoRegistry()
   {
      return null;
   }

   public default void destroy()
   {
      
   }
}
