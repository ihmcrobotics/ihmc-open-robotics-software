package us.ihmc.behaviors;

import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeControlFlowNodeBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public interface BehaviorInterface extends BehaviorTreeControlFlowNodeBasics
{
   default YoRegistry getYoRegistry()
   {
      return null;
   }

   default void destroy()
   {
      
   }
}
