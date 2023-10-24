package us.ihmc.behaviors;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeControlFlowNodeBasics;

public interface BehaviorInterface extends BehaviorTreeControlFlowNodeBasics
{
   default void destroy()
   {
      
   }
}
