package us.ihmc.behaviors.tools.behaviorTree;

import us.ihmc.log.LogTools;

public interface ResettingNodeBasics extends BehaviorTreeControlFlowNodeBasics
{
   @Override
   default void clock()
   {
      if (getLastWasClock() && !isReset())
      {
         LogTools.info("Resetting {} node", getClass().getSimpleName());
         reset();
         setIsReset(true);
      }
      setLastWasClock(true);
      BehaviorTreeControlFlowNodeBasics.super.clock();
   }

   @Override
   default BehaviorTreeNodeStatus tick()
   {
      setLastWasClock(false);
      setIsReset(false);
      return BehaviorTreeControlFlowNodeBasics.super.tick();
   }

   void reset();

   boolean getLastWasClock();

   void setLastWasClock(boolean lastWasClock);

   boolean isReset();

   void setIsReset(boolean isReset);
}
