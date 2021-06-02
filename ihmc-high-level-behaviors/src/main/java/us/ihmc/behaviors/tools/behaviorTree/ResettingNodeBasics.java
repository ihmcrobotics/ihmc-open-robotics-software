package us.ihmc.behaviors.tools.behaviorTree;

public interface ResettingNodeBasics extends BehaviorTreeControlFlowNodeBasics
{
   @Override
   public default void clock()
   {
      if (getLastWasClock())
      {
         reset();
      }

      setLastWasClock(true);
   }

   @Override
   public default BehaviorTreeNodeStatus tick()
   {
      setLastWasClock(false);
      return BehaviorTreeControlFlowNodeBasics.super.tick();
   }

   public abstract void reset();

   public abstract boolean getLastWasClock();

   public abstract void setLastWasClock(boolean lastWasClock);
}
