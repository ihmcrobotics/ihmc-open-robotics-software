package us.ihmc.behaviors.tools.behaviorTree;

public interface ResettingNodeBasics extends BehaviorTreeControlFlowNodeBasics
{
   @Override
   public default void clock()
   {
      if (getLastWasClock() && !isReset())
      {
         reset();
         setIsReset(true);
      }
      setLastWasClock(true);
      BehaviorTreeControlFlowNodeBasics.super.clock();
   }

   @Override
   public default BehaviorTreeNodeStatus tick()
   {
      setLastWasClock(false);
      setIsReset(false);
      return BehaviorTreeControlFlowNodeBasics.super.tick();
   }

   public abstract void reset();

   public abstract boolean getLastWasClock();

   public abstract void setLastWasClock(boolean lastWasClock);

   public abstract boolean isReset();

   public abstract void setIsReset(boolean isReset);
}
