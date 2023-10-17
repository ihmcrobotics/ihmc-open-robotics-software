package us.ihmc.behaviors.behaviorTree;

import us.ihmc.log.LogTools;

public abstract class ResettingNode extends LocalOnlyBehaviorTreeNodeExecutor
{
   private boolean lastWasClock = false;
   private boolean isReset = true;

   public ResettingNode()
   {

   }

   public void clock()
   {
      if (getLastWasClock() && !isReset())
      {
         LogTools.info("Resetting {} node", getClass().getSimpleName());
         reset();
         setIsReset(true);
      }
      setLastWasClock(true);
      super.getState().clock();
   }

   public BehaviorTreeNodeStatus tick()
   {
      setLastWasClock(false);
      setIsReset(false);
      return super.tick();
   }

   public abstract void reset();

   public boolean getLastWasClock()
   {
      return lastWasClock;
   }

   public void setLastWasClock(boolean lastWasClock)
   {
      this.lastWasClock = lastWasClock;
   }

   public boolean isReset()
   {
      return isReset;
   }

   public void setIsReset(boolean isReset)
   {
      this.isReset = isReset;
   }
}
