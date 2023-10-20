package us.ihmc.behaviors.behaviorTree;

public abstract class ResettingNode extends BehaviorTreeControlFlowNode implements ResettingNodeBasics
{
   private boolean lastWasClock = false;
   private boolean isReset = true;

   public ResettingNode()
   {
      setType(ResettingNode.class);
   }

   @Override
   public boolean getLastWasClock()
   {
      return lastWasClock;
   }

   @Override
   public void setLastWasClock(boolean lastWasClock)
   {
      this.lastWasClock = lastWasClock;
   }

   @Override
   public boolean isReset()
   {
      return isReset;
   }

   @Override
   public void setIsReset(boolean isReset)
   {
      this.isReset = isReset;
   }
}
