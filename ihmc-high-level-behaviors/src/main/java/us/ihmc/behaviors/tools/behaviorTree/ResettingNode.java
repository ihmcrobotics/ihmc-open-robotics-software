package us.ihmc.behaviors.tools.behaviorTree;

public abstract class ResettingNode extends BehaviorTreeControlFlowNode implements ResettingNodeBasics
{
   private boolean lastWasClock = false;

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
}
