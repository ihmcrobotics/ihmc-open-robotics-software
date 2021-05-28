package us.ihmc.behaviors;

import us.ihmc.behaviors.tools.behaviorTree.*;

public abstract class FallbackNodeBehaviorInterface extends BehaviorInterface implements BehaviorTreeControlFlowNodeBasics
{
   private final FallbackNode fallbackNode = new FallbackNode();

   @Override
   public <T extends BehaviorTreeNodeBasics> T addChild(T child)
   {
      return fallbackNode.addChild(child);
   }

   @Override
   public double evaluateUtility()
   {
      return fallbackNode.evaluateUtility();
   }

   @Override
   public void clock()
   {
      fallbackNode.clock();
   }

   @Override
   public BehaviorTreeNodeStatus tickInternal()
   {
      return fallbackNode.tickInternal();
   }
}
