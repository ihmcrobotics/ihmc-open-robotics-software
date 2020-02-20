package us.ihmc.humanoidBehaviors.tools.behaviorTree;

import static us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus.SUCCESS;

/**
 * non-reactive
 */
public class AlwaysSucessfulAction implements BehaviorTreeAction
{
   private final Runnable action;

   public AlwaysSucessfulAction(Runnable action)
   {
      this.action = action;
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      action.run();
      return SUCCESS;
   }

   @Override
   public void reset()
   {
   }
}
