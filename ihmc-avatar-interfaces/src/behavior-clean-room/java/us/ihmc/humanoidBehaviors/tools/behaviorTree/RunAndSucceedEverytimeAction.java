package us.ihmc.humanoidBehaviors.tools.behaviorTree;

import static us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus.SUCCESS;

public class RunAndSucceedEverytimeAction implements BehaviorTreeAction
{
   private final Runnable action;

   public RunAndSucceedEverytimeAction(Runnable action)
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
