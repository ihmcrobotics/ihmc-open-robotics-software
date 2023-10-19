package us.ihmc.behaviors.tools.behaviorTrees;

import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeAction;
import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeStatus;

public class BehaviorTreeConstantInstantTestAction extends BehaviorTreeAction
{
   private final Runnable action;
   private BehaviorTreeNodeStatus status;

   public BehaviorTreeConstantInstantTestAction(Runnable action)
   {
      this.action = action;
   }

   public void setStatus(BehaviorTreeNodeStatus status)
   {
      this.status = status;
   }

   @Override
   public BehaviorTreeNodeStatus tickInternal()
   {
      action.run();
      return status;
   }
}
