package us.ihmc.humanoidBehaviors.tools.behaviorTrees;

import us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeAction;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus;

public class BehaviorTreeConstantInstantTestAction implements BehaviorTreeAction
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
   public BehaviorTreeNodeStatus tick()
   {
      action.run();
      return status;
   }
}
