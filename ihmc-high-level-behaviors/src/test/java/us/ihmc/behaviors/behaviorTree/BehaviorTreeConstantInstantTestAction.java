package us.ihmc.behaviors.behaviorTree;

public class BehaviorTreeConstantInstantTestAction extends LegacyBehaviorTreeNodeState
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
