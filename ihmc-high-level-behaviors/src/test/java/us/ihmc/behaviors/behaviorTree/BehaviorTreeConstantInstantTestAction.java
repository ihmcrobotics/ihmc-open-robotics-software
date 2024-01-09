package us.ihmc.behaviors.behaviorTree;

public class BehaviorTreeConstantInstantTestAction extends LocalOnlyBehaviorTreeNodeExecutor
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
   public BehaviorTreeNodeStatus determineStatus()
   {
      action.run();
      return status;
   }
}
