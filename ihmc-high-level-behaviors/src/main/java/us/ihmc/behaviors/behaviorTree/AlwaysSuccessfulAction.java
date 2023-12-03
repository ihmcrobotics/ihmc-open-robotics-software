package us.ihmc.behaviors.behaviorTree;

/**
 * Experimental action. Not sure about this one.
 */
public class AlwaysSuccessfulAction extends LocalOnlyBehaviorTreeNodeExecutor
{
   private final Runnable action;

   public AlwaysSuccessfulAction(Runnable action)
   {
      this.action = action;
   }

   @Override
   public BehaviorTreeNodeStatus determineStatus()
   {
      action.run();
      return BehaviorTreeNodeStatus.SUCCESS;
   }
}
